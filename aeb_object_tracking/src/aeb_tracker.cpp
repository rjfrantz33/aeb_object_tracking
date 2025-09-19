/// @file aeb_tracker.cpp

#include "../include/aeb_tracker.h"
#include <algorithm>  // for sort, max, min, any_of, copy_if, find_if, parti...
#include <iomanip>    // for operator<<, setprecision
#include <iostream>   // for basic_ostream, operator<<, cout, basic_ios, bas...
#include <iterator>   // for back_insert_iterator, back_inserter
#include <limits>     // for numeric_limits

namespace aeb {
namespace object_tracking {

DetectedObject::DetectedObject() noexcept : DetectedObject(0, 0.0f, 0.0f) {}

bool AEBObjectTracker::Comparators::byCollisionTime(
    DetectedObject const &first_object, DetectedObject const &second_object) {
  auto const first_collision_time = first_object.getCollisionTime();
  auto const second_collision_time = second_object.getCollisionTime();

  // Handle infinite values: finite collision times are always "less" (more
  // critical)
  if (std::isinf(first_collision_time) && std::isinf(second_collision_time)) {
    // If both objects have infinite collision time, sort by distance (closer is
    // more relevant)
    return first_object.getDistance() < second_object.getDistance();
  }
  // If first object has infinite collision time, it's less critical than the
  // second
  if (std::isinf(first_collision_time)) {
    return false;
  }
  // If second object has infinite collision time, first is more critical
  if (std::isinf(second_collision_time)) {
    return true;
  }
  // Otherwise, compare by collision time (smaller is more critical)
  return first_collision_time < second_collision_time;
}

bool AEBObjectTracker::Comparators::byThreatLevel(
    DetectedObject const &first_object, DetectedObject const &second_object) {
  constexpr float kThreatLevelFactor{0.001f};

  if (std::abs(first_object.getThreatLevel() - second_object.getThreatLevel()) <
      kThreatLevelFactor) {
    return first_object.getDistance() < second_object.getDistance();
  }

  return first_object.getThreatLevel() > second_object.getThreatLevel();
}

// DetectedObject Implementation
constexpr float DetectedObject::calculateThreatLevel() const noexcept {
  if (collision_time_ > 10.0f)
    return 0.0f;
  if (collision_time_ < 1.0f)
    return 1.0f;

  const float distance_factor = std::max(0.0f, 1.0f - distance_ / 100.0f);
  const float time_factor = std::max(0.0f, 1.0f - collision_time_ / 10.0f);
  return (distance_factor + time_factor) / 2.0f;
}

DetectedObject::DetectedObject(int obj_id, float dist, float rel_vel) noexcept
    : id_{obj_id}, distance_{dist}, relative_velocity_{rel_vel} {
  // Calculate Time-To-Collision (TTC)
  // Formula: TTC = distance / |relative_velocity|
  // relative_velocity is negative, i.e., object is approaching.
  collision_time_ = (relative_velocity_ < -0.1f)
                        ? distance_ / (-relative_velocity_)
                        : std::numeric_limits<float>::infinity();

  // Calculate threat level based on distance and TTC
  threat_level_ = calculateThreatLevel();
}

bool DetectedObject::operator<(const DetectedObject &other) const noexcept {
  return collision_time_ < other.collision_time_;
}

bool DetectedObject::operator==(const DetectedObject &other) const noexcept {
  return id_ == other.id_;
}

/// @brief AEBObjectTracker Implementation
void AEBObjectTracker::addObject(const DetectedObject &object) {
  objects_.push_back(object);
}

void AEBObjectTracker::reserveCapacity(size_t capacity) {
  objects_.reserve(capacity);
}

void AEBObjectTracker::clear() noexcept { objects_.clear(); }

const std::vector<DetectedObject> &AEBObjectTracker::getObjects() const {
  return objects_;
}

size_t AEBObjectTracker::size() const { return objects_.size(); }

bool AEBObjectTracker::empty() const { return objects_.empty(); }

void AEBObjectTracker::sortByCollisionTime() {
  std::sort(objects_.begin(), objects_.end(), Comparators::byCollisionTime);
}

void AEBObjectTracker::sortByThreatLevel() {
  std::sort(objects_.begin(), objects_.end(), Comparators::byCollisionTime);
}

void AEBObjectTracker::partialSortCriticalObjects(size_t max_objects) {
  if (objects_.empty())
    return;

  const size_t num_to_sort = std::min(max_objects, objects_.size());
  using diff_t = std::vector<DetectedObject>::difference_type;

  std::partial_sort(objects_.begin(),
                    objects_.begin() + static_cast<diff_t>(num_to_sort),
                    objects_.end(), Comparators::byCollisionTime);
}

void AEBObjectTracker::sortMultiCriteria() {
  std::sort(objects_.begin(), objects_.end(), multiCriteriaComparator);
}

std::vector<DetectedObject>
AEBObjectTracker::getCriticalObjects(size_t max_objects) const {
  const size_t num_objects = std::min(max_objects, objects_.size());
  using diff_t = std::vector<DetectedObject>::difference_type;
  return std::vector<DetectedObject>(
      objects_.begin(), objects_.begin() + static_cast<diff_t>(num_objects));
}

std::vector<DetectedObject>
AEBObjectTracker::getObjectsWithinTimeThreshold(float threshold_seconds) const {
  std::vector<DetectedObject> critical_objects;
  critical_objects.reserve(objects_.size());

  std::copy_if(objects_.begin(), objects_.end(),
               std::back_inserter(critical_objects),
               [threshold_seconds](const DetectedObject &obj) noexcept {
                 return !std::isinf(obj.getCollisionTime()) &&
                        obj.getCollisionTime() <= threshold_seconds;
               });

  return critical_objects;
}

std::vector<DetectedObject>::const_iterator
AEBObjectTracker::findObjectById(int id) const noexcept {
  return std::find_if(
      objects_.begin(), objects_.end(),
      [id](const DetectedObject &obj) noexcept { return obj.getId() == id; });
}

bool AEBObjectTracker::hasCriticalObjects(float threshold_seconds) const {
  return std::any_of(objects_.begin(), objects_.end(),
                     [threshold_seconds](const DetectedObject &obj) noexcept {
                       return !std::isinf(obj.getCollisionTime()) &&
                              obj.getCollisionTime() <= threshold_seconds;
                     });
}

void AEBObjectTracker::printObjects(const std::string &title) const {
  if (!title.empty()) {
    std::cout << "\n=== " << title << " ===\n";
  }

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "ID\tDist(m)\tRelVel(m/s)\tTTC(s)\tThreat\n";
  std::cout << "----------------------------------------\n";

  for (const auto &obj : objects_) {
    std::cout << obj.getId() << "\t" << obj.getDistance() << "\t"
              << obj.getRelativeVelocity() << "\t\t";

    if (std::isinf(obj.getCollisionTime())) {
      std::cout << "INF";
    } else {
      std::cout << obj.getCollisionTime();
    }

    std::cout << "\t" << obj.getThreatLevel() << "\n";
  }
}

} // namespace object_tracking
} // namespace aeb
