/// @file aeb_tracker.cpp

#include "../include/aeb_tracker.h"
#include <tuple>

namespace aeb
{
namespace object_tracking
{

DetectedObject::DetectedObject() noexcept : DetectedObject(0, 0.0f, 0.0f) {}

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

const std::vector<DetectedObject> &
AEBObjectTracker::getObjects() const {
  return objects_;
}

size_t AEBObjectTracker::size() const { return objects_.size(); }

bool AEBObjectTracker::empty() const { return objects_.empty(); }

void AEBObjectTracker::sortByCollisionTime() {
  std::sort(objects_.begin(), objects_.end(), collisionTimeComparator);
}

void AEBObjectTracker::sortByThreatLevel() {
  std::sort(objects_.begin(), objects_.end(), threatLevelComparator);
}

void AEBObjectTracker::partialSortCriticalObjects(size_t max_objects) {
  if (objects_.empty())
    return;

  const size_t num_to_sort = std::min(max_objects, objects_.size());
  using diff_t = std::vector<DetectedObject>::difference_type;

  std::partial_sort(objects_.begin(), objects_.begin() + static_cast<diff_t>(num_to_sort),
                    objects_.end(), collisionTimeComparator);
}

void AEBObjectTracker::sortMultiCriteria() {
  std::sort(objects_.begin(), objects_.end(), multiCriteriaComparator);
}

std::vector<DetectedObject>
AEBObjectTracker::getCriticalObjects(size_t max_objects) const {
  const size_t num_objects = std::min(max_objects, objects_.size());
  using diff_t = std::vector<DetectedObject>::difference_type;
  return std::vector<DetectedObject>(objects_.begin(),
                                     objects_.begin() + static_cast<diff_t>(num_objects));
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

bool AEBObjectTracker::hasCriticalObjects(
    float threshold_seconds) const {
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

} // object_tracking
} // aeb
