/// \file aeb_tracker.h
/// @brief AEB object tracking system header.
/// @details Defines DetectedObject and AEBObjectTracker classes for
/// collision risk analysis in autonomous emergency braking systems.

#ifndef AEB_OBJECT_TRACKING_INCLUDE_AEB_TRACKER_H
#define AEB_OBJECT_TRACKING_INCLUDE_AEB_TRACKER_H

#include <bits/std_abs.h>  // for abs
#include <cmath>           // for isinf
#include <cstddef>         // for size_t
#include <string>          // for allocator, string
#include <vector>          // for vector

namespace aeb {
namespace object_tracking {

/// @brief Detected objects for Autonomous Emergency Braking and Collision
/// Warning Systems (AEB/CW).
/// @details This class represents an object detected by AEB tracking system,
/// encapsulating its unique identifier, distance, relative velocity, estimated
/// collision time, and calculated threat level.
///
/// The threat level is computed based on object dynamics and is used to assess
/// collision risk.
///
class DetectedObject {
public:
  // Constructors
  DetectedObject(int obj_id, float dist, float rel_vel) noexcept;
  DetectedObject() noexcept;

  // Getters
  constexpr int getId() const { return id_; }
  constexpr float getDistance() const { return distance_; }
  constexpr float getRelativeVelocity() const { return relative_velocity_; }
  constexpr float getCollisionTime() const { return collision_time_; }
  constexpr float getThreatLevel() const { return threat_level_; }

  // Comparison operators for sorting
  bool operator<(const DetectedObject &other) const noexcept;
  bool operator==(const DetectedObject &other) const noexcept;

private:
  int id_;
  float distance_;          // meters
  float relative_velocity_; // m/s (negative = approaching)
  float collision_time_;    // seconds (calculated TTC)
  float threat_level_;      // 0.0 to 1.0

  constexpr float calculateThreatLevel() const noexcept;
};

/// @brief AEB Object Tracking System.
/// @details Main class for managing detected objects and performing collision
/// risk analysis.
/// It uses introsort (std::sort) for full sorting and partial sort
/// for optimization.
///
class AEBObjectTracker {
public:
  struct Comparators {
    /// @brief Comparator for sorting DetectedObject by collision time.
    /// Handles infinity values and uses distance as tie-breaker.
    /// @param first_object First object to compare.
    /// @param second_object Second object to compare.
    /// @return True if first_object is more critical (smaller collision time or
    /// distance).
    ///
    static bool byCollisionTime(DetectedObject const &first_object,
                                DetectedObject const &second_object);

    /// @brief Comparator for sorting DetectedObject by threat level.
    /// Uses distance as a tie-breaker for equal threat levels.
    /// @param first_object First DetectedObject to compare.
    /// @param second_object Second DetectedObject to compare.
    /// @return true if first_object has higher threat level, false otherwise.
    ///
    static bool byThreatLevel(DetectedObject const &first_object,
                              DetectedObject const &second_object);
  };

  /// @brief Add a detected object to the tracking system.
  /// @param object DetectedObject to add.
  void addObject(DetectedObject const &object);

  /// @brief Reserve memory capacity for objects (performance optimization).
  /// @param capacity Number of objects to reserve space for.
  void reserveCapacity(std::size_t capacity);

  /// @brief Clear all tracked objects.
  void clear() noexcept;

  /// @brief Get reference to all tracked objects.
  /// @return Const reference to object vector.
  std::vector<DetectedObject> const &getObjects() const;

  /// @brief Get number of tracked objects.
  /// @return Number of objects.
  std::size_t size() const;

  /// @brief Check if tracker is empty.
  /// @return true if no objects are tracked.
  bool empty() const;

  /// @brief Sort all objects by collision time (full sort using introsort).
  /// Time complexity: O(n log n), Space: O(log n).
  void sortByCollisionTime();

  /// @brief Sort all objects by threat level (full sort using introsort).-
  /// Time complexity: O(n log n), Space: O(log n).
  void sortByThreatLevel();

  /// @brief Get only the n most critical objects by collision time.
  /// Time complexity: O(n log k) where k = max_objects, Space: O(1).
  /// @param max_objects Maximum number of critical objects to sort (default: 5)
  ///
  /// TODO: Remove default argument from getCriticalObjects.
  ///
  void
  partialSortCriticalObjects(std::size_t max_objects = kMaxCriticalObjects);

  /// @brief Multi-criteria sort (full sort using introsort - std::sort),
  /// combining threat level, collision time, and distance.
  /// Time complexity: O(n log n), Space: O(log n).
  void sortMultiCriteria();

  /// @brief Get the most critical objects (assumes partialSortCriticalObjects
  /// was called).
  /// @param max_objects Maximum number of objects to return (default: 5).
  ///
  /// TODO: Remove default argument from getCriticalObjects.
  ///
  /// @return Vector of critical objects.
  std::vector<DetectedObject>
  getCriticalObjects(std::size_t max_objects = kMaxCriticalObjects) const;

  /// @brief Get objects within critical collision time threshold.
  /// @param threshold_seconds Time threshold in seconds.
  /// @return Vector of objects within threshold.
  std::vector<DetectedObject>
  getObjectsWithinTimeThreshold(float threshold_seconds) const;

  /// @brief Find object by ID.
  /// @param id Object ID to search for.
  /// @return Iterator to found object or end() if not found.
  auto findObjectById(int id) const noexcept
      -> std::vector<DetectedObject>::const_iterator;

  /// @brief Check if any object has critical collision time.
  /// @param threshold_seconds Critical time threshold in seconds
  /// (default: 2.0s).
  /// @return true if any object is within critical threshold
  ///
  /// TODO: Remove default argument from hasCriticalObjects.
  /// To enforce the caller to consciously choose a threshold,
  /// making the code's intent clearer and reducing the risk of accidental
  /// misuse.
  ///
  bool hasCriticalObjects(float threshold_seconds = 2.0f) const;

  /// @brief Print objects for debugging.
  /// @param title Optional title for the output.
  void printObjects(std::string const &title = "") const;

private:
  std::vector<DetectedObject> objects_; ///< Container for detected objects

  static constexpr std::size_t kMaxCriticalObjects =
      5U; ///< Default maximum critical objects to track.

  // /// @brief Comparator for sorting by collision time.
  // /// Handles infinity values and uses distance as tie-breaker.
  // static constexpr auto collisionTimeComparator =
  //         [](DetectedObject const& first_object,
  //             DetectedObject const& second_object) noexcept -> bool {
  //     const float first_collision_time = first_object.getCollisionTime();
  //     const float second_collision_time = second_object.getCollisionTime();

  //     // Handle infinity values
  //     if (std::isinf(first_collision_time) &&
  //     std::isinf(second_collision_time)) {
  //         return first_object.getDistance() < second_object.getDistance();
  //     }
  //     if (std::isinf(first_collision_time))
  //         return false;
  //     if (std::isinf(second_collision_time))
  //         return true;

  //     return first_collision_time < second_collision_time;
  // };

  // /// @brief Comparator for sorting by threat level
  // /// Uses distance as tie-breaker for equal threat levels
  // static constexpr auto threatLevelComparator =
  //     [](const DetectedObject &first_object,
  //        const DetectedObject &second_object) noexcept -> bool {
  //   constexpr float epsilon = 0.001f;
  //   if (std::abs(first_object.getThreatLevel() -
  //                second_object.getThreatLevel()) < epsilon) {
  //     return first_object.getDistance() < second_object.getDistance();
  //   }
  //   return first_object.getThreatLevel() > second_object.getThreatLevel();
  // };

  /// @brief Multi-criteria comparator combining threat level, collision time,
  /// and distance Primary: threat level (higher first), Secondary: collision
  /// time (lower first), Tertiary: distance (closer first)
  static constexpr auto multiCriteriaComparator =
      [](const DetectedObject &first_object,
         const DetectedObject &second_object) noexcept -> bool {
    constexpr float threat_epsilon = 0.01f;
    constexpr float time_epsilon = 0.1f;

    // Primary: threat level (higher first)
    if (std::abs(first_object.getThreatLevel() -
                 second_object.getThreatLevel()) > threat_epsilon) {
      return first_object.getThreatLevel() > second_object.getThreatLevel();
    }

    // Secondary: collision time (lower first)
    const float first_collision_time = first_object.getCollisionTime();
    const float second_collision_time = second_object.getCollisionTime();

    if (!std::isinf(first_collision_time) &&
        !std::isinf(second_collision_time)) {
      if (std::abs(first_collision_time - second_collision_time) >
          time_epsilon) {
        return first_collision_time < second_collision_time;
      }
    }

    // Tertiary: distance (closer first)
    return first_object.getDistance() < second_object.getDistance();
  };
};

/// @brief Demonstration function for AEB system
/// Shows practical usage of the tracking system in a traffic scenario
void demonstrateAEBSystem();

} // namespace object_tracking
} // namespace aeb

#endif // AEB_OBJECT_TRACKING_INCLUDE_AEB_TRACKER_H