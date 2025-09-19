/// @file aeb_tracker_test.cpp

#include "../include/aeb_tracker.h"
#include "gtest/gtest.h"  // for AssertionResult, Test, Message, TestPartResult

/// TODO: ADD more unit tests

namespace aeb {
namespace object_tracking {
namespace test {

TEST(AEBComparators, FiniteCollitionTime) {

  aeb::object_tracking::DetectedObject object1(1, 10.0f, -5.0f);
  aeb::object_tracking::DetectedObject object2(2, 15.0f, -3.0f);

  EXPECT_TRUE(AEBObjectTracker::Comparators::byCollisionTime(object1, object2))
      << "Comparator returns true as object1 has a lower collision time (i.e., "
         "will collide sooner) than object2.";

  EXPECT_FALSE(AEBObjectTracker::Comparators::byCollisionTime(object2, object1))
      << "Comparator returns false as object2 has a higher collision time "
         "(i.e., will collide later) than object1.";
}

TEST(AEBComparators, InfiniteCollisionTime) {
  aeb::object_tracking::DetectedObject object1(1, 9.0f, 0.0f); // TTC = infinity
  aeb::object_tracking::DetectedObject object2(2, 5.0f, -3.5f); // TTC = 1.43s
  // Additional checks for both infinite
  aeb::object_tracking::DetectedObject object3(3, 12.0f,
                                               0.0f); // TTC = infinity

  EXPECT_TRUE(AEBObjectTracker::Comparators::byCollisionTime(object2, object1))
      << "Comparator returns true: object2 has a finite collision time "
         "(1.43s), object1 has infinite collision time (velocity=0).";

  EXPECT_FALSE(AEBObjectTracker::Comparators::byCollisionTime(object1, object2))
      << "Comparator returns false: object1 has infinite collision time, "
         "object2 has finite collision time (1.43s).";

  EXPECT_TRUE(AEBObjectTracker::Comparators::byCollisionTime(object1, object3))
      << "Comparator returns true: both objects have infinite collision time, "
         "so the one with the smaller distance (object1) is considered more "
         "critical.";

  EXPECT_FALSE(AEBObjectTracker::Comparators::byCollisionTime(object3, object1))
      << "Comparator returns false: both objects have infinite collision time, "
         "but object3 is further away than object1, so it is less critical.";
}

TEST(AEBComparators, FiniteCollisionThreatlevel) {
  aeb::object_tracking::DetectedObject object1(
      7, 10.0f, -8.0f); // TTC = 1.25s High Threat
  aeb::object_tracking::DetectedObject object2(
      4, 15.0f, -3.0f); // TTC = 5.0s  Medium Threat

  EXPECT_FALSE(AEBObjectTracker::Comparators::byThreatLevel(object2, object1))
      << "Comparator returns false because object2 has a lower threat level "
         "than object1.";
  EXPECT_TRUE(AEBObjectTracker::Comparators::byThreatLevel(object1, object2))
      << "Comparator returns true as object1 poses a more critical threat.";
}

TEST(AEBComparators, FiniteCollisionThreatlevelSame) {
  aeb::object_tracking::DetectedObject object1(10, 50.0f,
                                               -5.0f); // TTC = 10s Low Threat
  aeb::object_tracking::DetectedObject object2(11, 50.0f,
                                               -5.0f); // TTC = 10s Low Threat

  EXPECT_FALSE(AEBObjectTracker::Comparators::byThreatLevel(object1, object2))
      << "Low threat level, the comparator return false";
  EXPECT_FALSE(AEBObjectTracker::Comparators::byThreatLevel(object2, object1))
      << "Low threat level, the comparator return false";
}

TEST(AEBComparators, ThreatlevelBelowThreasholdUseDistanceAsTieBreaker) {
  aeb::object_tracking::DetectedObject object1(12, 10.0f, 0.0001f);
  aeb::object_tracking::DetectedObject object2(13, 5.0f, 0.0001f);

  EXPECT_FALSE(AEBObjectTracker::Comparators::byThreatLevel(object1, object2))
      << "Threat levels are nearly equal (difference below threshold); "
         "comparator returns false because object2 is closer and thus "
         "considered more critical.";

EXPECT_TRUE(AEBObjectTracker::Comparators::byThreatLevel(object2, object1))
    << "Threat levels are almost identical (difference below threshold); "
       "comparator returns true since object2 is closer, making it more critical than object1.";

}

TEST(AEBComparators, ThreatLevelZeroDistance)
{
   aeb::object_tracking::DetectedObject object1(20, 0.0f, -10.0f);
   aeb::object_tracking::DetectedObject object2(21, 0.0f, -1.0f);

   EXPECT_FALSE(AEBObjectTracker::Comparators::byThreatLevel(object1, object2));
   EXPECT_FALSE(AEBObjectTracker::Comparators::byThreatLevel(object2, object1));

}


} // namespace test
} // namespace object_tracking
} // namespace aeb
