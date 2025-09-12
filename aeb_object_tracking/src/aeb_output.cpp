/// @file aeb_output.cpp 

#include "aeb_output.h"
#include "aeb_tracker.h"

namespace aeb
{
namespace object_tracking
{
namespace output
{

/// @brief Run comprehensive test suite with detailed output
void AEBOutput::runAllTests() {
  std::cout << "Running AEB Object Tracking Tests with Detailed Output...\n\n";

  testBasicSorting();
  testPartialSort();
  testMultiCriteriaSort();
  testPerformance();
  testEdgeCases();
  testModernFeatures();

  std::cout << "\n✅ All tests passed with validated output!\n";
}

/// @brief Test basic collision time sorting functionality with output
/// validation
void AEBOutput::testBasicSorting() {
  std::cout << "Test 1: Basic Sorting with Output Validation\n";

  aeb::object_tracking::AEBObjectTracker tracker;
  tracker.reserveCapacity(5);

  // Add test objects with known collision times
  tracker.addObject(aeb::object_tracking::DetectedObject(1, 50.0f, -10.0f)); // TTC = 5.0s
  tracker.addObject(aeb::object_tracking::DetectedObject(2, 20.0f, -20.0f)); // TTC = 1.0s (critical!)
  tracker.addObject(aeb::object_tracking::DetectedObject(3, 100.0f, 5.0f)); // Moving away (TTC = INF)
  tracker.addObject(aeb::object_tracking::DetectedObject(4, 30.0f, -15.0f)); // TTC = 2.0s
  tracker.addObject(aeb::object_tracking::DetectedObject(5, 80.0f, -8.0f));  // TTC = 10.0s

  tracker.printObjects("Before Sorting");

  tracker.sortByCollisionTime();
  tracker.printObjects("After Sorting by Collision Time");

  // Verify sorting correctness with detailed output
  const auto &objects = tracker.getObjects();
  std::cout << "Validation Results:\n";
  std::cout << "  Most critical object ID: " << objects[0].getId()
            << " (expected: 2) ";
  assert(objects[0].getId() == 2);
  std::cout << "✓\n";

  std::cout << "  Second most critical ID: " << objects[1].getId()
            << " (expected: 4) ";
  assert(objects[1].getId() == 4);
  std::cout << "✓\n";

  std::cout << "✅ Basic sorting test passed with output validation\n\n";
}

/// @brief Test partial sort optimization for critical objects with performance
/// output
void AEBOutput::testPartialSort() {
  std::cout << "Test 2: Partial Sort with Performance Output\n";

  aeb::object_tracking::AEBObjectTracker tracker;
  tracker.reserveCapacity(20);

  // Add many objects to test partial sort performance
  std::cout << "Generating test objects...\n";
  for (int i = 1; i <= 20; ++i) {
    const float distance = 10.0f + static_cast<float>(i) * 5.0f;
    const float velocity = -5.0f - static_cast<float>(i % 10);
    tracker.addObject(aeb::object_tracking::DetectedObject(i, distance, velocity));
    std::cout << "  Object " << i << ": Distance=" << distance
              << "m, Velocity=" << velocity << "m/s\n";
  }

  std::cout << "\nPerforming partial sort for top 5 critical objects...\n";

  const auto start = std::chrono::high_resolution_clock::now();
  tracker.partialSortCriticalObjects(5);
  const auto end = std::chrono::high_resolution_clock::now();

  const auto sort_time =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "Partial sort completed in: " << sort_time.count()
            << " microseconds\n";

  const auto critical = tracker.getCriticalObjects(5);

  std::cout << "\nTop 5 Critical Objects (Validation):\n";
  for (size_t i = 0; i < critical.size(); ++i) {
    const auto &obj = critical[i];
    std::cout << "  " << (i + 1) << ". ID: " << obj.getId()
              << ", TTC: " << obj.getCollisionTime() << "s"
              << ", Distance: " << obj.getDistance() << "m"
              << ", Threat: " << obj.getThreatLevel() << "\n";
  }

  // Verify the critical objects are properly sorted
  std::cout << "Sorting validation: ";
  assert(critical.size() == 5);
  for (size_t i = 1; i < critical.size(); ++i) {
    assert(critical[i - 1].getCollisionTime() <=
           critical[i].getCollisionTime());
  }
  std::cout << "✓ All objects properly ordered by collision time\n";

  std::cout << "✅ Partial sort test passed with performance output\n\n";
}

/// @brief Test multi-criteria sorting algorithm with detailed analysis
void AEBOutput::testMultiCriteriaSort() {
  std::cout << "Test 3: Multi-Criteria Sort with Detailed Analysis\n";

  aeb::object_tracking::AEBObjectTracker tracker;
  tracker.reserveCapacity(4);

  // Add objects with varying threat characteristics
  std::cout << "Adding objects with varying threat profiles:\n";
  tracker.addObject(
      aeb::object_tracking::DetectedObject(1, 15.0f, -15.0f)); // High threat, close, fast approach
  std::cout << "  Object 1: Close & Fast (15m, -15m/s)\n";

  tracker.addObject(
      aeb::object_tracking::DetectedObject(2, 80.0f, -2.0f)); // Low threat, far, slow approach
  std::cout << "  Object 2: Far & Slow (80m, -2m/s)\n";

  tracker.addObject(aeb::object_tracking::DetectedObject(3, 25.0f, -12.0f)); // Medium threat
  std::cout << "  Object 3: Medium threat (25m, -12m/s)\n";

  tracker.addObject(
      aeb::object_tracking::DetectedObject(4, 10.0f, -20.0f)); // Very high threat, very close
  std::cout << "  Object 4: Very Close & Very Fast (10m, -20m/s)\n";

  tracker.printObjects("Before Multi-Criteria Sort");

  const auto start = std::chrono::high_resolution_clock::now();
  tracker.sortMultiCriteria();
  const auto end = std::chrono::high_resolution_clock::now();

  const auto sort_time =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  tracker.printObjects("After Multi-Criteria Sort");

  const auto &objects = tracker.getObjects();

  std::cout << "\nMulti-Criteria Analysis:\n";
  std::cout << "  Sort time: " << sort_time.count() << " microseconds\n";
  std::cout << "  Highest priority object: ID " << objects[0].getId()
            << " (Threat: " << objects[0].getThreatLevel() << ")\n";
  std::cout << "  Second priority object: ID " << objects[1].getId()
            << " (Threat: " << objects[1].getThreatLevel() << ")\n";

  // Verify highest threat is prioritized
  std::cout << "Threat level validation: ";
  assert(objects[0].getThreatLevel() >= objects[1].getThreatLevel());
  std::cout << "✓ Highest threat properly prioritized\n";

  std::cout << "✅ Multi-criteria sort test passed with detailed analysis\n\n";
}

/// @brief Performance benchmark comparing full sort vs partial sort with
/// detailed metrics
void AEBOutput::testPerformance() {
  std::cout << "Test 4: Performance Comparison with Detailed Metrics\n";

  // Initialize random number generation for realistic data
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist_range(5.0f, 200.0f);
  std::uniform_real_distribution<float> vel_range(-25.0f, 10.0f);

  std::cout << "Generating " << kPerformanceTestSize
            << " random objects for performance testing...\n";

  // Test full sort performance with large dataset
  {
    aeb::object_tracking::AEBObjectTracker tracker;
    tracker.reserveCapacity(kPerformanceTestSize);

    // Generate test data
    for (size_t i = 0; i < kPerformanceTestSize; ++i) {
      tracker.addObject(
          aeb::object_tracking::DetectedObject(static_cast<int>(i), dist_range(gen), vel_range(gen)));
    }

    std::cout << "Testing full sort (std::sort)...\n";

    // Benchmark full sort
    const auto start = std::chrono::high_resolution_clock::now();
    tracker.sortByCollisionTime();
    const auto end = std::chrono::high_resolution_clock::now();

    const auto full_sort_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Test partial sort performance with same dataset size
    aeb::object_tracking::AEBObjectTracker tracker2;
    tracker2.reserveCapacity(kPerformanceTestSize);

    for (size_t i = 0; i < kPerformanceTestSize; ++i) {
      tracker2.addObject(
          aeb::object_tracking::DetectedObject(static_cast<int>(i), dist_range(gen), vel_range(gen)));
    }

    std::cout << "Testing partial sort (top 10 objects)...\n";

    // Benchmark partial sort
    const auto start2 = std::chrono::high_resolution_clock::now();
    tracker2.partialSortCriticalObjects(10);
    const auto end2 = std::chrono::high_resolution_clock::now();

    const auto partial_sort_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);

    // Report detailed performance results
    std::cout << "\n📊 Performance Results for " << kPerformanceTestSize
              << " objects:\n";
    std::cout << "  Full Sort (std::sort):     " << full_sort_time.count()
              << " μs\n";
    std::cout << "  Partial Sort (top 10):    " << partial_sort_time.count()
              << " μs\n";
    std::cout << "  Speedup Factor:           "
              << static_cast<double>(full_sort_time.count()) /
                     static_cast<double>(partial_sort_time.count())
              << "x\n";
    std::cout << "  Memory Usage (estimated): "
              << (kPerformanceTestSize * sizeof(aeb::object_tracking::DetectedObject)) / 1024
              << " KB\n";

    // Real-time performance validation
    std::cout << "\n🚗 Real-time Performance Validation:\n";
    const float latency_ms = static_cast<float>(partial_sort_time.count()) / 1000.0f;
    std::cout << "  Partial sort latency:     " << latency_ms << " ms\n";

    if (latency_ms < 10.0f) {
      std::cout << "  ✅ PASS: Meets real-time requirement (< 10ms)\n";
    } else {
      std::cout << "  ❌ FAIL: Exceeds real-time requirement (< 10ms)\n";
    }

  } // RAII cleanup

  std::cout << "✅ Performance test completed with detailed metrics\n\n";
}

/// @brief Test edge cases and boundary conditions with detailed validation
/// output
void AEBOutput::testEdgeCases() {
  std::cout << "Test 5: Edge Cases with Detailed Validation\n";

  aeb::object_tracking::AEBObjectTracker tracker;

  std::cout << "Testing empty container operations...\n";
  // Test empty container operations
  tracker.sortByCollisionTime();
  std::cout << "  Empty sort: ";
  assert(tracker.getObjects().empty());
  assert(tracker.empty());
  assert(tracker.size() == 0);
  std::cout << "✓ No crash, proper empty state\n";

  std::cout << "\nTesting single object scenario...\n";
  // Test single object scenario
  tracker.addObject(aeb::object_tracking::DetectedObject(1, 50.0f, -10.0f));
  tracker.sortByCollisionTime();
  std::cout << "  Single object sort: ";
  assert(tracker.getObjects().size() == 1);
  assert(!tracker.empty());
  assert(tracker.size() == 1);
  std::cout << "✓ Proper handling\n";

  std::cout << "\nTesting tie-breaking scenario...\n";
  // Test objects with identical collision times (tie-breaking)
  tracker.clear();
  tracker.addObject(aeb::object_tracking::DetectedObject(1, 20.0f, -10.0f)); // TTC = 2.0s
  tracker.addObject(aeb::object_tracking::DetectedObject(2, 40.0f, -20.0f)); // TTC = 2.0s

  std::cout << "  Objects with same TTC (2.0s):\n";
  std::cout << "    Object 1: 20m distance\n";
  std::cout << "    Object 2: 40m distance\n";

  tracker.sortByCollisionTime();

  const auto &objects = tracker.getObjects();
  std::cout << "  Tie-breaking result: Object " << objects[0].getId()
            << " comes first (distance: " << objects[0].getDistance() << "m)\n";
  std::cout << "  Validation: ";
  assert(objects[0].getDistance() <=
         objects[1].getDistance()); // Tie-breaker by distance
  std::cout << "✓ Closer object prioritized correctly\n";

  std::cout << "✅ Edge cases test passed with detailed validation\n\n";
}

/// @brief Test modern C++ algorithm integration and query functions with result
/// verification
void AEBOutput::testModernFeatures() {
  std::cout << "Test 6: Modern C++ Features with Result Verification\n";

  aeb::object_tracking::AEBObjectTracker tracker;

  std::cout << "Setting up test scenario...\n";
  tracker.addObject(
      aeb::object_tracking::DetectedObject(1, 15.0f, -20.0f)); // Critical object (TTC = 0.75s)
  tracker.addObject(
      aeb::object_tracking::DetectedObject(2, 50.0f, -5.0f)); // Less critical (TTC = 10s)
  tracker.addObject(aeb::object_tracking::DetectedObject(3, 100.0f, 2.0f)); // Moving away (TTC = INF)

  std::cout << "  Object 1: Critical (15m, -20m/s, TTC ≈ 0.75s)\n";
  std::cout << "  Object 2: Moderate (50m, -5m/s, TTC = 10s)\n";
  std::cout << "  Object 3: Safe (100m, +2m/s, moving away)\n";

  // Test critical object detection algorithms
  std::cout << "\nTesting critical object detection...\n";
  const bool has_critical = tracker.hasCriticalObjects(2.0f);
  std::cout << "  Objects within 2s threshold: ";
  assert(has_critical);
  std::cout << "✓ Critical objects detected\n";

  const auto critical_within_threshold =
      tracker.getObjectsWithinTimeThreshold(2.0f);
  std::cout << "  Number of critical objects: "
            << critical_within_threshold.size();
  assert(!critical_within_threshold.empty());
  std::cout << " ✓ Non-empty result\n";

  // Test object search functionality
  std::cout << "\nTesting object search functionality...\n";

  // Use explicit type instead of auto
  std::vector<aeb::object_tracking::DetectedObject>::const_iterator found = tracker.findObjectById(1);
  std::cout << "  Search for Object ID 1: ";
  assert(found != tracker.getObjects().end());
  assert(found->getId() == 1);
  std::cout << "✓ Found successfully\n";

  std::vector<aeb::object_tracking::DetectedObject>::const_iterator not_found =
      tracker.findObjectById(999);
  std::cout << "  Search for non-existent ID 999: ";
  assert(not_found == tracker.getObjects().end());
  std::cout << "✓ Correctly returns end iterator\n";

  // Test advanced query features
  std::cout << "\nTesting advanced query features...\n";
  const auto very_critical = tracker.getObjectsWithinTimeThreshold(1.0f);
  std::cout << "  Objects within 1s (very critical): " << very_critical.size()
            << " objects\n";

  const auto all_critical = tracker.getObjectsWithinTimeThreshold(15.0f);
  std::cout << "  Objects within 15s (all approaching): " << all_critical.size()
            << " objects\n";

  std::cout
      << "✅ Modern C++ features test passed with result verification\n\n";
}

} // output
} // object_tracking
} // aeb
