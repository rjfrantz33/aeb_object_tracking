/// @file aeb_output.h
/// @brief Test output and validation suite for AEB Object Tracking System
/// @details Comprehensive testing including performance benchmarks and result
/// validation

#ifndef AEB_OBJECT_TRACKING_INCLUDE_AEB_OUTPUT_H
#define AEB_OBJECT_TRACKING_INCLUDE_AEB_OUTPUT_H

#include <cassert>
#include <chrono>
#include <random>

namespace aeb
{
namespace object_tracking
{
namespace output
{

/// @brief Test output and validation class for AEB Object Tracking System
/// @details Provides comprehensive testing including performance analysis,
/// edge case validation, and algorithm correctness verification with detailed
/// output
class AEBOutput {
private:
  static constexpr size_t kPerformanceTestSize =
      10000; ///< Number of objects for performance testing

public:
  /// @brief Run all test suites with detailed output
  /// Executes all available tests and reports results
  static void runAllTests();

private:
  /// @brief Test basic sorting functionality with output validation
  /// @details Verifies that objects are correctly sorted by collision time
  /// Tests introsort (std::sort) implementation
  static void testBasicSorting();

  /// @brief Test partial sort optimization with performance output
  /// @details Verifies that partial sort correctly identifies most critical
  /// objects Tests performance optimization for real-time systems
  static void testPartialSort();

  /// @brief Test multi-criteria sorting with detailed analysis
  /// @details Verifies complex sorting combining threat level, collision time,
  /// and distance Tests multi-factor decision making algorithm
  static void testMultiCriteriaSort();

  /// @brief Performance comparison between full and partial sort with metrics
  /// @details Benchmarks sorting algorithms with large datasets
  /// Measures execution time and calculates speedup ratios
  static void testPerformance();

  /// @brief Test edge cases and boundary conditions with validation output
  /// @details Tests empty containers, single objects, and tie-breaking
  /// scenarios Ensures robustness in unusual conditions
  static void testEdgeCases();

  /// @brief Test modern C++ features and algorithms with result verification
  /// @details Verifies STL algorithm integration and query functions
  /// Tests find operations, filtering, and boolean queries
  static void testModernFeatures();
};

} // output
} // object_tracking
} // aeb

#endif // AEB_OBJECT_TRACKING_INCLUDE_AEB_OUTPUT_H
