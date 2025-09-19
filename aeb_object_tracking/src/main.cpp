/// @file main.cpp

#include <stddef.h>       // for size_t
#include <exception>      // for exception
#include <iostream>       // for operator<<, basic_ostream, cout, endl, basi...
#include <string>         // for char_traits, allocator, basic_string
#include <tuple>          // for tuple
#include <vector>         // for vector
#include "aeb_output.h"   // for AEBOutput
#include "aeb_tracker.h"  // for AEBObjectTracker, DetectedObject

namespace aeb {
namespace object_tracking {
namespace output {

/// @brief Demonstrate AEB system functionality with realistic traffic scenario
void demonstrateAEBSystem() {
  std::cout << "\n🚗 Autonomous Emergency Braking - Object Tracking Demo\n";
  std::cout << "====================================================\n";

  aeb::object_tracking::AEBObjectTracker aeb_system;
  aeb_system.reserveCapacity(10); // Optimize memory allocation

  // Simulate detected objects in traffic scenario using modern initialization
  std::cout << "\n🔍 Detected Objects in Traffic:\n";

  // Use structured bindings for cleaner code (C++17)
  const std::vector<std::tuple<int, float, float>> detected_objects = {
      {101, 45.0f, -12.0f}, // Car ahead, moderate speed
      {102, 15.0f, -25.0f}, // Emergency! Close and fast
      {103, 80.0f, -5.0f},  // Distant slow vehicle
      {104, 25.0f, -18.0f}, // Another concerning object
      {105, 120.0f, 8.0f},  // Vehicle moving away
      {106, 35.0f, -8.0f}   // Moderate threat
  };

  // Add objects using simple loop with explicit construction
  for (const auto &[id, distance, velocity] : detected_objects) {
    aeb_system.addObject(
        aeb::object_tracking::DetectedObject(id, distance, velocity));
  }

  aeb_system.printObjects("All Detected Objects");

  // Get critical objects for immediate action
  std::cout << "\n🚨 Analyzing Critical Objects (Partial Sort)...\n";
  aeb_system.partialSortCriticalObjects(3);
  const auto critical_objects = aeb_system.getCriticalObjects(3);

  std::cout << "\nTop 3 Critical Objects requiring immediate attention:\n";
  for (size_t i = 0; i < critical_objects.size(); ++i) {
    const auto &obj = critical_objects[i];
    std::cout << (i + 1) << ". Object ID " << obj.getId()
              << " - Distance: " << obj.getDistance() << "m"
              << ", TTC: " << obj.getCollisionTime() << "s"
              << ", Threat: " << obj.getThreatLevel() << std::endl;
  }

  // Decision making based on critical objects using modern algorithms
  constexpr float kCriticalTimeThreshold = 2.0f;
  constexpr float kWarningTimeThreshold = 5.0f;

  if (aeb_system.hasCriticalObjects(kCriticalTimeThreshold)) {
    std::cout
        << "\n⚠️  CRITICAL: Collision imminent! Applying emergency braking!\n";
  } else if (aeb_system.hasCriticalObjects(kWarningTimeThreshold)) {
    std::cout << "\n⚠️  WARNING: Close object detected. Pre-charging brakes.\n";
  } else {
    std::cout << "\n✅ All clear. Normal driving conditions.\n";
  }

  // Demonstrate advanced queries
  const auto objects_within_2s = aeb_system.getObjectsWithinTimeThreshold(2.0f);
  std::cout << "\nObjects within 2-second collision threshold: "
            << objects_within_2s.size() << std::endl;
}

} // namespace output
} // namespace object_tracking
} // namespace aeb

/// @brief Main application entry point
/// @return Exit status code
int main() {
  std::cout << "╔══════════════════════════════════════════════════════════╗\n";
  std::cout << "║       AEB Object Tracking System - Main Application      ║\n";
  std::cout << "║          Autonomous Emergency Braking Demo               ║\n";
  std::cout << "╚══════════════════════════════════════════════════════════╝\n";

  try {
    // Run comprehensive test suite
    std::cout << "\n📋 Running System Validation Tests...\n";
    aeb::object_tracking::output::AEBOutput::runAllTests();

    // Demonstrate the system in action
    std::cout << "\n🎮 Running Interactive Demo...\n";
    aeb::object_tracking::output::demonstrateAEBSystem();

    std::cout
        << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║                 🎉 SYSTEM READY! 🎉                      ║\n";
    std::cout
        << "║         AEB Object Tracker Operating Normally            ║\n";
    std::cout
        << "╚══════════════════════════════════════════════════════════╝\n";

    return 0;

  } catch (const std::exception &e) {
    std::cerr << "\n❌ Application failed with exception: " << e.what()
              << std::endl;
    return 1;

  } catch (...) {
    std::cerr << "\n❌ Application failed with unknown exception" << std::endl;
    return 2;
  }
}
