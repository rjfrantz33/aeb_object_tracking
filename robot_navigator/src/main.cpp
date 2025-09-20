/******************************************************************************
main.cpp - Robot Navigator Main Application
MISRA C++ 2023 Compliant Application Entry Point
*******************************************************************************/

#include "robot_navigator/include/robot_navigator.h"

int main() {
    try {
        // Print header
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "    ROBOT NAVIGATOR - MISRA C++ 2023 COMPLIANT SYSTEM\n";
        std::cout << std::string(70, '=') << "\n";
        
        // ====================================================================
        // DEMONSTRATION 1: BASIC EXAMPLE (DEFAULT START POSITION)
        // ====================================================================
        
        std::cout << "\n" << std::string(60, '-') << "\n";
        std::cout << "DEMONSTRATION 1: BASIC NAVIGATION EXAMPLE\n";
        std::cout << std::string(60, '-') << "\n";
        
        // Create robot at default position (5,5) facing North
        robot_nav::Robot robot;
        
        // Parse and execute instructions
        std::string instruction_string = "R2,L3,L1";
        auto instructions = robot_nav::parseInstructions(instruction_string);
        robot.executeInstructions(instructions);
        
        // Print detailed analysis
        robot_nav::printAnalysis(robot, instructions);
        
        // ====================================================================
        // DEMONSTRATION 2: CUSTOM START POSITION
        // ====================================================================
        
        std::cout << "\n\n" << std::string(60, '-') << "\n";
        std::cout << "DEMONSTRATION 2: CUSTOM START POSITION EXAMPLE\n";
        std::cout << std::string(60, '-') << "\n";
        
        // Create robot at custom position
        robot_nav::Position custom_start(2, 7);
        robot_nav::Robot custom_robot(custom_start, robot_nav::Direction::kEast);
        
        // Different instruction set
        std::string custom_instructions = "R1,L2,R3,L1";
        auto custom_instruction_list = robot_nav::parseInstructions(custom_instructions);
        custom_robot.executeInstructions(custom_instruction_list);
        
        // Print detailed analysis
        robot_nav::printAnalysis(custom_robot, custom_instruction_list);
        
        // ====================================================================
        // DEMONSTRATION 3: EFFICIENCY COMPARISON
        // ====================================================================
        
        std::cout << "\n\n" << std::string(60, '-') << "\n";
        std::cout << "DEMONSTRATION 3: EFFICIENCY COMPARISON\n";
        std::cout << std::string(60, '-') << "\n";
        
        // Compare different paths to same destination
        std::vector<std::pair<std::string, std::string>> efficiency_tests = {
            {"Direct Path", "R2,R2"},           // More efficient
            {"Indirect Path", "R1,R1,R1,R1"},   // Less efficient
            {"Complex Path", "R1,L1,R1,L1,R2"}  // Very inefficient
        };
        
        for (const auto& test : efficiency_tests) {
            robot_nav::Robot test_robot;  // Default start (5,5)
            auto test_instructions = robot_nav::parseInstructions(test.second);
            test_robot.executeInstructions(test_instructions);
            
            std::cout << "\n" << test.first << " (" << test.second << "):\n";
            std::cout << "  Final Position: " << test_robot.getCurrentPosition().toString() << "\n";
            std::cout << "  Steps Taken: " << test_robot.getActualSteps() << "\n";
            std::cout << "  Manhattan Distance: " << test_robot.getManhattanDistance() << "\n";
            std::cout << "  Efficiency: " << std::fixed << std::setprecision(1) 
                      << test_robot.getEfficiencyPercent() << "%\n";
        }
        
        // ====================================================================
        // DEMONSTRATION 4: INTERACTIVE MODE
        // ====================================================================
        
        std::cout << "\n\n" << std::string(60, '-') << "\n";
        std::cout << "DEMONSTRATION 4: INTERACTIVE MODE\n";
        std::cout << std::string(60, '-') << "\n";
        
        std::cout << "\nEnter custom instructions (format: R2,L3,R1) or press Enter to skip:\n> ";
        std::string user_input;
        std::getline(std::cin, user_input);
        
        if (!user_input.empty()) {
            try {
                robot_nav::Robot interactive_robot;
                auto interactive_instructions = robot_nav::parseInstructions(user_input);
                interactive_robot.executeInstructions(interactive_instructions);
                
                std::cout << "\nYour Custom Navigation:\n";
                robot_nav::printAnalysis(interactive_robot, interactive_instructions);
            } catch (const std::exception& e) {
                std::cout << "Error parsing instructions: " << e.what() << "\n";
            }
        } else {
            std::cout << "Skipping interactive mode.\n";
        }
        
        // ====================================================================
        // COMPREHENSIVE TEST SUITE
        // ====================================================================
        
        std::cout << "\n\n" << std::string(60, '-') << "\n";
        std::cout << "RUNNING COMPREHENSIVE TEST SUITE\n";
        std::cout << std::string(60, '-') << "\n";
        
        robot_nav::runTestSuite();
        
        // ====================================================================
        // FINAL SUMMARY
        // ====================================================================
        
        std::cout << "\n\n" << std::string(70, '=') << "\n";
        std::cout << "                    PROGRAM SUMMARY\n";
        std::cout << std::string(70, '=') << "\n";
        
        std::cout << "\nRobot Navigator Features Demonstrated:\n";
        std::cout << "  ✓ MISRA C++ 2023 Compliant Implementation\n";
        std::cout << "  ✓ Grid-Based Navigation System (10x10 grid)\n";
        std::cout << "  ✓ Position Tracking with Path History\n";
        std::cout << "  ✓ Direction Management (North, East, South, West)\n";
        std::cout << "  ✓ Instruction Parsing (L/R + steps format)\n";
        std::cout << "  ✓ Boundary Checking with Clamping\n";
        std::cout << "  ✓ Manhattan Distance Calculations\n";
        std::cout << "  ✓ Efficiency Analysis and Reporting\n";
        std::cout << "  ✓ Comprehensive Test Suite\n";
        std::cout << "  ✓ Interactive Mode Support\n";
        
        std::cout << "\nSafety Features:\n";
        std::cout << "  ✓ Boundary clamping prevents out-of-grid movement\n";
        std::cout << "  ✓ Exception handling for invalid inputs\n";
        std::cout << "  ✓ noexcept specifications for critical functions\n";
        std::cout << "  ✓ Strong type safety with enum classes\n";
        std::cout << "  ✓ Immutable position and instruction objects\n";
        
        std::cout << "\nGrid Coordinate System:\n";
        std::cout << "  • Origin (0,0) at top-left corner\n";
        std::cout << "  • X increases rightward (East)\n";
        std::cout << "  • Y increases downward (South)  \n";
        std::cout << "  • Default start position: (5,5) facing North\n";
        std::cout << "  • Grid size: 10x10 (0-9 in both dimensions)\n";
        
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "Program completed successfully!\n";
        std::cout << std::string(70, '=') << "\n\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred\n";
        return 1;
    }
}
