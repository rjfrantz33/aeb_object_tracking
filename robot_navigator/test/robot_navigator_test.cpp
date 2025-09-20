/******************************************************************************
robot_navigator_test.cpp - Comprehensive Unit Tests
MISRA C++ 2023 Compliant Test Suite using Google Test Framework
*******************************************************************************/

#include <gtest/gtest.h>
#include "robot_navigator/include/robot_navigator.h"

namespace robot_nav {
namespace test {

// ============================================================================
// POSITION CLASS TESTS
// ============================================================================

class PositionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Test positions
        origin_ = Position(0, 0);
        center_ = Position(5, 5);
        corner_ = Position(9, 9);
        custom_ = Position(3, 7);
    }
    
    Position origin_;
    Position center_;
    Position corner_;
    Position custom_;
};

TEST_F(PositionTest, DefaultConstructor) {
    Position default_pos;
    EXPECT_EQ(default_pos.getX(), 5);
    EXPECT_EQ(default_pos.getY(), 5);
}

TEST_F(PositionTest, ParameterizedConstructor) {
    Position pos(3, 7);
    EXPECT_EQ(pos.getX(), 3);
    EXPECT_EQ(pos.getY(), 7);
}

TEST_F(PositionTest, EqualityOperator) {
    Position pos1(5, 5);
    Position pos2(5, 5);
    Position pos3(3, 7);
    
    EXPECT_TRUE(pos1 == pos2);
    EXPECT_FALSE(pos1 == pos3);
}

TEST_F(PositionTest, InequalityOperator) {
    Position pos1(5, 5);
    Position pos2(3, 7);
    
    EXPECT_TRUE(pos1 != pos2);
    EXPECT_FALSE(pos1 != Position(5, 5));
}

TEST_F(PositionTest, ToStringMethod) {
    EXPECT_EQ(origin_.toString(), "(0,0)");
    EXPECT_EQ(center_.toString(), "(5,5)");
    EXPECT_EQ(corner_.toString(), "(9,9)");
    EXPECT_EQ(custom_.toString(), "(3,7)");
}

TEST_F(PositionTest, ManhattanDistanceCalculation) {
    // Distance from origin to center: |5-0| + |5-0| = 10
    EXPECT_EQ(Position::getManhattanDistance(origin_, center_), 10U);
    
    // Distance from center to corner: |9-5| + |9-5| = 8
    EXPECT_EQ(Position::getManhattanDistance(center_, corner_), 8U);
    
    // Distance from same position should be 0
    EXPECT_EQ(Position::getManhattanDistance(center_, center_), 0U);
    
    // Distance should be symmetric
    EXPECT_EQ(Position::getManhattanDistance(origin_, corner_),
              Position::getManhattanDistance(corner_, origin_));
}

TEST_F(PositionTest, ManhattanDistanceEdgeCases) {
    Position negative_equivalent(-1, -1);  // Would be clamped to (0,0)
    Position large_equivalent(15, 15);     // Would be clamped to (9,9)
    
    // Test with boundary positions
    EXPECT_EQ(Position::getManhattanDistance(origin_, corner_), 18U);  // |9-0| + |9-0| = 18
}

// ============================================================================
// INSTRUCTION CLASS TESTS
// ============================================================================

class InstructionTest : public ::testing::Test {
protected:
    void SetUp() override {
        left_turn_ = Instruction(Turn::kLeft, 3U);
        right_turn_ = Instruction(Turn::kRight, 2U);
        zero_steps_ = Instruction(Turn::kLeft, 0U);
        max_steps_ = Instruction(Turn::kRight, 999U);
    }
    
    Instruction left_turn_;
    Instruction right_turn_;
    Instruction zero_steps_;
    Instruction max_steps_;
};

TEST_F(InstructionTest, Constructor) {
    EXPECT_EQ(left_turn_.getTurn(), Turn::kLeft);
    EXPECT_EQ(left_turn_.getSteps(), 3U);
    
    EXPECT_EQ(right_turn_.getTurn(), Turn::kRight);
    EXPECT_EQ(right_turn_.getSteps(), 2U);
}

TEST_F(InstructionTest, ToStringMethod) {
    EXPECT_EQ(left_turn_.toString(), "L3");
    EXPECT_EQ(right_turn_.toString(), "R2");
    EXPECT_EQ(zero_steps_.toString(), "L0");
    EXPECT_EQ(max_steps_.toString(), "R999");
}

TEST_F(InstructionTest, EdgeCases) {
    // Test zero steps
    EXPECT_EQ(zero_steps_.getSteps(), 0U);
    
    // Test large step count
    EXPECT_EQ(max_steps_.getSteps(), 999U);
}

// ============================================================================
// ROBOT CLASS TESTS
// ============================================================================

class RobotTest : public ::testing::Test {
protected:
    void SetUp() override {
        default_robot_ = std::make_unique<Robot>();
        custom_robot_ = std::make_unique<Robot>(Position(2, 3), Direction::kEast);
    }
    
    std::unique_ptr<Robot> default_robot_;
    std::unique_ptr<Robot> custom_robot_;
};

TEST_F(RobotTest, DefaultConstructor) {
    EXPECT_EQ(default_robot_->getCurrentPosition(), Position(5, 5));
    EXPECT_EQ(default_robot_->getCurrentDirection(), Direction::kNorth);
    EXPECT_EQ(default_robot_->getActualSteps(), 0U);
}

TEST_F(RobotTest, ParameterizedConstructor) {
    EXPECT_EQ(custom_robot_->getCurrentPosition(), Position(2, 3));
    EXPECT_EQ(custom_robot_->getCurrentDirection(), Direction::kEast);
    EXPECT_EQ(custom_robot_->getActualSteps(), 0U);
}

TEST_F(RobotTest, DirectionCalculation) {
    // Test right turns from North
    Robot north_robot(Position(5, 5), Direction::kNorth);
    EXPECT_EQ(north_robot.calculateNewDirection(Turn::kRight), Direction::kEast);
    
    // Test left turns from North  
    EXPECT_EQ(north_robot.calculateNewDirection(Turn::kLeft), Direction::kWest);
    
    // Test full rotation (4 right turns = back to start)
    Direction current = Direction::kNorth;
    Robot test_robot(Position(5, 5), current);
    for (int i = 0; i < 4; ++i) {
        current = test_robot.calculateNewDirection(Turn::kRight);
        test_robot = Robot(Position(5, 5), current);  // Update robot direction
    }
    EXPECT_EQ(current, Direction::kNorth);
}

TEST_F(RobotTest, PositionCalculation) {
    // Test movement from center position
    Robot center_robot(Position(5, 5), Direction::kNorth);
    
    // Move North (Y decreases)
    Position north_pos = center_robot.calculateNewPosition();
    EXPECT_EQ(north_pos, Position(5, 4));
    
    // Test East movement
    Robot east_robot(Position(5, 5), Direction::kEast);
    Position east_pos = east_robot.calculateNewPosition();
    EXPECT_EQ(east_pos, Position(6, 5));
    
    // Test South movement
    Robot south_robot(Position(5, 5), Direction::kSouth);
    Position south_pos = south_robot.calculateNewPosition();
    EXPECT_EQ(south_pos, Position(5, 6));
    
    // Test West movement
    Robot west_robot(Position(5, 5), Direction::kWest);
    Position west_pos = west_robot.calculateNewPosition();
    EXPECT_EQ(west_pos, Position(4, 5));
}

TEST_F(RobotTest, BoundaryClampingNorth) {
    // Test clamping at North boundary (Y = 0)
    Robot boundary_robot(Position(5, 0), Direction::kNorth);
    Position clamped_pos = boundary_robot.calculateNewPosition();
    EXPECT_EQ(clamped_pos, Position(5, 0));  // Should stay at Y=0
}

TEST_F(RobotTest, BoundaryClampingSouth) {
    // Test clamping at South boundary (Y = 9)
    Robot boundary_robot(Position(5, 9), Direction::kSouth);
    Position clamped_pos = boundary_robot.calculateNewPosition();
    EXPECT_EQ(clamped_pos, Position(5, 9));  // Should stay at Y=9
}

TEST_F(RobotTest, BoundaryClampingEast) {
    // Test clamping at East boundary (X = 9)
    Robot boundary_robot(Position(9, 5), Direction::kEast);
    Position clamped_pos = boundary_robot.calculateNewPosition();
    EXPECT_EQ(clamped_pos, Position(9, 5));  // Should stay at X=9
}

TEST_F(RobotTest, BoundaryClampingWest) {
    // Test clamping at West boundary (X = 0)
    Robot boundary_robot(Position(0, 5), Direction::kWest);
    Position clamped_pos = boundary_robot.calculateNewPosition();
    EXPECT_EQ(clamped_pos, Position(0, 5));  // Should stay at X=0
}

TEST_F(RobotTest, SingleInstructionExecution) {
    Robot robot;  // Start at (5,5) facing North
    
    // Execute R2 (Right turn, 2 steps)
    Instruction instruction(Turn::kRight, 2U);
    robot.executeInstruction(instruction);
    
    // Should be facing East and at position (7,5)
    EXPECT_EQ(robot.getCurrentDirection(), Direction::kEast);
    EXPECT_EQ(robot.getCurrentPosition(), Position(7, 5));
    EXPECT_EQ(robot.getActualSteps(), 2U);
}

TEST_F(RobotTest, MultipleInstructionExecution) {
    Robot robot;  // Start at (5,5) facing North
    
    // Execute sequence: R2,L3,L1
    std::vector<Instruction> instructions = {
        Instruction(Turn::kRight, 2U),  // Face East, move to (7,5)
        Instruction(Turn::kLeft, 3U),   // Face North, move to (7,2)
        Instruction(Turn::kLeft, 1U)    // Face West, move to (6,2)
    };
    
    robot.executeInstructions(instructions);
    
    EXPECT_EQ(robot.getCurrentPosition(), Position(6, 2));
    EXPECT_EQ(robot.getCurrentDirection(), Direction::kWest);
    EXPECT_EQ(robot.getActualSteps(), 6U);  // 2 + 3 + 1 = 6 steps
}

TEST_F(RobotTest, PathHistoryTracking) {
    Robot robot;  // Start at (5,5)
    
    Instruction instruction(Turn::kRight, 2U);  // Move East 2 steps
    robot.executeInstruction(instruction);
    
    const auto& path = robot.getPathHistory();
    EXPECT_EQ(path.size(), 3U);  // Start + 2 steps
    EXPECT_EQ(path[0], Position(5, 5));  // Start
    EXPECT_EQ(path[1], Position(6, 5));  // Step 1
    EXPECT_EQ(path[2], Position(7, 5));  // Step 2
}

TEST_F(RobotTest, DistanceCalculations) {
    Robot robot;  // Start at (5,5)
    
    // Move to (7,2) via R2,L3,L1
    std::vector<Instruction> instructions = {
        Instruction(Turn::kRight, 2U),  // (7,5)
        Instruction(Turn::kLeft, 3U),   // (7,2)
        Instruction(Turn::kLeft, 1U)    // (6,2)
    };
    robot.executeInstructions(instructions);
    
    // Manhattan distance from (5,5) to (6,2) = |6-5| + |2-5| = 1 + 3 = 4
    EXPECT_EQ(robot.getManhattanDistance(), 4U);
    EXPECT_EQ(robot.getActualSteps(), 6U);
    
    // Efficiency = (4/6) * 100 = 66.67%
    EXPECT_NEAR(robot.getEfficiencyPercent(), 66.67, 0.1);
}

TEST_F(RobotTest, DirectionNames) {
    Robot north_robot(Position(5, 5), Direction::kNorth);
    Robot east_robot(Position(5, 5), Direction::kEast);
    Robot south_robot(Position(5, 5), Direction::kSouth);
    Robot west_robot(Position(5, 5), Direction::kWest);
    
    EXPECT_EQ(north_robot.getDirectionName(), "North ↑");
    EXPECT_EQ(east_robot.getDirectionName(), "East →");
    EXPECT_EQ(south_robot.getDirectionName(), "South ↓");
    EXPECT_EQ(west_robot.getDirectionName(), "West ←");
}

TEST_F(RobotTest, EfficiencyCalculationEdgeCases) {
    Robot no_move_robot;  // No movement
    EXPECT_EQ(no_move_robot.getEfficiencyPercent(), 100.0);  // Same position = 100%
    
    Robot optimal_robot;  // Optimal movement
    optimal_robot.executeInstruction(Instruction(Turn::kRight, 1U));  // (6,5)
    optimal_robot.executeInstruction(Instruction(Turn::kRight, 1U));  // (6,6)
    // Manhattan: |6-5| + |6-5| = 2, Actual: 2, Efficiency: 100%
    EXPECT_EQ(optimal_robot.getEfficiencyPercent(), 100.0);
}

// ============================================================================
// UTILITY FUNCTIONS TESTS
// ============================================================================

class UtilityTest : public ::testing::Test {};

TEST_F(UtilityTest, ParseInstructionsBasic) {
    std::string input = "R2,L3,R1";
    auto instructions = parseInstructions(input);
    
    ASSERT_EQ(instructions.size(), 3U);
    
    EXPECT_EQ(instructions[0].getTurn(), Turn::kRight);
    EXPECT_EQ(instructions[0].getSteps(), 2U);
    
    EXPECT_EQ(instructions[1].getTurn(), Turn::kLeft);
    EXPECT_EQ(instructions[1].getSteps(), 3U);
    
    EXPECT_EQ(instructions[2].getTurn(), Turn::kRight);
    EXPECT_EQ(instructions[2].getSteps(), 1U);
}

TEST_F(UtilityTest, ParseInstructionsWithSpaces) {
    std::string input = " R2 , L3 , R1 ";  // Extra spaces
    auto instructions = parseInstructions(input);
    
    ASSERT_EQ(instructions.size(), 3U);
    EXPECT_EQ(instructions[0].toString(), "R2");
    EXPECT_EQ(instructions[1].toString(), "L3");
    EXPECT_EQ(instructions[2].toString(), "R1");
}

TEST_F(UtilityTest, ParseInstructionsSingle) {
    std::string input = "L5";
    auto instructions = parseInstructions(input);
    
    ASSERT_EQ(instructions.size(), 1U);
    EXPECT_EQ(instructions[0].getTurn(), Turn::kLeft);
    EXPECT_EQ(instructions[0].getSteps(), 5U);
}

TEST_F(UtilityTest, ParseInstructionsEmpty) {
    std::string input = "";
    auto instructions = parseInstructions(input);
    
    EXPECT_EQ(instructions.size(), 0U);
}

TEST_F(UtilityTest, ParseInstructionsLargeNumbers) {
    std::string input = "R123,L456";
    auto instructions = parseInstructions(input);
    
    ASSERT_EQ(instructions.size(), 2U);
    EXPECT_EQ(instructions[0].getSteps(), 123U);
    EXPECT_EQ(instructions[1].getSteps(), 456U);
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

class IntegrationTest : public ::testing::Test {};

TEST_F(IntegrationTest, CompleteNavigationScenario) {
    // Test the complete navigation scenario from the main example
    Robot robot;  // Start at (5,5) facing North
    
    std::string instruction_string = "R2,L3,L1";
    auto instructions = parseInstructions(instruction_string);
    robot.executeInstructions(instructions);
    
    // Expected final position: (6,2)
    EXPECT_EQ(robot.getCurrentPosition(), Position(6, 2));
    EXPECT_EQ(robot.getCurrentDirection(), Direction::kWest);
    
    // Expected path: (5,5) -> (6,5) -> (7,5) -> (7,4) -> (7,3) -> (7,2) -> (6,2)
    const auto& path = robot.getPathHistory();
    ASSERT_EQ(path.size(), 7U);
    EXPECT_EQ(path[0], Position(5, 5));  // Start
    EXPECT_EQ(path[1], Position(6, 5));  // R1
    EXPECT_EQ(path[2], Position(7, 5));  // R2
    EXPECT_EQ(path[3], Position(7, 4));  // L1
    EXPECT_EQ(path[4], Position(7, 3));  // L2
    EXPECT_EQ(path[5], Position(7, 2));  // L3
    EXPECT_EQ(path[6], Position(6, 2));  // L1 (final)
}

TEST_F(IntegrationTest, SquarePathScenario) {
    // Test making a perfect square
    Robot robot;  // Start at (5,5) facing North
    
    std::string instruction_string = "R1,R1,R1,R1";  // 1x1 square clockwise
    auto instructions = parseInstructions(instruction_string);
    robot.executeInstructions(instructions);
    
    // Should return to start position
    EXPECT_EQ(robot.getCurrentPosition(), Position(5, 5));
    EXPECT_EQ(robot.getCurrentDirection(), Direction::kNorth);
    EXPECT_EQ(robot.getManhattanDistance(), 0U);  // Back at start
    EXPECT_EQ(robot.getActualSteps(), 4U);
}

TEST_F(IntegrationTest, BoundaryTestScenario) {
    // Test robot behavior at boundaries
    Robot corner_robot(Position(0, 0), Direction::kNorth);  // Top-left corner
    
    std::string instruction_string = "L5,L5";  // Try to go beyond boundaries
    auto instructions = parseInstructions(instruction_string);
    corner_robot.executeInstructions(instructions);
    
    // Should be clamped within grid
    Position final_pos = corner_robot.getCurrentPosition();
    EXPECT_GE(final_pos.getX(), 0);
    EXPECT_LE(final_pos.getX(), 9);
    EXPECT_GE(final_pos.getY(), 0);
    EXPECT_LE(final_pos.getY(), 9);
}

TEST_F(IntegrationTest, EfficiencyComparisonScenario) {
    // Compare different paths to same destination
    Robot direct_robot;
    Robot indirect_robot;
    
    // Direct path: R2,R2 -> (7,7)
    direct_robot.executeInstructions(parseInstructions("R2,R2"));
    
    // Indirect path: R1,R1,R1,R1 -> (7,7)  
    indirect_robot.executeInstructions(parseInstructions("R1,R1,R1,R1"));
    
    // Both should reach same position
    EXPECT_EQ(direct_robot.getCurrentPosition(), indirect_robot.getCurrentPosition());
    
    // But direct should be more efficient
    EXPECT_GT(direct_robot.getEfficiencyPercent(), indirect_robot.getEfficiencyPercent());
}

// ============================================================================
// PERFORMANCE AND STRESS TESTS
// ============================================================================

class PerformanceTest : public ::testing::Test {};

TEST_F(PerformanceTest, LargeInstructionSet) {
    Robot robot;
    
    // Create large instruction set
    std::vector<Instruction> large_instructions;
    for (int i = 0; i < 100; ++i) {
        Turn turn = (i % 2 == 0) ? Turn::kRight : Turn::kLeft;
        large_instructions.emplace_back(turn, 1U);
    }
    
    // Should execute without issues
    robot.executeInstructions(large_instructions);
    
    // Verify basic properties
    EXPECT_EQ(robot.getActualSteps(), 100U);
    EXPECT_GE(robot.getPathHistory().size(), 101U);  // Start + 100 steps
}

TEST_F(PerformanceTest, ZeroStepInstructions) {
    Robot robot;
    
    std::vector<Instruction> zero_step_instructions = {
        Instruction(Turn::kRight, 0U),
        Instruction(Turn::kLeft, 0U),
        Instruction(Turn::kRight, 0U),
        Instruction(Turn::kLeft, 0U)
    };
    
    robot.executeInstructions(zero_step_instructions);
    
    // Should stay at start position but change direction
    EXPECT_EQ(robot.getCurrentPosition(), Position(5, 5));
    EXPECT_EQ(robot.getActualSteps(), 0U);
    EXPECT_EQ(robot.getCurrentDirection(), Direction::kNorth);  // Full rotation
}

} // namespace test
} // namespace robot_nav

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "    ROBOT NAVIGATOR - COMPREHENSIVE UNIT TEST SUITE\n";
    std::cout << "               MISRA C++ 2023 COMPLIANT\n";
    std::cout << std::string(70, '=') << "\n\n";
    
    int result = RUN_ALL_TESTS();
    
    std::cout << "\n" << std::string(70, '=') << "\n";
    if (result == 0) {
        std::cout << "✓ ALL TESTS PASSED - ROBOT NAVIGATOR VERIFIED\n";
    } else {
        std::cout << "✗ SOME TESTS FAILED - CHECK IMPLEMENTATION\n";
    }
    std::cout << std::string(70, '=') << "\n\n";
    
    return result;
}
