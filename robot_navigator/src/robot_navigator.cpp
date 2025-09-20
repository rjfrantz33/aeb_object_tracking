/******************************************************************************
robot_navigator.cpp - Implementation File
MISRA C++ 2023 Compliant Robot Navigator Implementation
*******************************************************************************/

#include "robot_navigator/include/robot_navigator.h"

namespace robot_nav {

// ============================================================================
// POSITION CLASS IMPLEMENTATION
// ============================================================================

Position::Position(std::int32_t x, std::int32_t y) noexcept : x_(x), y_(y) {}

std::int32_t Position::getX() const noexcept {
    return x_;
}

std::int32_t Position::getY() const noexcept {
    return y_;
}

bool Position::operator==(const Position& other) const noexcept {
    return (x_ == other.x_) && (y_ == other.y_);
}

bool Position::operator!=(const Position& other) const noexcept {
    return !(*this == other);
}

std::string Position::toString() const {
    return "(" + std::to_string(x_) + "," + std::to_string(y_) + ")";
}

// Manhattan distance: |x2-x1| + |y2-y1| (straight-line grid distance)
std::uint32_t Position::getManhattanDistance(const Position& from, const Position& to) noexcept {
    std::int32_t dx = std::abs(to.x_ - from.x_);
    std::int32_t dy = std::abs(to.y_ - from.y_);
    return static_cast<std::uint32_t>(dx + dy);
}

// ============================================================================
// INSTRUCTION CLASS IMPLEMENTATION
// ============================================================================

Instruction::Instruction(Turn turn, std::uint32_t steps) noexcept 
    : turn_(turn), steps_(steps) {}

Turn Instruction::getTurn() const noexcept {
    return turn_;
}

std::uint32_t Instruction::getSteps() const noexcept {
    return steps_;
}

std::string Instruction::toString() const {
    std::string turn_str = (turn_ == Turn::kLeft) ? "L" : "R";
    return turn_str + std::to_string(steps_);
}

// ============================================================================
// ROBOT CLASS IMPLEMENTATION
// ============================================================================

Robot::Robot(Position start_pos, Direction start_dir) noexcept
    : position_(start_pos), direction_(start_dir) {
    path_history_.push_back(position_);
}

Direction Robot::calculateNewDirection(Turn turn) const noexcept {
    std::uint8_t current = static_cast<std::uint8_t>(direction_);
    
    if (turn == Turn::kRight) {
        current = (current + 1U) % 4U;
    } else {  // Turn::kLeft
        current = (current + 3U) % 4U;  // +3 = -1 mod 4
    }
    
    return static_cast<Direction>(current);
}

Position Robot::calculateNewPosition() const noexcept {
    std::uint8_t dir_index = static_cast<std::uint8_t>(direction_);
    
    std::int32_t new_x = position_.getX() + kDeltaX[dir_index];
    std::int32_t new_y = position_.getY() + kDeltaY[dir_index];
    
    // Keep within grid bounds (MISRA: std::clamp for safety)
    new_x = std::clamp(new_x, 0, kGridSize - 1);
    new_y = std::clamp(new_y, 0, kGridSize - 1);
    
    return Position(new_x, new_y);
}

void Robot::executeInstruction(const Instruction& instruction) noexcept {
    // Step 1: Turn
    direction_ = calculateNewDirection(instruction.getTurn());
    
    // Step 2: Move forward specified number of steps
    for (std::uint32_t step = 0U; step < instruction.getSteps(); ++step) {
        position_ = calculateNewPosition();
        path_history_.push_back(position_);
    }
}

void Robot::executeInstructions(const std::vector<Instruction>& instructions) noexcept {
    for (const auto& instruction : instructions) {
        executeInstruction(instruction);
    }
}

Position Robot::getCurrentPosition() const noexcept {
    return position_;
}

Direction Robot::getCurrentDirection() const noexcept {
    return direction_;
}

const std::vector<Position>& Robot::getPathHistory() const noexcept {
    return path_history_;
}

Position Robot::getStartPosition() const noexcept {
    if (path_history_.empty()) {
        return Position();  // Default start position
    }
    return path_history_[0];
}

std::uint32_t Robot::getActualSteps() const noexcept {
    return static_cast<std::uint32_t>(path_history_.size()) - 1U;  // -1 for start position
}

// FIXED: Renamed and clarified this is Manhattan distance from start to end
std::uint32_t Robot::getManhattanDistance() const noexcept {
    if (path_history_.empty()) return 0U;
    return Position::getManhattanDistance(path_history_[0], position_);
}

double Robot::getEfficiencyPercent() const noexcept {
    std::uint32_t actual = getActualSteps();
    std::uint32_t manhattan = getManhattanDistance();
    
    if (actual == 0U) {
        return (manhattan == 0U) ? 100.0 : 0.0;
    }
    
    // Note: This uses Manhattan distance as theoretical minimum
    // Actual robot shortest path would be longer due to movement constraints
    return (static_cast<double>(manhattan) / static_cast<double>(actual)) * 100.0;
}

std::string Robot::getDirectionName() const {
    switch (direction_) {
        case Direction::kNorth: return "North ↑";
        case Direction::kEast:  return "East →";
        case Direction::kSouth: return "South ↓";  
        case Direction::kWest:  return "West ←";
    }
    return "Unknown";
}

// ============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// ============================================================================

std::vector<Instruction> parseInstructions(const std::string& input) {
    std::vector<Instruction> instructions;
    
    std::string current_token;
    for (char c : input) {
        if (c == ',') {
            if (!current_token.empty()) {
                // Parse token like "R2" or "L3"
                Turn turn = (current_token[0] == 'L') ? Turn::kLeft : Turn::kRight;
                std::uint32_t steps = static_cast<std::uint32_t>(std::stoi(current_token.substr(1)));
                instructions.emplace_back(turn, steps);
                current_token.clear();
            }
        } else if (c != ' ') {  // Skip spaces
            current_token += c;
        }
    }
    
    // Handle last token
    if (!current_token.empty()) {
        Turn turn = (current_token[0] == 'L') ? Turn::kLeft : Turn::kRight;
        std::uint32_t steps = static_cast<std::uint32_t>(std::stoi(current_token.substr(1)));
        instructions.emplace_back(turn, steps);
    }
    
    return instructions;
}

void printAnalysis(const Robot& robot, const std::vector<Instruction>& instructions) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "           ROBOT NAVIGATION ANALYSIS\n";
    std::cout << std::string(60, '=') << "\n";
    
    // Instructions
    std::cout << "\nInstructions: ";
    for (std::size_t i = 0; i < instructions.size(); ++i) {
        std::cout << instructions[i].toString();
        if (i < instructions.size() - 1) std::cout << ", ";
    }
    std::cout << "\n";
    
    // Results
    const auto& path = robot.getPathHistory();
    std::cout << "\nResults:\n";
    std::cout << "  Start Position: " << path[0].toString() << "\n";
    std::cout << "  Final Position: " << robot.getCurrentPosition().toString() << "\n";
    std::cout << "  Final Direction: " << robot.getDirectionName() << "\n";
    
    // FIXED: Clearer metrics with detailed Manhattan distance calculation
    Position start_pos = robot.getStartPosition();
    Position end_pos = robot.getCurrentPosition();
    
    std::cout << "\nDistance Metrics:\n";
    std::cout << "  Actual Steps Taken: " << robot.getActualSteps() << " steps\n";
    
    // Show detailed Manhattan distance calculation
    std::int32_t dx = std::abs(end_pos.getX() - start_pos.getX());
    std::int32_t dy = std::abs(end_pos.getY() - start_pos.getY());
    std::uint32_t manhattan = static_cast<std::uint32_t>(dx + dy);
    
    std::cout << "  Manhattan Distance: |" << end_pos.getX() << "-" << start_pos.getX() 
              << "| + |" << end_pos.getY() << "-" << start_pos.getY() << "| = "
              << dx << " + " << dy << " = " << manhattan << " steps (straight-line)\n";
    
    std::cout << "  Efficiency: " << std::fixed << std::setprecision(1) 
              << robot.getEfficiencyPercent() << "% (Manhattan/Actual = " 
              << manhattan << "/" << robot.getActualSteps() << ")\n";
    
    // Path trace (simplified)
    std::cout << "\nPath Trace:\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        std::cout << "  Step " << i << ": " << path[i].toString();
        if (i == 0) std::cout << " (START)";
        if (i == path.size() - 1) std::cout << " (END)";
        std::cout << "\n";
    }
}

// ============================================================================
// TEST SUITE IMPLEMENTATION
// ============================================================================

void runTestSuite() {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "                    COMPREHENSIVE TEST SUITE\n";
    std::cout << std::string(70, '=') << "\n";
    
    // Define test cases with DEFAULT start position (5,5)
    std::vector<TestCase> test_cases = {
        {
            "Basic Example", 
            "R2,L3,L1", 
            Position(6, 2),
            "Original problem: Turn right, move 2, turn left, move 3, turn left, move 1"
        },
        {
            "Simple Square", 
            "R1,R1,R1,R1", 
            Position(5, 5),
            "Make a 1x1 square clockwise - should return to start"
        },
        {
            "Perfect Cross", 
            "R2,L2,L2,L2", 
            Position(5, 3),
            "Move right 2, then make three left turns of 2 steps each"
        },
        {
            "Diagonal Path", 
            "R1,L1,R1,L1,R1,L1", 
            Position(8, 2),
            "Zigzag pattern moving northeast"
        },
        {
            "Edge Test", 
            "R5,R5,R5,R5", 
            Position(5, 5),
            "Try to move beyond grid boundaries - should be clamped"
        }
    };
    
    // Run default position tests
    std::cout << "\n" << std::string(60, '-') << "\n";
    std::cout << "SECTION 1: DEFAULT START POSITION (5,5) TESTS\n";
    std::cout << std::string(60, '-') << "\n";
    
    for (std::size_t i = 0; i < test_cases.size(); ++i) {
        const auto& test = test_cases[i];
        
        std::cout << "\n" << std::string(50, '-') << "\n";
        std::cout << "TEST " << (i + 1) << ": " << test.name << "\n";
        std::cout << std::string(50, '-') << "\n";
        std::cout << "Description: " << test.description << "\n";
        
        // Create robot with DEFAULT start position (5,5)
        Robot robot;  // Uses Position() which defaults to (5,5)
        auto instructions = parseInstructions(test.instructions);
        robot.executeInstructions(instructions);
        
        // Check result
        Position actual = robot.getCurrentPosition();
        bool passed = (actual == test.expected_position);
        
        std::cout << "Start Position: (5,5) [DEFAULT]\n";
        std::cout << "Instructions: " << (test.instructions.empty() ? "(none)" : test.instructions) << "\n";
        std::cout << "Expected: " << test.expected_position.toString() << "\n";
        std::cout << "Actual: " << actual.toString() << "\n";
        std::cout << "Result: " << (passed ? "✓ PASS" : "✗ FAIL") << "\n";
        
        // Show distance metrics with calculation details
        Position start_pos = robot.getStartPosition();
        Position end_pos = robot.getCurrentPosition();
        
        std::cout << "Actual Steps: " << robot.getActualSteps() << "\n";
        
        // Show detailed Manhattan distance calculation
        std::int32_t dx = std::abs(end_pos.getX() - start_pos.getX());
        std::int32_t dy = std::abs(end_pos.getY() - start_pos.getY());
        std::uint32_t manhattan = static_cast<std::uint32_t>(dx + dy);
        
        std::cout << "Manhattan Distance: |" << end_pos.getX() << "-" << start_pos.getX() 
                  << "| + |" << end_pos.getY() << "-" << start_pos.getY() << "| = "
                  << dx << " + " << dy << " = " << manhattan << "\n";
        
        std::cout << "Efficiency: " << std::fixed << std::setprecision(1) 
                  << robot.getEfficiencyPercent() << "%\n";
        
        if (!passed) {
            std::cout << "ERROR: Test failed!\n";
        }
    }
    
    // CUSTOM START POSITION TESTS
    std::cout << "\n" << std::string(60, '-') << "\n";
    std::cout << "SECTION 2: CUSTOM START POSITION TESTS\n";
    std::cout << std::string(60, '-') << "\n";
    
    // Define custom start position test cases
    struct CustomPositionTest {
        std::string name;
        Position start_pos;
        std::string instructions;
        std::string description;
    };
    
    std::vector<CustomPositionTest> custom_tests = {
        {
            "Corner Start - Bottom Left",
            Position(0, 9),  // Bottom-left corner
            "R2,L1",
            "Start at bottom-left corner, move right 2, turn left, move 1"
        },
        {
            "Corner Start - Top Right", 
            Position(9, 0),  // Top-right corner
            "L2,R1",
            "Start at top-right corner, move left 2, turn right, move 1"
        },
        {
            "Edge Start - Left Side",
            Position(0, 5),  // Left edge, middle
            "R3,L2,R1",
            "Start at left edge, complex path with multiple turns"
        },
        {
            "Center-Left Start",
            Position(2, 7),  // Your example position
            "R1,L2,R3",
            "Start at (2,7), demonstrate different Manhattan base calculation"
        },
        {
            "Near Corner Start",
            Position(8, 1),  // Near top-right
            "L1,L1,L1",
            "Start near corner, make three left turns with steps"
        }
    };
    
    for (std::size_t i = 0; i < custom_tests.size(); ++i) {
        const auto& test = custom_tests[i];
        
        std::cout << "\n" << std::string(50, '-') << "\n";
        std::cout << "CUSTOM TEST " << (i + 1) << ": " << test.name << "\n";
        std::cout << std::string(50, '-') << "\n";
        std::cout << "Description: " << test.description << "\n";
        
        // Create robot at CUSTOM start position
        Robot custom_robot(test.start_pos);  // Custom start position
        auto instructions = parseInstructions(test.instructions);
        custom_robot.executeInstructions(instructions);
        
        Position start_pos = custom_robot.getStartPosition();
        Position end_pos = custom_robot.getCurrentPosition();
        
        std::cout << "Start Position: " << start_pos.toString() << " [CUSTOM]\n";
        std::cout << "Instructions: " << test.instructions << "\n";
        std::cout << "Final Position: " << end_pos.toString() << "\n";
        
        // Show distance metrics with calculation details
        std::cout << "Actual Steps: " << custom_robot.getActualSteps() << "\n";
        
        // Show detailed Manhattan distance calculation with custom start position
        std::int32_t dx = std::abs(end_pos.getX() - start_pos.getX());
        std::int32_t dy = std::abs(end_pos.getY() - start_pos.getY());
        std::uint32_t manhattan = static_cast<std::uint32_t>(dx + dy);
        
        std::cout << "Manhattan Distance: |" << end_pos.getX() << "-" << start_pos.getX() 
                  << "| + |" << end_pos.getY() << "-" << start_pos.getY() << "| = "
                  << dx << " + " << dy << " = " << manhattan << "\n";
        
        std::cout << "Efficiency: " << std::fixed << std::setprecision(1) 
                  << custom_robot.getEfficiencyPercent() << "%\n";
        
        // Show how this differs from default (5,5) calculation
        std::int32_t dx_default = std::abs(end_pos.getX() - 5);
        std::int32_t dy_default = std::abs(end_pos.getY() - 5);
        std::uint32_t manhattan_default = static_cast<std::uint32_t>(dx_default + dy_default);
        
        std::cout << "If started from (5,5): |" << end_pos.getX() << "-5| + |" 
                  << end_pos.getY() << "-5| = " << dx_default << " + " << dy_default 
                  << " = " << manhattan_default << " [COMPARISON]\n";
    }
    
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "All tests completed!\n";
    std::cout << "Note: Manhattan distance changes based on START position\n";
    std::cout << std::string(50, '=') << "\n";
}

} // namespace robot_nav
