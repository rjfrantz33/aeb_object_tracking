/******************************************************************************
robot_navigator.h - Header File
MISRA C++ 2023 Compliant Robot Navigator Header

MISRA C++ 2023 Features Applied:
✓ Google naming style: kConstants, member_variables_, functionNames()
✓ Strong typing: enum class, explicit integer types
✓ Memory safety: No raw pointers, RAII, bounds checking
✓ Exception safety: noexcept, no try-catch blocks
✓ Header guards: Proper include protection
*******************************************************************************/

#ifndef ROBOT_NAVIGATOR_H
#define ROBOT_NAVIGATOR_H

#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <cstdint>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace robot_nav {

// ============================================================================
// CONSTANTS - Google Style with k prefix
// ============================================================================

constexpr std::int32_t kGridSize = 10;
constexpr std::int32_t kStartX = 5;
constexpr std::int32_t kStartY = 5;

// ============================================================================
// STRONG TYPED ENUMS - MISRA 5-0-3 compliance
// ============================================================================

enum class Direction : std::uint8_t {
    kNorth = 0U,  // ↑
    kEast = 1U,   // →  
    kSouth = 2U,  // ↓
    kWest = 3U    // ←
};

enum class Turn : std::uint8_t {
    kLeft,
    kRight
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

class Position;
class Instruction;
class Robot;

// ============================================================================
// POSITION CLASS DECLARATION
// ============================================================================

class Position {
private:
    std::int32_t x_;
    std::int32_t y_;

public:
    // Explicit constructor prevents implicit conversions (MISRA)
    explicit Position(std::int32_t x = kStartX, std::int32_t y = kStartY) noexcept;
    
    // Const accessors (MISRA 7-1-1)
    std::int32_t getX() const noexcept;
    std::int32_t getY() const noexcept;
    
    // Comparison operators
    bool operator==(const Position& other) const noexcept;
    bool operator!=(const Position& other) const noexcept;
    
    // Helper method for display
    std::string toString() const;
    
    // Calculate Manhattan distance between two positions
    static std::uint32_t getManhattanDistance(const Position& from, const Position& to) noexcept;
};

// ============================================================================
// INSTRUCTION CLASS DECLARATION
// ============================================================================

class Instruction {
private:
    Turn turn_;
    std::uint32_t steps_;

public:
    explicit Instruction(Turn turn, std::uint32_t steps) noexcept;
    
    Turn getTurn() const noexcept;
    std::uint32_t getSteps() const noexcept;
    std::string toString() const;
};

// ============================================================================
// ROBOT CLASS DECLARATION
// ============================================================================

class Robot {
private:
    Position position_;
    Direction direction_;
    std::vector<Position> path_history_;
    
    // Direction vectors for movement (MISRA: constexpr arrays)
    static constexpr std::array<std::int32_t, 4> kDeltaX = {0, 1, 0, -1};  // N,E,S,W
    static constexpr std::array<std::int32_t, 4> kDeltaY = {-1, 0, 1, 0};  // N,E,S,W
    
    // Helper methods
    Direction calculateNewDirection(Turn turn) const noexcept;
    Position calculateNewPosition() const noexcept;

public:
    // Constructor with default parameters
    explicit Robot(Position start_pos = Position(), Direction start_dir = Direction::kNorth) noexcept;
    
    // Execution methods
    void executeInstruction(const Instruction& instruction) noexcept;
    void executeInstructions(const std::vector<Instruction>& instructions) noexcept;
    
    // Getters
    Position getCurrentPosition() const noexcept;
    Direction getCurrentDirection() const noexcept;
    const std::vector<Position>& getPathHistory() const noexcept;
    Position getStartPosition() const noexcept;
    
    // Metrics - Fixed naming and calculation
    std::uint32_t getActualSteps() const noexcept;
    std::uint32_t getManhattanDistance() const noexcept;  // Renamed for clarity
    double getEfficiencyPercent() const noexcept;
    std::string getDirectionName() const;
};

// ============================================================================
// UTILITY FUNCTIONS DECLARATIONS
// ============================================================================

std::vector<Instruction> parseInstructions(const std::string& input);
void printAnalysis(const Robot& robot, const std::vector<Instruction>& instructions);

// ============================================================================
// TEST STRUCTURE
// ============================================================================

struct TestCase {
    std::string name;
    std::string instructions;
    Position expected_position;
    std::string description;
};

void runTestSuite();

} // namespace robot_nav

#endif // ROBOT_NAVIGATOR_H
