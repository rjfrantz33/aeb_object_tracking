
# /// \file

cmake_minimum_required(VERSION 3.16)
project(AEBObjectTracker VERSION 1.0.0 DESCRIPTION "Autonomous Emergency Braking Object Tracking System" LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

find_program(CLANG_FORMAT clang-format)

# Gather source and header files
file(GLOB_RECURSE SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
file(GLOB_RECURSE HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
)

# Separate main.cpp from library sources
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

# Create library for core functionality (excluding main.cpp)
add_library(aeb_core ${SOURCES} ${HEADERS})

target_include_directories(aeb_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executable that uses the library
add_executable(aeb_tracker src/main.cpp)

target_link_libraries(aeb_tracker
    PRIVATE
        aeb_core
)

# Compiler options for the library (will be inherited by executable)
if(MSVC)
    # MSVC warnings
    target_compile_options(aeb_core PUBLIC /W4)
    # Additional MSVC warnings for Debug builds
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(aeb_core PUBLIC /WX)  # Treat warnings as errors
    endif()
else()
    # Base warnings
    target_compile_options(aeb_core PUBLIC
        -Wall -Wextra -Wpedantic 
        -Wdisabled-optimization -Winit-self -Wnon-virtual-dtor 
        -Wold-style-cast -Woverloaded-virtual -Wparentheses 
        -Wredundant-decls -Wunused -Wmisleading-indentation 
        -Wnull-dereference -Wformat=2 -Wimplicit-fallthrough 
        -Wshadow -Wsign-promo -Wstrict-aliasing
        -Wconversion -Wsign-conversion -Wcast-align 
        -Wlogical-op -Wduplicated-cond -Wduplicated-branches
    )
    # GCC-specific additional warnings
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(aeb_core PUBLIC
            -Wuseless-cast -Wdouble-promotion
            -Wnoexcept -Wctor-dtor-privacy
            -Wstrict-null-sentinel -Wzero-as-null-pointer-constant
        )
    endif()
    # Extra warnings for Debug builds
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(aeb_core PUBLIC -Werror -fno-omit-frame-pointer)
    endif()
endif()

# Custom target to run the executable
add_custom_target(run
    COMMAND aeb_tracker
    DEPENDS aeb_tracker
    COMMENT "Running AEB Object Tracker"
)

# clang-format target
if(CLANG_FORMAT)
    file(GLOB_RECURSE ALL_SOURCE_FILES
        ${CMAKE_SOURCE_DIR}/include/*.h
        ${CMAKE_SOURCE_DIR}/include/*.hpp
        ${CMAKE_SOURCE_DIR}/src/*.cpp
        ${CMAKE_SOURCE_DIR}/test/*.cpp
    )
    add_custom_target(format
        COMMAND ${CLANG_FORMAT} -i ${ALL_SOURCE_FILES}
        COMMENT "Running clang-format on all source files"
    )
endif()

# Add test folder
enable_testing()
add_subdirectory(test)

# Print build information
message(STATUS "")
message(STATUS "AEB Object Tracker Build Configuration:")
message(STATUS "  Project Name: ${PROJECT_NAME}")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Source Directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "  Binary Directory: ${CMAKE_BINARY_DIR}")
message(STATUS "")
message(STATUS "Build Instructions:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake ..")
message(STATUS "  make")
message(STATUS "  ./aeb_tracker")
message(STATUS "")
message(STATUS "Available targets:")
message(STATUS "  make         - Build the project")
message(STATUS "  make run     - Build and run the application")
message(STATUS "  make format  - Format all source files")
message(STATUS "  make install - Install the application")
message(STATUS "")

