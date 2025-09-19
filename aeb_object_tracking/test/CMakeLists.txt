# /// \file

include(FetchContent)
cmake_policy(SET CMP0135 NEW)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# This only collects test sources in the test/ directory.
file(GLOB TEST_SOURCES "*.cpp")

add_executable(aeb_object_tracking_tests ${TEST_SOURCES})

target_link_libraries(aeb_object_tracking_tests 
    PRIVATE 
        aeb_core
        gtest_main
        gtest
)

set_target_properties(aeb_object_tracking_tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_compile_options(aeb_object_tracking_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-g -O0>
    $<$<CXX_COMPILER_ID:MSVC>:/Od /Zi>
)

add_test(NAME CoreLibraryTests COMMAND aeb_object_tracking_tests)

include(GoogleTest)
gtest_discover_tests(aeb_object_tracking_tests)

