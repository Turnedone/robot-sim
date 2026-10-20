cmake_minimum_required(VERSION 3.25)
project(robot_sim
    VERSION 0.4.0
    DESCRIPTION "2D mobile-robot autonomy stack: kinematics, lidar, mapping, planning, control"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # for clang-tidy and editors

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(ROBOT_SIM_BUILD_VIEWER "Build the raylib viewer (needs a display library)" ON)
option(ROBOT_SIM_BUILD_TESTS "Build the unit and closed-loop tests" ON)
option(ROBOT_SIM_WERROR "Treat compiler warnings as errors" OFF)
option(ROBOT_SIM_SANITIZE "Build with AddressSanitizer + UndefinedBehaviorSanitizer (GCC/Clang)" OFF)

include(FetchContent)

# --- Shared compiler settings ------------------------------------------------

add_library(robot_sim_options INTERFACE)
if(MSVC)
    target_compile_options(robot_sim_options INTERFACE /W4 /permissive- $<$<BOOL:${ROBOT_SIM_WERROR}>:/WX>)
else()
    target_compile_options(robot_sim_options INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wcast-align -Wunused -Woverloaded-virtual
        $<$<BOOL:${ROBOT_SIM_WERROR}>:-Werror>)
    if(MINGW)
        # Put the C++ runtime inside the .exe. Otherwise Windows loads whichever
        # libstdc++-6.dll comes first on PATH, and an older one (Git for Windows
        # ships one) crashes programs built with a newer GCC.
        target_link_options(robot_sim_options INTERFACE -static)
    endif()
    if(ROBOT_SIM_SANITIZE)
        target_compile_options(robot_sim_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer
                                                           -fno-sanitize-recover=all)
        target_link_options(robot_sim_options INTERFACE -fsanitize=address,undefined)
    endif()
endif()

# --- The robot logic: no graphics, so it can be tested and reused on its own ---

add_library(robot_core
    src/grid.cpp
    src/robot.cpp
    src/lidar.cpp
    src/occupancy_map.cpp
    src/planner.cpp
    src/controller.cpp
    src/navigator.cpp
)
target_include_directories(robot_core PUBLIC src)
target_link_libraries(robot_core PUBLIC robot_sim_options)

# --- The viewer --------------------------------------------------------------

if(ROBOT_SIM_BUILD_VIEWER)
    # raylib draws the window. CMake downloads and builds it on the first
    # configure, so nothing has to be installed by hand.
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)  # X11 only on Linux: fewer system packages
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)            # raylib's bundled GLFW predates CMake 4
    FetchContent_Declare(raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
        URL_HASH SHA256=aea98ecf5bc5c5e0b789a76de0083a21a70457050ea4cc2aec7566935f5e258e
        SYSTEM)
    FetchContent_MakeAvailable(raylib)

    add_executable(robot_sim src/main.cpp src/viewer.cpp)
    target_link_libraries(robot_sim PRIVATE robot_core raylib)
endif()

# --- Tests -------------------------------------------------------------------

if(ROBOT_SIM_BUILD_TESTS)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
        SYSTEM)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # MSVC: same runtime as the project
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)

    add_executable(robot_tests
        tests/test_robot.cpp
        tests/test_lidar.cpp
        tests/test_mapping.cpp
        tests/test_planner.cpp
        tests/test_navigation.cpp
    )
    target_link_libraries(robot_tests PRIVATE robot_core GTest::gtest_main)
    target_compile_definitions(robot_tests PRIVATE ROBOT_SIM_MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
    gtest_discover_tests(robot_tests DISCOVERY_TIMEOUT 60)
endif()
