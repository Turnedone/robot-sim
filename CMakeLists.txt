cmake_minimum_required(VERSION 3.25)
project(robot_sim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# raylib draws the window. CMake downloads and builds it on the first configure,
# so nothing has to be installed by hand.
include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)  # raylib's bundled GLFW predates CMake 4
FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
    SYSTEM
)
FetchContent_MakeAvailable(raylib)

# The robot logic: no graphics in here, so it can be tested on its own
# and later reused unchanged (e.g. inside a ROS2 node).
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
target_compile_options(robot_core PRIVATE -Wall -Wextra -Wpedantic)

# The program: loads a map and shows it, either in a window or as text.
add_executable(robot_sim
    src/main.cpp
    src/viewer.cpp
)
target_link_libraries(robot_sim PRIVATE robot_core raylib)
target_compile_options(robot_sim PRIVATE -Wall -Wextra -Wpedantic)

# Tests for the robot logic. Run with: ctest --test-dir build --output-on-failure
enable_testing()
add_executable(test_robot tests/test_robot.cpp)
target_link_libraries(test_robot PRIVATE robot_core)
target_compile_options(test_robot PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME robot_motion COMMAND test_robot)

add_executable(test_lidar tests/test_lidar.cpp)
target_link_libraries(test_lidar PRIVATE robot_core)
target_compile_options(test_lidar PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME lidar COMMAND test_lidar)

add_executable(test_mapping tests/test_mapping.cpp)
target_link_libraries(test_mapping PRIVATE robot_core)
target_compile_options(test_mapping PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mapping COMMAND test_mapping)

add_executable(test_navigation tests/test_navigation.cpp)
target_link_libraries(test_navigation PRIVATE robot_core)
target_compile_options(test_navigation PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(test_navigation PRIVATE ROBOT_SIM_MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
add_test(NAME navigation COMMAND test_navigation)
