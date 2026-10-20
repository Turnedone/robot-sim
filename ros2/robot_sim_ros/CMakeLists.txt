cmake_minimum_required(VERSION 3.16)
project(robot_sim_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# The robot logic from the repository's src/ folder, unchanged: it has no
# graphics and no ROS in it, so the same code runs in the viewer and here.
set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
add_library(robot_core STATIC
    ${CORE_DIR}/grid.cpp
    ${CORE_DIR}/robot.cpp
    ${CORE_DIR}/lidar.cpp
    ${CORE_DIR}/occupancy_map.cpp
    ${CORE_DIR}/planner.cpp
    ${CORE_DIR}/controller.cpp
    ${CORE_DIR}/navigator.cpp
    ${CORE_DIR}/particle_filter.cpp
)
target_include_directories(robot_core PUBLIC ${CORE_DIR})
set_target_properties(robot_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The nodes are classes in a library, so tests can run them in-process.
add_library(robot_sim_nodes
    src/sim_node.cpp
    src/mapper_node.cpp
    src/localizer_node.cpp
    src/navigator_node.cpp
)
target_include_directories(robot_sim_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(robot_sim_nodes PUBLIC
    robot_core
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros)

foreach(node sim_node mapper_node localizer_node navigator_node)
    add_executable(${node} src/main_${node}.cpp)
    target_link_libraries(${node} robot_sim_nodes)
    install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(TARGETS robot_sim_nodes robot_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../maps/ DESTINATION share/${PROJECT_NAME}/maps)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_conversions test/test_conversions.cpp)
    target_link_libraries(test_conversions robot_sim_nodes)

    # The ROS graph closing the loop: simulator + localizer + navigator,
    # talking over real topics and TF, must drive the robot to a goal.
    ament_add_gtest(test_nodes test/test_nodes.cpp TIMEOUT 180)
    target_link_libraries(test_nodes robot_sim_nodes)
    target_compile_definitions(test_nodes PRIVATE
        ROBOT_SIM_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../maps")
endif()

ament_package()
