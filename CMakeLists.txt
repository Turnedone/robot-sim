cmake_minimum_required(VERSION 3.20)
project(robot_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(robot_sim
    src/main.cpp
    src/grid.cpp
)

target_compile_options(robot_sim PRIVATE -Wall -Wextra -Wpedantic)
