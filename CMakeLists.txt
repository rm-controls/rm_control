cmake_minimum_required(VERSION 2.8.3)
project(rm_common)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rm_msgs
    roscpp
    geometry_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    rm_msgs
    roscpp
    DEPENDS
    LIBRARIES
    rm_common
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(rm_common SHARED ${sources})
add_executable(test_traj test/test_traj.cpp)
add_executable(test_kalman test/test_kalman_filter.cpp)
add_executable(test_bullet_solver test/test_bullet_solver.cpp)

target_link_libraries(rm_common ${catkin_LIBRARIES})
target_link_libraries(test_traj rm_common ${catkin_LIBRARIES})
target_link_libraries(test_kalman rm_common ${catkin_LIBRARIES})
target_link_libraries(test_bullet_solver rm_common ${catkin_LIBRARIES})
