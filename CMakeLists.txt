cmake_minimum_required(VERSION 3.10)
project(rm_common)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint

        tf
        rm_msgs
        geometry_msgs
        realtime_tools
        )

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIR}
        CATKIN_DEPENDS
        rm_msgs
        roscpp
        DEPENDS
        LIBRARIES
        rm_common
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(rm_common SHARED ${sources})
add_executable(test_traj test/test_traj.cpp)
add_executable(test_kalman test/test_kalman_filter.cpp)
add_executable(test_lqr test/test_lqr.cpp)

target_link_libraries(rm_common ${catkin_LIBRARIES})
target_link_libraries(test_traj rm_common ${catkin_LIBRARIES})
target_link_libraries(test_kalman rm_common ${catkin_LIBRARIES})
target_link_libraries(test_lqr rm_common ${catkin_LIBRARIES})

roslint_cpp()
