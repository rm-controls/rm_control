cmake_minimum_required(VERSION 3.10)
project(rm_referee)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-address-of-packed-member)

## Find catkin macros and libraries
find_package(serial REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        sensor_msgs
        roslint
        rm_msgs
        rm_common
        tf2_geometry_msgs
        serial
        std_msgs
        actionlib
        nav_msgs
        )

## Find system libraries
#find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        sensor_msgs
        roslint
        rm_msgs
        rm_common
        tf2_geometry_msgs
        std_msgs
        actionlib
        nav_msgs
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
FILE(GLOB ALL_SOURCES "src/*.cpp" "src/common/*.cpp" "src/referee/*.cpp")
add_executable(${PROJECT_NAME} ${ALL_SOURCES} include/rm_referee/common/referee_base.h include/rm_referee/common/data.h src/main.cpp src/standard_referee.cpp include/rm_referee/standard_referee.h include/rm_referee/engineer_referee.h src/engineer_referee.cpp src/hero_referee.cpp include/rm_referee/hero_referee.h src/common/referee_base.cpp include/rm_referee/robot_referee.h src/robot_referee.cpp include/rm_referee/radar_referee.h src/radar_referee.cpp)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation
install(
        DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
