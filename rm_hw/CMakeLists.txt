cmake_minimum_required(VERSION 3.10)
project(rm_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimize
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)
## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rm_common
        rm_msgs
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        urdf
        realtime_tools
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
        ${PROJECT_NAME}_loader_plugins
        CATKIN_DEPENDS
        roscpp
        roslint

        rm_common
        rm_msgs
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        urdf
        realtime_tools
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_loader_plugins
        src/transmission/double_actuator_transmission_loader.cpp
        )
## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        src/hardware_interface/hardware_interface.cpp
        src/hardware_interface/parse.cpp
        src/hardware_interface/socketcan.cpp
        src/hardware_interface/can_bus.cpp
        src/control_loop.cpp
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_loader_plugins
        ${catkin_LIBRARIES}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_loader_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(
        FILES rm_hw_loader_plugins.xml
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
