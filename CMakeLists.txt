cmake_minimum_required(VERSION 2.8.3)
project(rm_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    )

add_message_files(
        FILES
        Joint.msg
        DbusData.msg
        ChassisCmd.msg
        ShootCmd.msg
        GimbalCmd.msg
        GimbalTrackCmd.msg
)

add_service_files(
    FILES
    ContinousDetectorSwitch.srv
)

add_message_files(
    DIRECTORY msg/detection
    FILES
    TargetDetection.msg
    TargetDetectionArray.msg
    TrackData.msg
)

add_action_files(
    FILES
    ShootCmd.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
