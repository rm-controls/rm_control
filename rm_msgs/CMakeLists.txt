cmake_minimum_required(VERSION 3.10)
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
        ActuatorState.msg
        BalanceState.msg
        DbusData.msg
        ChassisCmd.msg
        ShootCmd.msg
        GimbalCmd.msg
        GimbalDesError.msg
        Referee.msg
        LpData.msg
        KalmanData.msg
        MovingAverageData.msg
        SuperCapacitor.msg
        GpioData.msg
        TofRadarData.msg
)

add_service_files(
        FILES
        ContinousDetectorSwitch.srv
        StatusChange.srv
        CameraStatus.srv
        EnableImuTrigger.srv
)

add_message_files(
        DIRECTORY msg/detection
        FILES
        TargetDetection.msg
        TargetDetectionArray.msg
        TrackData.msg
)

add_message_files(
        DIRECTORY msg/referee
        FILES
        GameRobotStatus.msg
        GameStatus.msg
        EventData.msg
)

add_action_files(
        FILES
        Engineer.action
)
# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
)
