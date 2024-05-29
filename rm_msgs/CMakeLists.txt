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
        ExchangerMsg.msg
        ChassisCmd.msg
        ShootCmd.msg
        ShootState.msg
        ShootBeforehandCmd.msg
        GimbalCmd.msg
        GimbalDesError.msg
        GimbalPosState.msg
        LpData.msg
        KalmanData.msg
        MovingAverageData.msg
        GpioData.msg
        TofRadarData.msg
        MultiDofCmd.msg
        TagMsg.msg
        TagMsgArray.msg
        SentryDeviate.msg
)

add_service_files(
        FILES
        ContinousDetectorSwitch.srv
        StatusChange.srv
        SetLimitVel.srv
        CameraStatus.srv
        EnableImuTrigger.srv
        EnableGyro.srv
)

add_message_files(
        DIRECTORY msg/detection
        FILES
        PriorityArray.msg
        TargetDetection.msg
        TargetDetectionArray.msg
        TrackData.msg
)

add_message_files(
        DIRECTORY msg/referee
        FILES
        CapacityData.msg
        GameRobotStatus.msg
        GameStatus.msg
        StateCmd.msg
        EngineerUi.msg
        GameRobotHp.msg
        PowerHeatData.msg
        EventData.msg
        DartStatus.msg
        IcraBuffDebuffZoneStatus.msg
        SupplyProjectileAction.msg
        DartRemainingTime.msg
        RobotHurt.msg
        CurrentSentryPosData.msg
        ShootData.msg
        BulletAllowance.msg
        RfidStatus.msg
        DartClientCmd.msg
        ManualToReferee.msg
        RadarData.msg
        RadarMarkData.msg
        RobotsPositionData.msg
        ClientMapSendData.msg
        ClientMapReceiveData.msg
        MapSentryData.msg
        GameRobotPosData.msg
        SentryInfo.msg
        RadarInfo.msg
        Buff.msg
        PowerManagementSampleAndStatusData.msg
        PowerManagementInitializationExceptionData.msg
        PowerManagementProcessStackOverflowData.msg
        PowerManagementSystemExceptionData.msg
        PowerManagementUnknownExceptionData.msg
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
