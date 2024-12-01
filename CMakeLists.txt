cmake_minimum_required(VERSION 3.0.2)
project(pidtuner)

add_compile_options(-std=c++11)

find_package(
  catkin
  REQUIRED COMPONENTS
    control_msgs
    moveit_msgs
    message_generation
    control_toolbox
    roscpp
    urdf
)

add_message_files(
  DIRECTORY msg
  FILES
  Configuration.msg
  VelocityCommand.msg
  VelocityFeedback.msg
  PositionCommand.msg
  PositionFeedback.msg
  EmergencyStop.msg
  StepCommand.msg
  Step.msg
  Configuration.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ConfigurationServer.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    actionlib
    control_msgs
    moveit_msgs
    control_toolbox
    message_runtime
)
