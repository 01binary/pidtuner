cmake_minimum_required(VERSION 3.0.2)
project(pituner)

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
  StepCommand.msg
  Steps.msg
  Configuration.msg
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${pituner_INCLUDE_DIRS}
)

add_executable(
  pidtuner
  src/pidtuner.cpp
)

add_dependencies(
  pidtuner
  ${catkin_EXPORTED_TARGETS}
  ${pituner_EXPORTED_TARGETS}
)

target_link_libraries(
  pidtuner
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    pidtuner
  RUNTIME
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
