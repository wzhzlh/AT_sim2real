cmake_minimum_required(VERSION 3.5)
project(robot_msgs VERSION 3.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorCommand.msg"
    "msg/MotorState.msg"
    "msg/RobotCommand.msg"
    "msg/RobotState.msg"
    "msg/IMU.msg"
    "msg/Cmd.msg"
    "msg/Remote.msg"
    "msg/Int.msg"
    "msg/BoxIdGrid.msg"
    "msg/Arm.msg"
    "msg/Arm4.msg"
    "msg/Armmode.msg"
    "msg/GM6020.msg"
    "msg/Joint.msg"
    "msg/RobStride.msg"
    "msg/Servo.msg"
    "msg/Vis.msg" 
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
  )
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
