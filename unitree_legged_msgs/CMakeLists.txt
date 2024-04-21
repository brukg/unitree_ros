cmake_minimum_required(VERSION 3.5)
project(unitree_legged_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/Cartesian.msg"
  "msg/Imu.msg"
  "msg/Led.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/HighCmd.msg"
  "msg/HighState.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


ament_package()
