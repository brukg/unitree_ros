cmake_minimum_required(VERSION 3.5)
project(unitree_robots_description)

find_package(ament_cmake REQUIRED)

find_package(xacro REQUIRED)


install(
  DIRECTORY launch models rviz config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
