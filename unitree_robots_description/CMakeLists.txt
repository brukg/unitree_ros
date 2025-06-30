cmake_minimum_required(VERSION 3.5)
project(unitree_robots_description)

find_package(ament_cmake REQUIRED)

find_package(xacro REQUIRED)


install(
  DIRECTORY launch models rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/test_g1_control.py
    scripts/test_g1_simple_hand_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
