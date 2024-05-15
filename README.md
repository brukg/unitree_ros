
# Introduction

This repository contains ROS 2 simulation packages for Unitree robots, compatible with ROS 2 Humble and Gazebo Harmonic. You can load robots and joint controllers in Gazebo, enabling low-level control of the robot joints (torque, position, and angular velocity). Note that the Gazebo simulation does not support high-level control like walking. These simulation capabilities complement the ability to control real robots using our ROS packages, found in [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).

## Packages

- Robot description packages: `go1_description`, `a1_description`, `aliengo_description`, `laikago_description`, `z1_description`
- Robot and joints controller packages: `unitree_controller`, `z1_controller`
- Simulation-related packages: `unitree_gazebo`, `unitree_legged_control`

# Dependencies

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [Gazebo Harmonic](https://gazebosim.org/home)

# Build Instructions

Ensure ROS 2 Humble and the required dependencies are installed:

```bash
sudo apt-get install ros-humble-controller-manager ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-broadcaster ros-humble-effort-controllers ros-humble-joint-trajectory-controller
```

Use colcon to build your packages:

```bash
cd ~/your_ros2_ws
colcon build --symlink-install
```

If you encounter dependency issues, try building again or ensure all dependencies are correctly installed.

# Package Details

## unitree_legged_control

This package contains joint controllers for Gazebo simulation, which enables users to control joints with position, velocity, and torque. For examples of joint control in different modes, refer to "[unitree_ros/unitree_controller/src/servo.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/servo.cpp)".

## Robot Descriptions

Descriptions for Go1, A1, Aliengo, Laikago, and Z1 are provided, including mesh, URDF, and Xacro files. For example, to view the Laikago model in RViz:

```bash
ros2 launch laikago_description rviz.launch.py
```

## unitree_gazebo & unitree_controller

Launch the Gazebo simulation with:

```bash
ros2 launch unitree_gazebo gazebo.launch.py robot_name:=a1 world_name:=stairs
```

Specify `robot_name` for the robot type (`laikago`, `aliengo`, `a1`, `go1`) and `world_name` for the environment (`earth`, `space`, `stairs`). The default values are `laikago` and `earth`. In Gazebo, the robot should appear lying on the ground with joints not activated.

### Stand Controller

After launching the Gazebo simulation, start controlling the robot with:

```bash
ros2 run unitree_controller stand_up
```

And you can add external disturbances, like a push or a kick:

```bash
ros2 run unitree_controller apply_force
```

### Position and Pose Publisher

To control the position and pose of the robot without a controller, which is useful for SLAM or vision development:

```bash
ros2 run unitree_controller move_publisher
```

The robot will turn around the origin, which is movement under the world coordinate frame.

## z1_controller

Launch the Z1 Gazebo simulation with:

```bash
ros2 launch unitree_gazebo z1_simulation.launch.py
```

After launching the Gazebo simulation, start controlling the Z1 robot via z1_sdk.  
See [z1_documentation](https://dev-z1.unitree.com) for more details.