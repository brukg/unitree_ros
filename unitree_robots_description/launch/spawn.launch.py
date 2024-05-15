from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros
import os
import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_robots_description"
    ).find("unitree_robots_description")

    model = LaunchConfiguration("model", default="h1")
    
    robot = os.path.join(pkg_share, "models/h1/urdf/robot.urdf")
    desc = xacro.parse(open(robot))
    xacro.process_doc(desc)
    
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': desc.toxml()}
        ],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "h1_robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "1.50",
        ],
    )
  

    return LaunchDescription(
      [ 
        DeclareLaunchArgument(
            name="model",
            default_value=model,
            description="unitree robot model name",
            choices=["h1", "a1", "b1"],
        ),
        robot_state_publisher_node,
        joint_state_publisher,
        spawn_entity,
      ]
    )
