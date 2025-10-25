from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import launch_ros
import os
import xacro
import launch
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.actions import AppendEnvironmentVariable
from os.path import join
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


gz_version = 6
# gz_version = PythonExpression(["$0", " == 6 or ", "$0", " == 7 or ", "$0", " == 8"])

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_robots_description"
    ).find("unitree_robots_description")
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    
    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="g1",
        description="unitree robot model name",
        choices=["h1", "a1", "b1", "g1", "g1_simple"],
    )
    model = LaunchConfiguration("model")
    
    # Robot URDF path based on model
    robot_urdf = PythonExpression([
        "'", pkg_share, "/models/g1/g1_29dof_with_control.urdf.xacro' if '", model, "' == 'g1' else '",
        pkg_share, "/models/g1/g1_simple_hand_with_control.urdf.xacro' if '", model, "' == 'g1_simple' else '",
        pkg_share, "/models/h1/h1.urdf.xacro'"
    ])
    
    # # Control config path based on model  
    # control_config = PythonExpression([
    #     "'", pkg_share, "/config/g1_control.yaml' if '", model, "' == 'g1' else '",
    #     pkg_share, "/config/g1_simple_hand_control.yaml' if '", model, "' == 'g1_simple' else '",
    #     pkg_share, "/config/", model, "_control.yaml'"
    # ])

    desc = Command(["xacro ", robot_urdf])

    gz_sim_resource_path = AppendEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH' if gz_version == 7 or gz_version == 8 else 'IGN_GAZEBO_RESOURCE_PATH',
    value=join(pkg_share, "models"))
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': desc}
        ],
    )
    
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            PythonExpression(["'", model, "_robot'"]),
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y", 
            "0.0",
            "-z",
            "1.250",
        ],
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            'gz_args': ["empty.sdf"," -r"],
            'use_sim_time': 'True',
        }.items(),
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    lower_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lower_body_controller", "-c", "/controller_manager"],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )
    
    left_wrist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_wrist_controller", "-c", "/controller_manager"],
    )
    
    right_wrist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_wrist_controller", "-c", "/controller_manager"],
    )
    
    # G1 with inspire hands - finger controllers
    left_fingers_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_fingers_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", model, "' == 'g1'"]))
    )
    
    right_fingers_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_fingers_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", model, "' == 'g1'"]))
    )
    
    # G1 with simple hands - simple hand controllers
    left_simple_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_simple_hand_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", model, "' == 'g1_simple'"]))
    )
    
    right_simple_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_simple_hand_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", model, "' == 'g1_simple'"]))
    )

    full_body_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["full_body_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", model, "' == 'h1'"]))
    )
    # Event handlers for sequential controller spawning
    spawn_entity_finished = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    joint_state_broadcaster_finished = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                lower_body_controller_spawner, 
                left_arm_controller_spawner, 
                right_arm_controller_spawner,
                left_wrist_controller_spawner, 
                right_wrist_controller_spawner,
                left_fingers_controller_spawner,      # G1 inspire hands
                right_fingers_controller_spawner,     # G1 inspire hands
                left_simple_hand_controller_spawner,  # G1 simple hands
                right_simple_hand_controller_spawner,  # G1 simple hands
                full_body_controller # H1
            ]
        )
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # "/world/default/model/prometheus/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ]
    )

    return LaunchDescription(
      [ 
        model_arg,
        gz_sim_resource_path,
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        spawn_entity_finished,
        joint_state_broadcaster_finished,
        gz_ros2_bridge
      ]
    )
