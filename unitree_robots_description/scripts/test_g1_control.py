#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class G1ControllerTest(Node):
    def __init__(self):
        super().__init__('g1_controller_test')
        
        # Action clients for different controllers
        self.lower_body_client = ActionClient(
            self, FollowJointTrajectory, '/lower_body_controller/follow_joint_trajectory'
        )
        
        self.left_arm_client = ActionClient(
            self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory'
        )
        
        self.right_arm_client = ActionClient(
            self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory'
        )
        
        self.left_wrist_client = ActionClient(
            self, FollowJointTrajectory, '/left_wrist_controller/follow_joint_trajectory'
        )
        
        self.right_wrist_client = ActionClient(
            self, FollowJointTrajectory, '/right_wrist_controller/follow_joint_trajectory'
        )
        
        self.left_fingers_client = ActionClient(
            self, FollowJointTrajectory, '/left_fingers_controller/follow_joint_trajectory'
        )
        
        self.right_fingers_client = ActionClient(
            self, FollowJointTrajectory, '/right_fingers_controller/follow_joint_trajectory'
        )
        
        # Joint names for each controller
        self.lower_body_joints = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint'
        ]
        
        self.left_arm_joints = [
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint'
        ]
        
        self.right_arm_joints = [
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint'
        ]
        
        self.left_wrist_joints = [
            'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint'
        ]
        
        self.right_wrist_joints = [
            'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'
        ]
        
        self.left_fingers_joints = [
            'left_thumb_1_joint', 'left_thumb_2_joint', 'left_thumb_3_joint', 'left_thumb_4_joint',
            'left_index_1_joint', 'left_index_2_joint',
            'left_middle_1_joint', 'left_middle_2_joint',
            'left_ring_1_joint', 'left_ring_2_joint',
            'left_little_1_joint', 'left_little_2_joint'
        ]
        
        self.right_fingers_joints = [
            'right_thumb_1_joint', 'right_thumb_2_joint', 'right_thumb_3_joint', 'right_thumb_4_joint',
            'right_index_1_joint', 'right_index_2_joint',
            'right_middle_1_joint', 'right_middle_2_joint',
            'right_ring_1_joint', 'right_ring_2_joint',
            'right_little_1_joint', 'right_little_2_joint'
        ]
        
        # Wait for action servers to be available
        self.get_logger().info('Waiting for action servers...')
        self.wait_for_servers()
        
        # Timer for demonstration
        self.timer = self.create_timer(5.0, self.demo_sequence)
        self.step = 0
        
        self.get_logger().info('G1 Controller Test Node started with action clients')

    def wait_for_servers(self):
        """Wait for all action servers to be available"""
        clients = [
            (self.lower_body_client, 'lower_body'),
            (self.left_arm_client, 'left_arm'),
            (self.right_arm_client, 'right_arm'),
            (self.left_wrist_client, 'left_wrist'),
            (self.right_wrist_client, 'right_wrist'),
            (self.left_fingers_client, 'left_fingers'),
            (self.right_fingers_client, 'right_fingers')
        ]
        
        for client, name in clients:
            if not client.wait_for_server(timeout_sec=10.0):
                self.get_logger().warn(f'{name} action server not available, continuing anyway...')
            else:
                self.get_logger().info(f'{name} action server is ready')

    def create_trajectory_goal(self, joint_names, positions, duration=2.0):
        """Create a trajectory goal for action client"""
        goal = FollowJointTrajectory.Goal()
        
        goal.trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration - int(duration)) * 1e9))
        
        goal.trajectory.points = [point]
        return goal

    def send_goal_async(self, client, goal, controller_name):
        """Send goal asynchronously and set up callbacks"""
        if not client.server_is_ready():
            self.get_logger().warn(f'{controller_name} action server not ready, skipping...')
            return
            
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self.goal_response_callback(fut, controller_name))

    def goal_response_callback(self, future, controller_name):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{controller_name} goal rejected')
            return
        
        self.get_logger().info(f'{controller_name} goal accepted')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.result_callback(fut, controller_name))

    def result_callback(self, future, controller_name):
        """Handle action result"""
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f'{controller_name} goal completed successfully')
        else:
            self.get_logger().warn(f'{controller_name} goal failed with error code: {result.error_code}')

    def demo_sequence(self):
        """Demonstration sequence for G1 robot with independent left/right control using action clients"""
        self.get_logger().info(f'Demo step: {self.step}')
        
        if self.step == 0:
            # Initialize to zero position
            self.get_logger().info('Moving to zero position')
            lower_body_pos = [0.0] * len(self.lower_body_joints)
            left_arm_pos = [0.0] * len(self.left_arm_joints)
            right_arm_pos = [0.0] * len(self.right_arm_joints)
            left_wrist_pos = [0.0] * len(self.left_wrist_joints)
            right_wrist_pos = [0.0] * len(self.right_wrist_joints)
            left_fingers_pos = [0.0] * len(self.left_fingers_joints)
            right_fingers_pos = [0.0] * len(self.right_fingers_joints)
            
        elif self.step == 1:
            # Very slight knee bend and lift arms differently
            self.get_logger().info('Slight knee bend, left arm up, right arm side')
            lower_body_pos = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # Left leg - very slight knee bend
                             0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # Right leg - very slight knee bend
                             0.0, 0.0, 0.0]  # Waist
            
            left_arm_pos = [1.0, 0.3, 0.0, -0.3]   # Left arm - lift up high
            right_arm_pos = [0.0, -0.8, 0.0, 0.0]  # Right arm - extend to side
            
            left_wrist_pos = [0.0, 0.0, 0.0]
            right_wrist_pos = [0.0, 0.0, 0.0]
            left_fingers_pos = [0.0] * len(self.left_fingers_joints)
            right_fingers_pos = [0.0] * len(self.right_fingers_joints)
            
        elif self.step == 2:
            # Keep legs stable, focus on arm/hand movements
            self.get_logger().info('Left hand fist, right hand open - stable stance')
            lower_body_pos = [0.0] * len(self.lower_body_joints)  # Keep legs stable
            
            left_arm_pos = [1.0, 0.5, 0.0, -0.5]   # Left arm - wave position
            right_arm_pos = [0.5, -0.8, 0.0, -0.3] # Right arm - bend elbow
            
            left_wrist_pos = [0.3, 0.3, 0.3]   # Left wrist - rotate
            right_wrist_pos = [0.0, 0.0, 0.0]  # Right wrist - neutral
            
            # Left hand makes fist, right hand stays open
            left_fingers_pos = [
                0.5, 0.5, 0.5, 0.5,  # left thumb - curl
                0.8, 0.8,  # left index - curl
                0.8, 0.8,  # left middle - curl
                0.8, 0.8,  # left ring - curl
                0.8, 0.8   # left little - curl
            ]
            right_fingers_pos = [0.0] * len(self.right_fingers_joints)  # Right hand open
            
        elif self.step == 3:
            # Left hand points, right hand makes peace sign - stable stance
            self.get_logger().info('Left hand points, right hand peace sign - stable stance')
            lower_body_pos = [0.0] * len(self.lower_body_joints)  # Keep legs stable
            
            left_arm_pos = [0.7, 0.2, 0.0, -0.2]   # Left arm - point forward
            right_arm_pos = [0.8, -0.5, 0.0, -0.5] # Right arm - peace sign position
            
            left_wrist_pos = [0.0, 0.0, 0.0]   # Left wrist - neutral for pointing
            right_wrist_pos = [-0.3, 0.2, 0.0] # Right wrist - orient for peace sign
            
            # Left hand points, right hand peace sign
            left_fingers_pos = [
                0.5, 0.5, 0.5, 0.5,  # left thumb - curl
                0.0, 0.0,  # left index - straight (pointing)
                0.8, 0.8,  # left middle - curl
                0.8, 0.8,  # left ring - curl
                0.8, 0.8   # left little - curl
            ]
            right_fingers_pos = [
                0.5, 0.5, 0.5, 0.5,  # right thumb - curl
                0.0, 0.0,  # right index - straight (peace)
                0.0, 0.0,  # right middle - straight (peace)
                0.8, 0.8,  # right ring - curl
                0.8, 0.8   # right little - curl
            ]
            
        elif self.step == 4:
            # Both hands wave goodbye differently - stable stance
            self.get_logger().info('Different wave patterns - stable stance')
            lower_body_pos = [0.0] * len(self.lower_body_joints)  # Keep legs stable
            
            left_arm_pos = [1.2, 0.8, 0.0, -0.3]   # Left arm - high wave
            right_arm_pos = [0.5, -0.8, 0.0, 0.0]  # Right arm - side wave
            
            left_wrist_pos = [0.5, 0.0, 0.5]   # Left wrist - rotate for wave
            right_wrist_pos = [0.0, 0.5, -0.5] # Right wrist - different rotation
            
            # Both hands open for waving
            left_fingers_pos = [0.0] * len(self.left_fingers_joints)
            right_fingers_pos = [0.0] * len(self.right_fingers_joints)
            
        elif self.step == 5:
            # Return to zero
            self.get_logger().info('Returning to zero position')
            lower_body_pos = [0.0] * len(self.lower_body_joints)
            left_arm_pos = [0.0] * len(self.left_arm_joints)
            right_arm_pos = [0.0] * len(self.right_arm_joints)
            left_wrist_pos = [0.0] * len(self.left_wrist_joints)
            right_wrist_pos = [0.0] * len(self.right_wrist_joints)
            left_fingers_pos = [0.0] * len(self.left_fingers_joints)
            right_fingers_pos = [0.0] * len(self.right_fingers_joints)
            
        else:
            # Reset step counter
            self.step = -1
            
        # Send goals to action servers
        if self.step >= 0:
            lower_body_goal = self.create_trajectory_goal(self.lower_body_joints, lower_body_pos)
            left_arm_goal = self.create_trajectory_goal(self.left_arm_joints, left_arm_pos)
            right_arm_goal = self.create_trajectory_goal(self.right_arm_joints, right_arm_pos)
            left_wrist_goal = self.create_trajectory_goal(self.left_wrist_joints, left_wrist_pos)
            right_wrist_goal = self.create_trajectory_goal(self.right_wrist_joints, right_wrist_pos)
            left_fingers_goal = self.create_trajectory_goal(self.left_fingers_joints, left_fingers_pos)
            right_fingers_goal = self.create_trajectory_goal(self.right_fingers_joints, right_fingers_pos)
            
            # Send all goals
            self.send_goal_async(self.lower_body_client, lower_body_goal, 'lower_body')
            self.send_goal_async(self.left_arm_client, left_arm_goal, 'left_arm')
            self.send_goal_async(self.right_arm_client, right_arm_goal, 'right_arm')
            self.send_goal_async(self.left_wrist_client, left_wrist_goal, 'left_wrist')
            self.send_goal_async(self.right_wrist_client, right_wrist_goal, 'right_wrist')
            self.send_goal_async(self.left_fingers_client, left_fingers_goal, 'left_fingers')
            self.send_goal_async(self.right_fingers_client, right_fingers_goal, 'right_fingers')
        
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    
    controller_test = G1ControllerTest()
    
    try:
        rclpy.spin(controller_test)
    except KeyboardInterrupt:
        pass
    
    controller_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 