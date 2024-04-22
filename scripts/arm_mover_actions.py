#!/usr/bin/python3

import rclpy
import rclpy.duration
from rclpy.node import Node

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class ArmMoverAction(Node):
    def __init__(self):
        super().__init__('transform_point')

        # Basic ROS stuff
        timer_frequency = 1
        timer_period = 1/timer_frequency

        # General variables for setting the arm position
        self.new_command_arrived = False
        self.executing_command = False

        # Subscribers / Publishers
        self.arm_command_sub = self.create_subscription(String, "/arm_command", self.arm_command_callback, 1)
        self.arm_position_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Predefined positions for the robot arm
        self.joint_names = ['arm_base_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint']
        self.arm_poses = {'look_for_parking':[0.,0.4,1.5,1.2],
                          'look_for_qr':[0.,0.6,0.5,2.0],
                          'garage':[0.,-0.45,2.8,-0.8],
                          'up':[0.,0.,0.,0.],
                          'manual':None}

        self.get_logger().info(f"Initialized the Arm Mover node! Waiting for commands...")

    def set_arm_position(self, command_string):

        self.executing_command = True

        while not self.arm_position_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'Arm controller' action server not available, waiting...")

        point = JointTrajectoryPoint()

        command = self.arm_poses[command_string.split(':')[0]] # Only beacause we want to detect manual setting of position
        # self.get_logger().info(f"Recieved command {command}")
        if command is None:
            self.get_logger().info(f"Recieved command MANUAL command {command_string.split(':')[1]}")
            point.positions = eval(command_string.split(':')[1])
        else:
            point.positions = command
        point.time_from_start = rclpy.duration.Duration(seconds=3.).to_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = rclpy.duration.Duration(seconds=3.).to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending a goal to the action server, position is {command}')
        self.send_goal_future = self.arm_position_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_accepted_callback)

        self.new_command_arrived = False
    
    def goal_accepted_callback(self, future):
        goal_handle = future.result()

        # If the goal was accepted
        if goal_handle.accepted: 
            self.get_logger().info('Arm controller ACCEPTED the goal.')

            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.get_result_callback)

        elif not goal_handle.accepted:
            self.get_logger().error('Arm controller REJECTED the goal.')
            
            self.executing_command = False

    def get_result_callback(self, future):
        status = future.result().status
        
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arm controller says GOAL FAILED: {status}')
        else:
            self.get_logger().info(f'Arm controller says GOAL REACHED.')
        
        self.executing_command = False


    def timer_callback(self):
        if self.new_command_arrived and not self.executing_command:

            self.set_arm_position(self.current_command)

            self.get_logger().info(f"Will set a new position for the arm joints: {self.current_command}")

            self.previous_command = self.current_command
            self.new_command_arrived = False
        

    def arm_command_callback(self, msg):
        command_string = msg.data.strip().lower()
        command_test = msg.data.strip().lower().split(":")[0] # Split is only needed to be able to set a position manually

        assert command_test in list(self.arm_poses.keys())

        self.current_command = command_string
        self.new_command_arrived = True

        self.get_logger().info(f"Got a new command for the arm configuration: {command_string}")


    # def arm_command_string_to_msg(self, command_string):
    #     positions = self.arm_poses[command_string]

    #     point = JointTrajectoryPoint()
    #     point.positions = positions
    #     point.time_from_start = rclpy.duration.Duration(seconds=3.).to_msg()

    #     pos_msg = JointTrajectory()
    #     pos_msg.header.stamp = self.get_clock().now().to_msg()

    #     pos_msg.joint_names = self.joint_names
    #     pos_msg.points.append(point)

    #     return pos_msg


def main():

    rclpy.init(args=None)
    rd_node = ArmMoverAction()

    rclpy.spin(rd_node)

if __name__ == '__main__':
    main()