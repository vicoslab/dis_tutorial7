#!/usr/bin/python3

import rclpy
import rclpy.duration
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class ArmMover(Node):
    def __init__(self):
        super().__init__('transform_point')

        # Basic ROS stuff
        timer_frequency = 1
        timer_period = 1/timer_frequency

        # General variables for setting the arm position
        self.new_command_arrived = False
        self.previous_command = ''
        self.current_command = ''

        # Subscribers / Publishers
        self.arm_command_sub = self.create_subscription(String, "/arm_command", self.arm_command_callback, 1)
        self.arm_position_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 1)

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Predefined positions for the robot arm
        self.arm_poses = {'reatracted':[0.,0.,0.,0.,0.],
                          'look_for_parking':[0.,0.,0.,0.,0.],
                          'look_for_qr':[0.,0.,0.,0.,0.],
                          'test':[1.5]}

        self.get_logger().info(f"Initialized the Arm Mover node! Waiting for commands...")


    def timer_callback(self):
        if self.new_command_arrived and self.current_command != self.previous_command:
            pos_msg = self.arm_command_string_to_msg(self.current_command)
            self.arm_position_pub.publish(pos_msg)

            self.get_logger().info(f"Will set a new position for the arm joints: {self.arm_poses[self.current_command]}")

            self.previous_command = self.current_command
            self.new_command_arrived = False
        

    def arm_command_callback(self, msg):
        command = msg.data.strip().lower()

        assert command in list(self.arm_poses.keys())

        self.current_command = command
        self.new_command_arrived = True

        self.get_logger().info(f"Got a new command for the arm configuration: {command}")


    def arm_command_string_to_msg(self, command_string):
        positions = self.arm_poses[command_string]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(seconds=1.).to_msg()

        pos_msg = JointTrajectory()
        pos_msg.header.stamp = self.get_clock().now().to_msg()

        pos_msg.joint_names = ['joint_0']
        pos_msg.points.append(point)

        return pos_msg


def main():

    rclpy.init(args=None)
    rd_node = ArmMover()

    rclpy.spin(rd_node)

if __name__ == '__main__':
    main()