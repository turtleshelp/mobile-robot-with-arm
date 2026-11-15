#!/usr/bin/env python3
# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmGripperTest(Node):

    def __init__(self):
        super().__init__('arm_gripper_test')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Both fingers are effort controlled: [left, right] in N,
        # positive opens, negative closes
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        self.get_logger().info('Arm + gripper test started')

    def move_arm(self, positions, seconds=6):

        msg = JointTrajectory()

        msg.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = seconds

        msg.points = [point]

        self.arm_pub.publish(msg)

        self.get_logger().info(
            f'Arm command: {positions}'
        )

    def move_gripper(self, effort):

        while self.gripper_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.2)

        msg = Float64MultiArray()
        msg.data = [float(effort), float(effort)]
        self.gripper_pub.publish(msg)

        self.get_logger().info(
            f'Gripper effort: {effort:+.2f} N per finger'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ArmGripperTest()

    # Move arm to verified stable pose
    node.move_arm(
        [0.0, -0.30, 0.20, 0.10],
        6
    )

    time.sleep(7.0)

    # Open gripper
    node.move_gripper(0.4)

    rclpy.spin_once(node, timeout_sec=0.5)
    time.sleep(2.0)

    # Close gripper
    node.move_gripper(-0.4)

    rclpy.spin_once(node, timeout_sec=0.5)
    time.sleep(2.0)

    # Return arm to zero
    node.move_arm(
        [0.0, 0.0, 0.0, 0.0],
        6
    )

    time.sleep(7.0)

    node.get_logger().info('Test complete')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

