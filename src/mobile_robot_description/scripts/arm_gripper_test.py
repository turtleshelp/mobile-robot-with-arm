#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


class ArmGripperTest(Node):

    def __init__(self):
        super().__init__('arm_gripper_test')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
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

    def move_gripper(self, position):

        self.gripper_client.wait_for_server()

        goal = GripperCommand.Goal()

        goal.command.position = position
        goal.command.max_effort = 1.0

        self.gripper_client.send_goal_async(goal)

        self.get_logger().info(
            f'Gripper command: {position}'
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
    node.move_gripper(0.019)

    rclpy.spin_once(node, timeout_sec=0.5)
    time.sleep(2.0)

    # Close gripper
    node.move_gripper(-0.010)

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

