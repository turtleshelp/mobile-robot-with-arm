#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from moveit_msgs.msg import MoveItErrorCodes

from control_msgs.action import GripperCommand


class MobileManipulationMission(Node):

    def __init__(self):

        super().__init__('mobile_manipulation_mission')

        # =====================================
        # Action clients
        # =====================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.moveit_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # =====================================
        # Arm poses
        # =====================================

        # MoveIt SRDF home pose
        self.arm_home = [
            0.0,
            -1.0,
            0.7,
            0.3
        ]

        # Previously verified stable arm pose
        self.arm_work = [
            0.0,
            -0.30,
            0.20,
            0.10
        ]

        # =====================================
        # Gripper positions
        # =====================================

        self.gripper_open = 0.019
        self.gripper_closed = -0.010

        # =====================================
        # Navigation stations
        #
        # These are safe starting test goals
        # inside the 6 m x 5 m arena.
        # =====================================

        self.station_a = {
            'x': 1.5,
            'y': -1.5,
            'yaw': 0.0,
        }

        self.station_b = {
            'x': -1.8,
            'y': 0.0,
            'yaw': math.pi,
        }

    # =========================================
    # Wait for all required action servers
    # =========================================

    def wait_for_servers(self):

        self.get_logger().info(
            'Waiting for Nav2...'
        )
        self.nav_client.wait_for_server()

        self.get_logger().info(
            'Waiting for MoveIt...'
        )
        self.moveit_client.wait_for_server()

        self.get_logger().info(
            'Waiting for gripper controller...'
        )
        self.gripper_client.wait_for_server()

        self.get_logger().info(
            'All mission action servers are ready.'
        )

    # =========================================
    # Navigation
    # =========================================

    def navigate_to(self, x, y, yaw):

        self.get_logger().info(
            f'Navigating to x={x:.2f}, '
            f'y={y:.2f}, yaw={yaw:.2f}'
        )

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )

        goal.pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        send_future = self.nav_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                'Navigation goal rejected.'
            )

            return False

        self.get_logger().info(
            'Navigation goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if (
            result is not None and
            result.status == GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().info(
                'Navigation goal reached.'
            )

            return True

        self.get_logger().error(
            'Navigation failed.'
        )

        return False

    # =========================================
    # MoveIt arm motion
    # =========================================

    def move_arm(self, positions, name):

        self.get_logger().info(
            f'Moving arm to {name}: {positions}'
        )

        request = MotionPlanRequest()

        request.group_name = 'arm'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0

        request.max_velocity_scaling_factor = 0.30
        request.max_acceleration_scaling_factor = 0.30

        constraints = Constraints()

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
        ]

        for joint_name, position in zip(
            joint_names,
            positions
        ):

            joint_constraint = JointConstraint()

            joint_constraint.joint_name = joint_name
            joint_constraint.position = position

            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01

            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(
                joint_constraint
            )

        request.goal_constraints.append(
            constraints
        )

        goal = MoveGroup.Goal()

        goal.request = request

        # Plan AND execute
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = (
            self.moveit_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                f'MoveIt rejected {name} goal.'
            )

            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        action_result = result_future.result()

        if action_result is None:

            self.get_logger().error(
                f'No MoveIt result for {name}.'
            )

            return False

        moveit_result = action_result.result

        if (
            moveit_result.error_code.val ==
            MoveItErrorCodes.SUCCESS
        ):

            self.get_logger().info(
                f'Arm reached {name}.'
            )

            return True

        self.get_logger().error(
            f'MoveIt failed for {name}. '
            f'Error code: '
            f'{moveit_result.error_code.val}'
        )

        return False

    # =========================================
    # Gripper
    # =========================================

    def move_gripper(self, position, name):

        self.get_logger().info(
            f'Gripper: {name}'
        )

        goal = GripperCommand.Goal()

        goal.command.position = position
        goal.command.max_effort = 1.0

        send_future = (
            self.gripper_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                f'Gripper {name} goal rejected.'
            )

            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        action_result = result_future.result()

        if (
            action_result is not None and
            action_result.status ==
            GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().info(
                f'Gripper {name} complete.'
            )

            return True

        self.get_logger().error(
            f'Gripper {name} failed.'
        )

        return False

    # =========================================
    # Full mission
    # =========================================

    def run_mission(self):

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            'MOBILE MANIPULATION MISSION START'
        )

        self.get_logger().info(
            '================================='
        )

        self.wait_for_servers()

        # ---------------------------------
        # Prepare robot
        # ---------------------------------

        if not self.move_arm(
            self.arm_home,
            'HOME'
        ):
            return False

        if not self.move_gripper(
            self.gripper_open,
            'OPEN'
        ):
            return False

        # ---------------------------------
        # Navigate to Station A
        # ---------------------------------

        if not self.navigate_to(
            self.station_a['x'],
            self.station_a['y'],
            self.station_a['yaw']
        ):
            return False

        # ---------------------------------
        # Simulated pickup sequence
        # ---------------------------------

        if not self.move_arm(
            self.arm_work,
            'WORK / PICK'
        ):
            return False

        if not self.move_gripper(
            self.gripper_closed,
            'CLOSE'
        ):
            return False

        if not self.move_arm(
            self.arm_home,
            'CARRY / HOME'
        ):
            return False

        # ---------------------------------
        # Navigate to Station B
        # ---------------------------------

        if not self.navigate_to(
            self.station_b['x'],
            self.station_b['y'],
            self.station_b['yaw']
        ):
            return False

        # ---------------------------------
        # Simulated place sequence
        # ---------------------------------

        if not self.move_arm(
            self.arm_work,
            'WORK / PLACE'
        ):
            return False

        if not self.move_gripper(
            self.gripper_open,
            'OPEN'
        ):
            return False

        if not self.move_arm(
            self.arm_home,
            'HOME'
        ):
            return False

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            'MISSION COMPLETE'
        )

        self.get_logger().info(
            '================================='
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = MobileManipulationMission()

    success = node.run_mission()

    if not success:
        node.get_logger().error(
            'MISSION ABORTED'
        )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
