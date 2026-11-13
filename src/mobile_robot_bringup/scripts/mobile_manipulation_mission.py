#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from moveit_msgs.msg import MoveItErrorCodes

from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


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
        # Final approach (docking) I/O
        # =====================================

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_scan = None
        self.latest_odom = None

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

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
        # Navigation stations (pre-grasp poses)
        #
        # Nav2 brings the robot to a staging
        # point 0.70 m in front of each
        # pedestal face, facing it, in open
        # space. The lidar docking step then
        # drives the last ~0.53 m.
        #
        # pickup pedestal: face at x = 2.175,
        #                  center y = -0.30
        # drop pedestal:   face at x = -2.175,
        #                  center y = 0.0
        # =====================================

        self.station_a = {
            'x': 1.475,
            'y': -0.30,
            'yaw': 0.0,
        }

        self.station_b = {
            'x': -1.475,
            'y': 0.0,
            'yaw': math.pi,
        }

        # =====================================
        # Docking parameters
        # =====================================

        # Distance from base_footprint to the
        # table edge when docked. Robot front
        # bumper is at +0.069 m, so this leaves
        # ~0.10 m clearance.
        self.dock_distance = 0.17

        # base_scan frame offset from
        # base_footprint (from the URDF)
        self.scan_offset_x = -0.024

        self.backup_distance = 0.35

    # =========================================
    # Wait for all required action servers
    # =========================================

    def wait_for_servers(self):

        self.get_logger().info(
            'Waiting for Nav2...'
        )
        self.nav_client.wait_for_server()

        # The action server exists before Nav2
        # is fully active; on a slow machine a
        # goal sent too early gets rejected.
        for manager in (
            'lifecycle_manager_localization',
            'lifecycle_manager_navigation',
        ):
            self.wait_until_active(manager)

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
    # Nav2 readiness
    # =========================================

    def wait_until_active(self, manager):

        client = self.create_client(
            Trigger, f'/{manager}/is_active'
        )

        client.wait_for_service()

        while rclpy.ok():

            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)

            result = future.result()

            if result is not None and result.success:
                self.get_logger().info(f'{manager} is active.')
                break

            self.get_logger().info(
                f'Waiting for {manager} to become active...'
            )
            time_end = self.get_clock().now().nanoseconds + 2e9
            while self.get_clock().now().nanoseconds < time_end:
                rclpy.spin_once(self, timeout_sec=0.2)

        self.destroy_client(client)

    # =========================================
    # Sensor callbacks
    # =========================================

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _send_cmd(self, vx, wz):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    # =========================================
    # Table edge from lidar
    # =========================================

    def measure_table_edge(self):
        """Fit a line to the lidar points in front of the robot.

        Returns (distance, yaw_error, lateral_offset, n_points) in the
        base_footprint frame, or None if no table edge is visible.

        distance:       perpendicular distance base_footprint -> edge
        yaw_error:      rotation needed to face the edge squarely (rad)
        lateral_offset: y of the visible edge segment's center (m)
        """

        scan = self.latest_scan

        if scan is None:
            return None

        xs = []
        ys = []

        for i, r in enumerate(scan.ranges):

            if not (scan.range_min < r < scan.range_max):
                continue

            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))

            if abs(a) > math.radians(60):
                continue

            x = r * math.cos(a) + self.scan_offset_x
            y = r * math.sin(a)

            # Only the table: close, roughly in front
            if 0.05 < x < 1.05 and abs(y) < 0.45:
                xs.append(x)
                ys.append(y)

        if len(xs) < 8:
            return None

        # Keep the nearest band of points: the
        # pedestal face toward us (0.25 m wide;
        # seen at 15 deg it spans ~0.065 m in x).
        # Narrow enough to drop the side faces
        # when we are offset sideways.
        x_near = min(xs)
        pts = [
            (x, y) for x, y in zip(xs, ys)
            if x < x_near + 0.07
        ]

        if len(pts) < 5:
            return None

        # Least squares: x = a + b*y
        n = len(pts)
        my = sum(p[1] for p in pts) / n
        mx = sum(p[0] for p in pts) / n
        syy = sum((p[1] - my) ** 2 for p in pts)

        if syy < 1e-6:
            return None

        b = sum(
            (p[1] - my) * (p[0] - mx) for p in pts
        ) / syy
        a = mx - b * my

        # Edge normal angle relative to robot x
        yaw_error = -math.atan(b)
        distance = a * math.cos(math.atan(b))

        y_vals = [p[1] for p in pts]
        lateral = (min(y_vals) + max(y_vals)) / 2.0

        return distance, yaw_error, lateral, n

    def _wait_for_fresh_scan(self, timeout=3.0):
        self.latest_scan = None
        end = self.get_clock().now().nanoseconds + timeout * 1e9

        while self.latest_scan is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > end:
                return False

        return True

    # =========================================
    # Final approach: lidar docking to table
    # =========================================

    def _odom_yaw(self):
        q = self.latest_odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _wait_for_odom(self):
        self.latest_odom = None
        while self.latest_odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def rotate_by(self, angle, timeout=20.0):
        """Rotate in place by angle (rad) using odometry."""

        self._wait_for_odom()
        target = self._odom_yaw() + angle
        start = self.get_clock().now().nanoseconds

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            err = math.atan2(
                math.sin(target - self._odom_yaw()),
                math.cos(target - self._odom_yaw())
            )

            if abs(err) < math.radians(1.0):
                break

            if (
                self.get_clock().now().nanoseconds - start
            ) / 1e9 > timeout:
                self._stop()
                self.get_logger().error('Rotation timed out.')
                return False

            wz = max(-0.5, min(0.5, 1.5 * err))
            if abs(wz) < 0.08:
                wz = math.copysign(0.08, wz)
            self._send_cmd(0.0, wz)

        self._stop()
        return True

    def drive_straight(self, distance, speed=0.06, timeout=20.0):
        """Drive forward (+) or backward (-) using odometry."""

        self._wait_for_odom()
        p0 = self.latest_odom.pose.pose.position
        start = self.get_clock().now().nanoseconds

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            p = self.latest_odom.pose.pose.position
            if math.hypot(p.x - p0.x, p.y - p0.y) >= abs(distance):
                break

            if (
                self.get_clock().now().nanoseconds - start
            ) / 1e9 > timeout:
                self._stop()
                self.get_logger().error('Straight drive timed out.')
                return False

            self._send_cmd(math.copysign(speed, distance), 0.0)

        self._stop()
        return True

    def _measure_settled(self, samples=3):
        """Average a few fresh scans while standing still."""

        results = []
        for _ in range(samples):
            if not self._wait_for_fresh_scan():
                break
            m = self.measure_table_edge()
            if m is not None:
                results.append(m)

        if not results:
            return None

        n = len(results)
        return tuple(
            sum(r[k] for r in results) / n for k in range(3)
        ) + (results[-1][3],)

    def correct_lateral_offset(self, name, tolerance=0.03):

        for attempt in range(2):

            # Face the table squarely first so the
            # lateral measurement is meaningful
            m = self._measure_settled()
            if m is None:
                self.get_logger().error(
                    f'{name}: table edge not visible.'
                )
                return False

            if abs(m[1]) > math.radians(2):
                if not self.rotate_by(m[1]):
                    return False
                m = self._measure_settled()
                if m is None:
                    return False

            lateral = m[2]

            self.get_logger().info(
                f'{name}: dist={m[0]:.3f} m, '
                f'lateral offset={lateral:+.3f} m'
            )

            if abs(lateral) <= tolerance:
                return True

            side = math.copysign(math.pi / 2.0, lateral)

            if not (
                self.rotate_by(side) and
                self.drive_straight(abs(lateral)) and
                self.rotate_by(-side)
            ):
                return False

        return True

    def dock_to_table(self, name, timeout=60.0):

        self.get_logger().info(
            f'Docking to {name} '
            f'(target {self.dock_distance:.2f} m)...'
        )

        if not self._wait_for_fresh_scan():
            self.get_logger().error('No /scan data for docking.')
            return False

        # A diff-drive robot cannot slide sideways,
        # and Nav2 may stop up to ~0.15 m off to the
        # side. Fix that first: turn 90 deg, drive
        # the offset, turn back.
        if not self.correct_lateral_offset(name):
            return False

        start = self.get_clock().now().nanoseconds
        aligned_count = 0
        last = None

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.05)

            elapsed = (
                self.get_clock().now().nanoseconds - start
            ) / 1e9

            if elapsed > timeout:
                self._stop()
                self.get_logger().error(
                    f'Docking to {name} timed out. Last: {last}'
                )
                return False

            m = self.measure_table_edge()

            if m is None:
                self._stop()
                continue

            dist, yaw_err, lateral, n = m
            last = (round(dist, 3), round(yaw_err, 3), round(lateral, 3))

            dist_err = dist - self.dock_distance

            # Turn first if badly misaligned,
            # otherwise drive and steer together
            wz = max(-0.4, min(0.4, 1.5 * yaw_err))

            if abs(yaw_err) > math.radians(8):
                vx = 0.0
            else:
                vx = max(-0.05, min(0.08, 0.6 * dist_err))

            if (
                abs(dist_err) < 0.01 and
                abs(yaw_err) < math.radians(2)
            ):
                self._stop()
                aligned_count += 1

                # Require a few consecutive good
                # readings (scan runs at 5 Hz)
                if aligned_count >= 3:
                    self.get_logger().info(
                        f'Docked at {name}: '
                        f'dist={dist:.3f} m, '
                        f'yaw_err={math.degrees(yaw_err):.1f} deg, '
                        f'lateral={lateral:+.3f} m'
                    )
                    self.dock_result = {
                        'distance': dist,
                        'yaw_error': yaw_err,
                        'lateral': lateral,
                    }
                    return True

                continue

            aligned_count = 0
            self._send_cmd(vx, wz)

    # =========================================
    # Back away from table before Nav2 resumes
    # =========================================

    def back_up(self, distance=None, timeout=15.0):

        distance = distance or self.backup_distance

        self.get_logger().info(f'Backing up {distance:.2f} m...')

        self.latest_odom = None
        while self.latest_odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        p0 = self.latest_odom.pose.pose.position
        start = self.get_clock().now().nanoseconds

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.05)

            p = self.latest_odom.pose.pose.position
            moved = math.hypot(p.x - p0.x, p.y - p0.y)

            if moved >= distance:
                break

            if (
                self.get_clock().now().nanoseconds - start
            ) / 1e9 > timeout:
                self._stop()
                self.get_logger().error('Back-up timed out.')
                return False

            self._send_cmd(-0.08, 0.0)

        self._stop()
        self.get_logger().info('Back-up complete.')
        return True

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

        if not self.dock_to_table('PICKUP PEDESTAL'):
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

        if not self.back_up():
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

        if not self.dock_to_table('DROP PEDESTAL'):
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

        if not self.back_up():
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
