#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transforms
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from moveit_msgs.msg import MoveItErrorCodes

from std_srvs.srv import Trigger


class MobileManipulationMission(Node):

    def __init__(self):

        # Use simulation time like every other node, so all timeouts
        # here are in simulated seconds (the sim may run slower than
        # real time on a busy machine).
        super().__init__(
            'mobile_manipulation_mission',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ],
        )

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

        # Straight-line moves near the cube go directly
        # to the arm controller (no random planner detours)
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Both fingers are effort controlled
        # (JointGroupEffortController): [left, right] in N
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10
        )

        # Simulation grasp assist (Gazebo plugin): attaches the cube
        # to the gripper only if it is really between the jaws.
        self.attach_client = self.create_client(
            Trigger, '/grasp_assist/attach'
        )
        self.detach_client = self.create_client(
            Trigger, '/grasp_assist/detach'
        )

        # =====================================
        # Final approach (docking) I/O
        # =====================================

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_scan = None
        self.latest_odom = None

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.gripper_position = None
        self.arm_position = None

        # Cube position from the camera (cube_detector node, odom frame)
        self.cube_detections = []
        self.cube_odom = None
        self.create_subscription(
            PointStamped, '/cube_detection/position',
            self._cube_cb, 10
        )
        # The TF listener gets its own small node, spun in its own thread.
        # Spinning this node from a second thread would race with the
        # main thread's spin and can crash rclpy's wait set.
        self.tf_buffer = Buffer()
        self.tf_node = rclpy.create_node(
            'mobile_manipulation_mission_tf',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ],
        )
        self.tf_listener = TransformListener(
            self.tf_buffer, self.tf_node, spin_thread=True
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10
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

        # =====================================
        # Arm geometry (from the URDF), all in
        # base_footprint, x forward, z up
        # =====================================

        self.j1_x = -0.080        # joint1 (yaw) axis
        self.sh_z = 0.1775        # joint2 (shoulder) height
        self.link_a = math.hypot(0.024, 0.128)
        self.link_a_angle = math.atan2(0.128, 0.024)
        self.link_b = 0.124
        self.ee_len = 0.126       # link5 -> end_effector_link

        self.joint_limits = [
            (-2.82, 2.82),
            (-1.79, 1.57),
            (-0.94, 1.38),
            (-1.79, 2.04),
        ]

        # =====================================
        # Object geometry (from the world)
        # =====================================

        self.pedestal_top = 0.18
        self.cube_size = 0.04
        # cube center behind the pedestal face
        # (front face 1 cm behind the edge)
        self.cube_inset = 0.01 + self.cube_size / 2.0

        # Stop the end effector this far short of
        # the cube center: the cube is still between
        # the finger pads, and the wide gripper palm
        # (about 3.4 cm behind the end effector, and
        # lower than the pedestal top) stays ~8 mm
        # in front of the pedestal edge. With a
        # position-driven arm any touch throws the
        # cube.
        self.grasp_short = 0.008

        # Grasp this much above the cube center:
        # the palm's lower edge then stays level
        # with the pedestal top
        self.grasp_high = 0.005

        # =====================================
        # Gripper (both fingers effort controlled
        # in simulation, same force on each)
        #
        # Positive effort opens (to the 0.019
        # limit), negative effort closes. The
        # grasp itself is confirmed by the
        # grasp assist, which only attaches the
        # cube if it is between the jaws.
        # =====================================

        self.gripper_open = 0.4      # N per finger
        self.gripper_grasp = -0.2    # N per finger (gentle)

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

        # Expected view on arrival, for the
        # localization check (see the world)
        self.staging_distance = 0.70
        self.pedestal_width = 0.25

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
        while self.gripper_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.2)

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
    # Arm inverse kinematics
    # =========================================

    def arm_ik(self, x, y, z, pitch=0.0):
        """Joint angles that put end_effector_link at (x, y, z) in
        base_footprint with the gripper at the given pitch
        (0 = horizontal, positive = tilted up). Elbow-up solution.
        Returns None if unreachable or outside joint limits."""

        q1 = math.atan2(y, x - self.j1_x)
        r = math.hypot(x - self.j1_x, y)

        wx = r - self.ee_len * math.cos(pitch)
        wz = (z - self.sh_z) - self.ee_len * math.sin(pitch)

        a = self.link_a
        b = self.link_b
        c = (wx * wx + wz * wz - a * a - b * b) / (2.0 * a * b)

        if abs(c) > 1.0:
            return None

        delta = math.acos(c)
        t1 = math.atan2(wz, wx) + math.atan2(
            b * math.sin(delta), a + b * math.cos(delta)
        )
        t2 = t1 - delta

        q2 = self.link_a_angle - t1
        q3 = -t2 - q2
        q4 = -pitch + t2

        q = [q1, q2, q3, q4]

        for v, (lo, hi) in zip(q, self.joint_limits):
            if not (lo <= v <= hi):
                return None

        return q

    def arm_to_point(self, x, y, z, name):

        q = self.arm_ik(x, y, z)

        if q is None:
            self.get_logger().error(
                f'{name}: point ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'is out of reach.'
            )
            return False

        self.get_logger().info(
            f'{name}: EE -> ({x:.3f}, {y:+.3f}, {z:.3f})'
        )

        return self.move_arm(q, name)

    # =========================================
    # Grasp check
    # =========================================

    def _joint_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        arm = ['joint1', 'joint2', 'joint3', 'joint4']
        if all(j in pos for j in arm):
            self.arm_position = [pos[j] for j in arm]
        if 'gripper_left_joint' in pos and 'gripper_right_joint' in pos:
            self.gripper_position = (
                pos['gripper_left_joint'],
                pos['gripper_right_joint'],
            )

    def read_gripper(self, settle=1.0):
        end = self.get_clock().now().nanoseconds + settle * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.gripper_position

    # =========================================
    # Pick and place
    # =========================================

    def fingertip(self, q):
        """Fingertip x and end-effector z in base_footprint (FK)."""
        q1, q2, q3, q4 = q
        t1 = self.link_a_angle - q2
        t2 = -(q2 + q3)
        t3 = -(q2 + q3 + q4)
        r = (self.link_a * math.cos(t1) + self.link_b * math.cos(t2)
             + self.ee_len * math.cos(t3))
        z = (self.link_a * math.sin(t1) + self.link_b * math.sin(t2)
             + self.ee_len * math.sin(t3))
        tip = r + 0.021 * math.cos(t3)
        return self.j1_x + tip * math.cos(q1), self.sh_z + z

    def send_trajectory(self, waypoints, duration, name):

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        n = len(waypoints)
        for i, q in enumerate(waypoints, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            t = duration * i / n
            pt.time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1.0) * 1e9)
            )
            goal.trajectory.points.append(pt)

        send = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()

        if handle is None or not handle.accepted:
            self.get_logger().error(f'{name}: trajectory rejected')
            return False

        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        result = res.result()

        if result is None or result.result.error_code != 0:
            code = None if result is None else result.result.error_code
            self.get_logger().error(f'{name}: trajectory failed ({code})')
            return False

        self.get_logger().info(f'Arm reached {name}.')
        return True

    def arm_joint_move(self, q_target, name, duration=4.0, steps=20,
                       max_tip_x=None):
        """Move all joints evenly from the current pose to q_target
        (deterministic, unlike a sampling planner). If max_tip_x is
        given, the whole path is checked first and the move is refused
        if the fingertips would come closer than that."""

        while self.arm_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        q0 = list(self.arm_position)
        path = [
            [a + (i / steps) * (b - a) for a, b in zip(q0, q_target)]
            for i in range(1, steps + 1)
        ]

        if max_tip_x is not None:
            worst = max(self.fingertip(q)[0] for q in path)
            if worst > max_tip_x:
                self.get_logger().error(
                    f'{name}: path would bring the fingertips to '
                    f'x={worst:.3f} (limit {max_tip_x:.3f}); refusing.'
                )
                return False

        self.get_logger().info(f'Moving arm to {name} (joint path)')
        return self.send_trajectory(path, duration, name)

    def arm_line(self, p0, p1, name, duration=3.0, steps=10):
        """Move the end effector along a straight line p0 -> p1
        (base_footprint), solving IK at each waypoint and sending the
        whole trajectory straight to the arm controller."""

        self.get_logger().info(
            f'{name}: line ({p0[0]:.3f}, {p0[1]:+.3f}, {p0[2]:.3f}) -> '
            f'({p1[0]:.3f}, {p1[1]:+.3f}, {p1[2]:.3f})'
        )

        path = []
        for i in range(1, steps + 1):
            f = i / steps
            q = self.arm_ik(*[a + f * (b - a) for a, b in zip(p0, p1)])
            if q is None:
                self.get_logger().error(f'{name}: waypoint {i} out of reach')
                return False
            path.append(q)

        return self.send_trajectory(path, duration, name)

    def pick_cube(self):

        d = self.dock_result

        # Where the camera saw the cube, in the robot frame now
        seen = self.cube_in_base()
        if seen is None:
            return False

        cx, cy = seen.x, seen.y
        # Height from the known pedestal; the camera's z is logged
        cz = self.pedestal_top + self.cube_size / 2.0

        self.get_logger().info(
            f'Cube (camera) at ({cx:.3f}, {cy:+.3f}, {seen.z:.3f}) '
            f'in base_footprint; pedestal face at {d["distance"]:.3f}'
        )

        # Only grasp what the gripper can safely reach: the cube must
        # be on the pedestal, its front face 0-2.5 cm behind the edge
        # (deeper, and the wide gripper palm would hit the pedestal)
        inset = cx - self.cube_size / 2.0 - d['distance']
        if not (-0.01 <= inset <= 0.025) or abs(cy - d['lateral']) > 0.11:
            self.get_logger().error(
                f'Cube not graspable here: front face {inset * 100:.1f} cm '
                f'behind the pedestal edge, {cy - d["lateral"]:+.3f} m '
                f'from the pedestal center.'
            )
            return False

        gx = cx - self.grasp_short
        gz = cz + self.grasp_high
        pre = (cx - 0.07, cy, gz)
        grasp = (gx, cy, gz)
        lift = (gx, cy, gz + 0.05)

        if not self.move_gripper(self.gripper_open, 'OPEN'):
            return False

        # Even joint move to pre-grasp, checked in advance: the
        # fingertips must stay 2 cm in front of the cube. (MoveIt's
        # random planner does not know the cube and once swung the
        # open gripper into it.)
        q_pre = self.arm_ik(*pre)
        if q_pre is None:
            self.get_logger().error('PRE-GRASP out of reach')
            return False

        cube_front = cx - self.cube_size / 2.0
        if not self.arm_joint_move(
            q_pre, 'PRE-GRASP', max_tip_x=cube_front - 0.02
        ):
            return False

        # Final approach: straight line toward the cube
        if not self.arm_line(pre, grasp, 'GRASP'):
            return False

        if not self.move_gripper(self.gripper_grasp, 'CLOSE ON CUBE'):
            return False

        ok, msg = self.call_trigger(self.attach_client)

        if not ok:
            self.get_logger().error(f'Grasp FAILED: {msg}')
            return False

        self.get_logger().info(f'Grasp OK: {msg}')

        if not self.arm_line(grasp, lift, 'LIFT'):
            return False

        return True

    def call_trigger(self, client, timeout=10.0):

        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'{client.srv_name} not available'

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            return False, 'no response'

        return res.success, res.message

    def place_cube(self):

        d = self.dock_result
        # Same geometry as the pick: cube center sits grasp_short ahead
        # of and grasp_high below the end effector. Release with the
        # cube bottom 3 mm above the pedestal top.
        px = d['distance'] + self.cube_inset - self.grasp_short
        py = d['lateral']
        pz = (
            self.pedestal_top + self.cube_size / 2.0 + 0.003
            + self.grasp_high
        )

        self.get_logger().info(
            f'Placing at ({px:.3f}, {py:+.3f}, {pz:.3f}) '
            f'in base_footprint'
        )

        above = (px, py, pz + 0.05)
        place = (px, py, pz)
        retreat = (px - 0.07, py, pz)

        q_above = self.arm_ik(*above)
        if q_above is None:
            self.get_logger().error('PRE-PLACE out of reach')
            return False

        if not self.arm_joint_move(q_above, 'PRE-PLACE'):
            return False

        if not self.arm_line(above, place, 'PLACE'):
            return False

        # Open the fingers first, while the cube is still attached:
        # detaching with the fingers squeezing (slightly sunk into the
        # cube) made the physics throw the cube away.
        if not self.move_gripper(self.gripper_open, 'OPEN'):
            return False

        ok, msg = self.call_trigger(self.detach_client)
        if not ok:
            self.get_logger().error(f'Release failed: {msg}')
            return False

        self.get_logger().info('Cube released.')

        if not self.arm_line(place, retreat, 'RETREAT'):
            return False

        return True

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
        width = max(y_vals) - min(y_vals)

        return distance, yaw_error, lateral, n, width

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
        avg = [sum(r[k] for r in results) / n for k in range(5)]
        return avg[0], avg[1], avg[2], results[-1][3], avg[4]

    def wait_until_still(self, still_for=0.5, timeout=3.0):
        """Stop and wait until odometry shows no motion, so LiDAR scans
        are not smeared by a robot that is still slowing or turning."""

        self._stop()
        start = self.get_clock().now().nanoseconds
        still_since = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = self.get_clock().now().nanoseconds

            if (now - start) / 1e9 > timeout:
                return False

            if self.latest_odom is None:
                continue

            t = self.latest_odom.twist.twist
            moving = abs(t.linear.x) > 0.005 or abs(t.angular.z) > 0.01

            if moving:
                still_since = None
            elif still_since is None:
                still_since = now
            elif (now - still_since) / 1e9 >= still_for:
                return True

        return False

    def check_pedestal_with_retries(self, name, m, retries=2):
        """Localization check that tolerates a single bad scan: if it
        fails, wait until the robot is still and measure again. A real
        localization error (e.g. facing a wall) fails every time."""

        for attempt in range(retries + 1):
            if self.pedestal_where_expected(name, m, final=attempt == retries):
                return m
            self.wait_until_still()
            m = self._measure_settled()
            if m is None:
                self.get_logger().error(f'{name}: table edge not visible.')
                return None
        return None

    def pedestal_where_expected(self, name, m, final=True):
        """Localization check on arrival: Nav2 stops 0.70 m in front of
        a 0.25 m wide pedestal face. If the LiDAR sees something else
        (e.g. a wall, because AMCL is wrong about where the robot is),
        stop before the arm moves."""

        dist, width = m[0], m[4]
        ok = (
            abs(dist - self.staging_distance) < 0.25 and
            abs(width - self.pedestal_width) < 0.09
        )

        msg = (
            f'{name}: saw a face {width:.2f} m wide at {dist:.2f} m '
            f'(expected {self.pedestal_width:.2f} m wide at about '
            f'{self.staging_distance:.2f} m)'
        )

        if ok:
            self.get_logger().info(f'Localization check OK: {msg}')
        elif not final:
            self.get_logger().warning(
                f'Localization check doubtful, measuring again: {msg}'
            )
        else:
            self.get_logger().error(
                f'Localization check FAILED: {msg}. The robot is probably '
                f'not where AMCL thinks it is.'
            )
        return ok

    def correct_lateral_offset(self, name, tolerance=0.015):

        for attempt in range(3):

            # Face the table squarely first so the
            # lateral measurement is meaningful
            m = self._measure_settled()
            if m is None:
                self.get_logger().error(
                    f'{name}: table edge not visible.'
                )
                return False

            if attempt == 0:
                m = self.check_pedestal_with_retries(name, m)
                if m is None:
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

    # =========================================
    # Camera: locate the cube from the staging pose
    # =========================================

    def _cube_cb(self, msg):
        # Positions must be in a fixed frame so they stay valid while
        # the robot drives on; ignore anything else.
        if msg.header.frame_id == 'odom':
            self.cube_detections.append(msg)

    def locate_cube(self, timeout=5.0, min_detections=5):
        """Average a few camera detections while standing still at the
        staging pose (0.7 m from the pedestal, where the cube is in the
        camera's view) and remember the cube in the odom frame."""

        self.get_logger().info('Looking for the cube with the camera...')

        self.cube_detections = []
        start = self.get_clock().now()
        end = start.nanoseconds + timeout * 1e9

        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            fresh = [
                d for d in self.cube_detections
                if Time.from_msg(d.header.stamp) >= start
            ]
            if len(fresh) >= min_detections:
                break
        else:
            self.get_logger().error(
                'Cube not found by the camera '
                f'({len(self.cube_detections)} detections).'
            )
            return False

        xs = sorted(d.point.x for d in fresh)
        ys = sorted(d.point.y for d in fresh)
        zs = sorted(d.point.z for d in fresh)
        mid = len(fresh) // 2

        self.cube_odom = PointStamped()
        self.cube_odom.header.frame_id = fresh[0].header.frame_id
        self.cube_odom.point.x = xs[mid]
        self.cube_odom.point.y = ys[mid]
        self.cube_odom.point.z = zs[mid]

        self.get_logger().info(
            f'Cube seen by the camera at ({xs[mid]:.3f}, {ys[mid]:.3f}, '
            f'{zs[mid]:.3f}) in {self.cube_odom.header.frame_id} '
            f'({len(fresh)} detections).'
        )
        return True

    def cube_in_base(self):
        """The remembered cube position in base_footprint, now."""

        try:
            return self.tf_buffer.transform(
                self.cube_odom, 'base_footprint',
                timeout=RclpyDuration(seconds=1.0)
            ).point
        except TransformException as error:
            self.get_logger().error(f'Cube transform failed: {error}')
            return None

    def dock_to_table(self, name, timeout=60.0, at_staging=None):

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
        # Nav2 may still be slowing down; measure only once still
        self.wait_until_still()

        if not self.correct_lateral_offset(name):
            return False

        # Something to do while still 0.7 m away, facing the
        # pedestal (e.g. look for the cube with the camera)
        if at_staging is not None and not at_staging():
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

            dist, yaw_err, lateral, n, _ = m
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

    def move_gripper(self, effort, name, settle=1.5):
        """Apply the same effort to both fingers and wait for them to
        settle (open: against the joint limit; close: on the object)."""

        self.get_logger().info(f'Gripper: {name} ({effort:+.2f} N)')

        msg = Float64MultiArray()
        msg.data = [float(effort), float(effort)]
        self.gripper_pub.publish(msg)

        pos = self.read_gripper(settle)

        if pos is None:
            self.get_logger().error('No gripper joint states.')
            return False

        self.get_logger().info(
            f'Gripper {name} complete: '
            f'fingers at ({pos[0]:.4f}, {pos[1]:.4f})'
        )
        return True

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
        self.traj_client.wait_for_server()

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

        if not self.dock_to_table(
            'PICKUP PEDESTAL', at_staging=self.locate_cube
        ):
            return False

        # ---------------------------------
        # Pick the cube
        # ---------------------------------

        if not self.pick_cube():
            return False

        if not self.arm_joint_move(self.arm_home, 'CARRY / HOME'):
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
        # Place the cube
        # ---------------------------------

        if not self.place_cube():
            return False

        if not self.arm_joint_move(self.arm_home, 'HOME'):
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
