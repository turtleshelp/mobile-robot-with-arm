#!/usr/bin/env python3
# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0
"""Find the red cube in the RGB-D camera image and publish its 3D position.

Pipeline, for every synchronized colour + depth frame:
  1. HSV colour threshold for saturated red (two hue bands), then a small
     morphological opening to remove speckles.
  2. Largest red blob above a minimum size = the cube's visible front face.
  3. Median depth over that blob = distance to the front face.
  4. Back-project the blob centroid with the camera intrinsics, then move
     half a cube further along the horizontal viewing direction to get
     the cube center.
  5. Transform from the camera optical frame to the odom frame and publish.

Publishes:
  /cube_detection/position     geometry_msgs/PointStamped (odom frame)
  /cube_detection/debug_image  sensor_msgs/Image with the detection drawn
"""

import math

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transforms
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class CubeDetector(Node):

    def __init__(self):
        super().__init__(
            'cube_detector',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ],
        )

        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('cube_size', 0.04)
        self.declare_parameter('min_area_px', 40)
        self.declare_parameter('min_saturation', 120)
        self.declare_parameter('min_value', 60)

        self.target_frame = self.get_parameter('target_frame').value
        self.cube_size = self.get_parameter('cube_size').value
        self.min_area = self.get_parameter('min_area_px').value
        self.min_sat = self.get_parameter('min_saturation').value
        self.min_val = self.get_parameter('min_value').value

        self.bridge = CvBridge()
        self.camera_info = None

        # The listener gets its own node and thread: the image callback
        # waits for the transform at the image time, and without a
        # separate thread that wait would block the TF updates it is
        # waiting for. A separate node, because spinning this node from
        # two threads would race in rclpy.
        self.tf_buffer = Buffer()
        self.tf_node = rclpy.create_node(
            'cube_detector_tf',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ],
        )
        self.tf_listener = TransformListener(
            self.tf_buffer, self.tf_node, spin_thread=True
        )

        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._info_cb, 10
        )

        rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw'
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.05
        )
        self.sync.registerCallback(self._frames_cb)

        self.pos_pub = self.create_publisher(
            PointStamped, '/cube_detection/position', 10
        )
        self.debug_pub = self.create_publisher(
            Image, '/cube_detection/debug_image', 2
        )

        self.get_logger().info('Cube detector ready.')

    def _info_cb(self, msg):
        self.camera_info = msg

    def find_red_blob(self, bgr):
        """Largest saturated-red region: (mask, contour) or (None, None)."""

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        low = cv2.inRange(
            hsv, (0, self.min_sat, self.min_val), (10, 255, 255)
        )
        high = cv2.inRange(
            hsv, (170, self.min_sat, self.min_val), (180, 255, 255)
        )
        mask = cv2.morphologyEx(
            low | high, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, None

        blob = max(contours, key=cv2.contourArea)
        if cv2.contourArea(blob) < self.min_area:
            return None, None

        blob_mask = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(blob_mask, [blob], -1, 255, thickness=cv2.FILLED)
        return blob_mask, blob

    def _frames_cb(self, rgb_msg, depth_msg):

        if self.camera_info is None:
            return

        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg)

        blob_mask, blob = self.find_red_blob(bgr)
        debug = bgr.copy()

        if blob is not None:
            d_vals = depth[(blob_mask > 0) & np.isfinite(depth)]
            if d_vals.size > 0:
                self.publish_cube(
                    rgb_msg, blob, float(np.median(d_vals)), debug
                )

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    def to_target_frame(self, pt):
        """Transform at the image time if possible. The robot-position
        transform can lag the camera by a few frames when the simulation
        is slow; then use the latest one. The mission only uses
        detections taken while the robot stands still, where both give
        the same result."""

        try:
            return self.tf_buffer.transform(
                pt, self.target_frame, timeout=Duration(seconds=0.2)
            )
        except TransformException:
            pass

        latest = PointStamped()
        latest.header.frame_id = pt.header.frame_id
        latest.point = pt.point
        try:
            out = self.tf_buffer.transform(
                latest, self.target_frame, timeout=Duration(seconds=0.2)
            )
        except TransformException as error:
            self.get_logger().warning(
                f'No transform to {self.target_frame}: {error}',
                throttle_duration_sec=5.0,
            )
            return None

        out.header.stamp = pt.header.stamp
        return out

    def publish_cube(self, rgb_msg, blob, face_depth, debug):

        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]

        m = cv2.moments(blob)
        u = m['m10'] / m['m00']
        v = m['m01'] / m['m00']

        # Front-face point in the optical frame (z forward, x right, y down)
        x = (u - cx) * face_depth / fx
        y = (v - cy) * face_depth / fy
        z = face_depth

        # Cube center: half a cube further along the horizontal ray
        horiz = math.hypot(x, z)
        half = self.cube_size / 2.0
        x += half * x / horiz
        z += half * z / horiz

        pt = PointStamped()
        pt.header = rgb_msg.header
        pt.point.x, pt.point.y, pt.point.z = x, y, z

        out = self.to_target_frame(pt)
        if out is None:
            return

        self.pos_pub.publish(out)

        bx, by, bw, bh = cv2.boundingRect(blob)
        cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.drawMarker(debug, (int(u), int(v)), (0, 255, 0),
                       cv2.MARKER_CROSS, 12, 2)
        cv2.putText(
            debug,
            f'cube  {face_depth:.2f} m  '
            f'({out.point.x:.2f}, {out.point.y:.2f}, {out.point.z:.2f})',
            (bx - 60, max(by - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 255, 0), 1, cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
