#!/usr/bin/env python3
"""Save the cube detector's debug image during a recorded demo run.

Writes every /cube_detection/debug_image frame as a JPEG to
<record_dir>/detector/frame-<n>.jpg. make_demo_video.py matches them to
the scene cameras by file time and shows them as an inset while the
robot looks for the cube.

Usage:
  record_detector_view.py <record_dir>
"""

import os
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DetectorViewRecorder(Node):

    def __init__(self, out_dir):
        super().__init__('detector_view_recorder')
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.count = 0
        self.create_subscription(
            Image, '/cube_detection/debug_image', self._cb, 5
        )

    def _cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        path = os.path.join(self.out_dir, f'frame-{self.count}.jpg')
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        self.count += 1


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    rclpy.init()
    node = DetectorViewRecorder(os.path.join(sys.argv[1], 'detector'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
