#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.amcl_state_client = self.create_client(
            GetState,
            '/amcl/get_state'
        )

        self.publish_count = 0
        self.waiting_logged = False

        self.timer = self.create_timer(
            1.0,
            self.check_amcl
        )

    def check_amcl(self):

        if not self.amcl_state_client.service_is_ready():

            if not self.waiting_logged:
                self.get_logger().info(
                    'Waiting for AMCL lifecycle service...'
                )
                self.waiting_logged = True

            return

        request = GetState.Request()

        future = self.amcl_state_client.call_async(request)

        future.add_done_callback(self.state_response)

    def state_response(self, future):

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                f'Could not read AMCL state: {error}'
            )
            return

        # Lifecycle PRIMARY_STATE_ACTIVE = 3
        if response.current_state.id != 3:

            self.get_logger().info(
                f'Waiting for AMCL to become active '
                f'(current state: {response.current_state.label})'
            )

            return

        self.publish_initial_pose()

    def publish_initial_pose(self):

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher.publish(msg)

        self.publish_count += 1

        self.get_logger().info(
            f'Published initial AMCL pose '
            f'({self.publish_count}/3)'
        )

        if self.publish_count >= 3:

            self.get_logger().info(
                'AMCL initial pose initialization complete.'
            )

            self.timer.cancel()

            # Allow the final message to leave the publisher.
            self.create_timer(
                1.0,
                self.shutdown_node
            )

    def shutdown_node(self):

        self.get_logger().info(
            'Initial pose publisher finished.'
        )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = InitialPosePublisher()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
