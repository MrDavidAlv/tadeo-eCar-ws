#!/usr/bin/env python3
"""Subscribe to /odom and publish TF odom -> base_link."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info('odom_to_tf node started')

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header                    # frame_id = "odom"
        # Publish odom -> base_footprint to avoid TF conflict with
        # robot_state_publisher which publishes base_footprint -> base_link
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0          # base_footprint is on the ground
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
