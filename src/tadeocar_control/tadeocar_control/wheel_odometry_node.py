#!/usr/bin/env python3
"""Wheel states -> odometry, for a 4WD4WS robot.

This node exists because the simulation had no odometry at all. What it had was
Gazebo's OdometryPublisher, which derives its output from the model's world
pose: it is ground truth wearing an odometry topic's name. Everything
downstream - SLAM Toolbox's scan matcher, Nav2's costmaps, RTAB-Map's motion
prediction - was being handed a perfect answer, so nothing in the stack was
ever tested against the error the real robot will have. Ground truth still
comes out of Gazebo, on /odom_truth, where it belongs: as the thing results are
scored against.

The estimate here uses only what the real robot can measure: four wheel
rotation rates and four steering angles.

---------------------------------------------------------------------------
The estimator
---------------------------------------------------------------------------
For a rigid body, the velocity of the point at p = (px, py) is

    v_p = (vx - wz*py,  vy + wz*px)

A wheel that is not slipping travels along its own steering direction at its
rolling speed, so for wheel i

    omega_i * r * cos(delta_i) = vx - wz*py_i
    omega_i * r * sin(delta_i) = vy + wz*px_i

Four wheels give eight equations for three unknowns. The system is
overdetermined, which is the point: on a 4WS robot the wheels disagree whenever
one of them slips, and least squares spreads that disagreement over all four
instead of trusting whichever pair a closed-form solution happened to pick. The
matrix depends only on where the wheels are, so it is inverted once at startup
and each cycle is one matrix-vector product.

---------------------------------------------------------------------------
What it cannot know
---------------------------------------------------------------------------
Everything above is planar. Drive up the yard world's 6.7 degree ramp and this
node reports the distance travelled as horizontal distance and never changes z,
because a wheel encoder cannot tell forward from up. That error is not noise
and no amount of tuning removes it; it is removed by fusing an attitude sensor,
which is what tadeocar_perception's EKF does with the ZED 2i's IMU.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('wheel_radius', 0.125)
        self.declare_parameter('front_axle_x', 0.478)
        self.declare_parameter('rear_axle_x', -0.580)
        self.declare_parameter('half_track', 0.275)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Whether this node owns odom -> base_footprint. It does when it is the
        # only estimator running; it does not when the EKF is up, because a
        # frame has exactly one parent and two publishers of one transform is a
        # broken TF tree, not a merged one.
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        front_x = self.get_parameter('front_axle_x').value
        rear_x = self.get_parameter('rear_axle_x').value
        half_track = self.get_parameter('half_track').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.positions = {
            'front_left': (front_x, half_track),
            'front_right': (front_x, -half_track),
            'rear_left': (rear_x, half_track),
            'rear_right': (rear_x, -half_track),
        }

        # Geometry matrix, two rows per wheel. Constant, so pseudo-invert once.
        rows = []
        for w in WHEELS:
            px, py = self.positions[w]
            rows.append([1.0, 0.0, -py])
            rows.append([0.0, 1.0, px])
        self.pinv = np.linalg.pinv(np.array(rows))

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_stamp = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 20)

        self.get_logger().info(
            f'wheel odometry up: r={self.wheel_radius:.3f} m, '
            f'axles at {front_x:+.3f} / {rear_x:+.3f} m, '
            f'track={2*half_track:.3f} m, publish_tf={self.publish_tf}')

    def on_joint_states(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        b = np.zeros(8)
        for k, w in enumerate(WHEELS):
            steer = index.get(f'{w}_steering_joint')
            drive = index.get(f'{w}_wheel_joint')
            if steer is None or drive is None:
                return                      # partial message, wait for the next
            delta = msg.position[steer]
            speed = msg.velocity[drive] * self.wheel_radius
            b[2 * k] = speed * math.cos(delta)
            b[2 * k + 1] = speed * math.sin(delta)

        vx, vy, wz = self.pinv @ b

        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = now - self.last_stamp
        self.last_stamp = now
        # Guard both ends: a zero dt divides by nothing useful, and a jump
        # means the clock was reset or messages were dropped, in which case
        # integrating across the gap invents motion that never happened.
        if dt <= 0.0 or dt > 0.5:
            return

        # Second-order integration: the heading used for the step is the one at
        # the middle of it, not at its start. On a robot that turns at 1 rad/s
        # in 20 ms steps the difference is small per step and accumulates into
        # a visible arc-shaped bias over a lap.
        mid = self.theta + 0.5 * wz * dt
        self.x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
        self.y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
        self.theta = math.atan2(math.sin(self.theta + wz * dt),
                                math.cos(self.theta + wz * dt))

        self.publish(stamp, vx, vy, wz)

    def publish(self, stamp, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Covariances are an assumption, not a measurement, and the honest
        # place to say so is here. They encode "the twist is worth listening
        # to, the dead-reckoned pose is not": the EKF in tadeocar_perception
        # fuses vx, vy and vyaw from this message and ignores x, y and yaw,
        # so the large pose numbers below are what makes that ordering true
        # rather than a matter of configuration alone.
        #
        # The twist figures assume roughly 5 % slip on a driven wheel, with
        # lateral velocity trusted half as much because it is the component
        # a steered wheel scrubs away first. Measuring them properly means
        # driving known distances on each of the yard world's three friction
        # patches and comparing against /odom_truth. Until that is done these
        # are round numbers with a rationale, and they are documented as such
        # in docs/mathematical-model/parameters.md.
        odom.pose.covariance[0] = 1e3
        odom.pose.covariance[7] = 1e3
        odom.pose.covariance[14] = 1e3
        odom.pose.covariance[21] = 1e3
        odom.pose.covariance[28] = 1e3
        odom.pose.covariance[35] = 1e3
        odom.twist.covariance[0] = 0.05 ** 2
        odom.twist.covariance[7] = 0.10 ** 2
        odom.twist.covariance[14] = 1e3
        odom.twist.covariance[21] = 1e3
        odom.twist.covariance[28] = 1e3
        odom.twist.covariance[35] = 0.05 ** 2
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
