#!/usr/bin/env python3
"""
4WD4WS Kinematics Controller Node for TadeoeCar.
Compatible with Gz Sim (Fortress) via ros_gz_bridge.

Steering joints use position control (JointPositionController).
Wheel joints use velocity control (JointController).

Modes: omnidirectional, ackermann, crab.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState


WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']


class FourWSKinematicsNode(Node):

    def __init__(self):
        super().__init__('fourws_kinematics_node')

        self.mode = 'omnidirectional'

        # Robot parameters (configurable via ROS2 parameters)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 1.058)
        self.declare_parameter('track_width', 0.55)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_steering_angle', 2.356)  # 135 deg (270/2)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value

        # Half dimensions for kinematics
        self.half_base = self.wheel_base / 2.0
        self.half_track = self.track_width / 2.0

        # Wheel positions relative to robot center (x, y)
        self.wheel_positions = {
            'front_left':  ( self.half_base,  self.half_track),
            'front_right': ( self.half_base, -self.half_track),
            'rear_left':   (-self.half_base,  self.half_track),
            'rear_right':  (-self.half_base, -self.half_track),
        }

        # Current steering angles from joint states (for feedback)
        self.current_steering = {w: 0.0 for w in WHEELS}

        # Steering publishers (position control: send target angle in radians)
        self.steering_pubs = {
            w: self.create_publisher(
                Float64,
                f'/model/tadeocar/joint/{w}_steering_joint/cmd_pos',
                10,
            )
            for w in WHEELS
        }

        # Wheel publishers (velocity control: send angular velocity in rad/s)
        self.wheel_pubs = {
            w: self.create_publisher(
                Float64,
                f'/model/tadeocar/joint/{w}_wheel_joint/cmd_vel',
                10,
            )
            for w in WHEELS
        }

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/robot_mode', self.mode_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info(
            f'4WS Kinematics Node initialized (position control). '
            f'Mode: {self.mode}. '
            f'L={self.wheel_base}m, W={self.track_width}m, r={self.wheel_radius}m'
        )

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if 'steering' not in name:
                continue
            for w in WHEELS:
                if w in name:
                    self.current_steering[w] = msg.position[i]
                    break

    def mode_callback(self, msg):
        new_mode = msg.data.lower()
        if new_mode in ('omnidirectional', 'ackermann', 'crab'):
            self.mode = new_mode
            self.get_logger().info(f'Mode changed to: {self.mode}')
        else:
            self.get_logger().warn(f'Unknown mode: {new_mode}')

    def cmd_vel_callback(self, msg):
        vx = self._clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        vy = self._clamp(msg.linear.y, -self.max_linear_speed, self.max_linear_speed)
        wz = self._clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)

        if self.mode == 'omnidirectional':
            steering, velocities = self._compute_omnidirectional(vx, vy, wz)
        elif self.mode == 'ackermann':
            steering, velocities = self._compute_ackermann(vx, wz)
        elif self.mode == 'crab':
            steering, velocities = self._compute_crab(vx, vy)
        else:
            return

        self._publish_commands(steering, velocities)

    # -- Kinematics modes --

    def _compute_omnidirectional(self, vx, vy, wz):
        """Each wheel has independent steering angle based on instantaneous velocity."""
        steering = {}
        velocities = {}

        for w, (px, py) in self.wheel_positions.items():
            vw_x = vx - wz * py
            vw_y = vy + wz * px

            if abs(vw_x) < 1e-3 and abs(vw_y) < 1e-3:
                steering[w] = 0.0
                velocities[w] = 0.0
            else:
                angle = math.atan2(vw_y, vw_x)
                angle, direction = self._normalize_steering(angle)
                steering[w] = self._clamp(
                    angle, -self.max_steering_angle, self.max_steering_angle
                )
                speed = math.hypot(vw_x, vw_y)
                velocities[w] = (speed / self.wheel_radius) * direction

        return steering, velocities

    def _compute_ackermann(self, vx, wz):
        """Front wheels steer with geometric Ackermann, rear wheels straight."""
        steering = {w: 0.0 for w in WHEELS}
        velocities = {w: 0.0 for w in WHEELS}

        if abs(vx) < 1e-3 and abs(wz) < 1e-3:
            return steering, velocities

        base_vel = vx / self.wheel_radius

        if abs(wz) > 1e-3:
            R = vx / wz  # signed turn radius

            # Geometric Ackermann: atan(L / (R +/- T/2))
            steering['front_left'] = math.atan2(
                self.wheel_base, R - self.half_track
            ) if abs(R - self.half_track) > 1e-3 else math.copysign(
                math.pi / 2, wz
            )
            steering['front_right'] = math.atan2(
                self.wheel_base, R + self.half_track
            ) if abs(R + self.half_track) > 1e-3 else math.copysign(
                math.pi / 2, wz
            )

            # Differential wheel speeds
            vel_diff = wz * self.half_track / self.wheel_radius
            velocities['front_left'] = base_vel - vel_diff
            velocities['front_right'] = base_vel + vel_diff
            velocities['rear_left'] = base_vel - vel_diff
            velocities['rear_right'] = base_vel + vel_diff
        else:
            velocities = {w: base_vel for w in WHEELS}

        for w in WHEELS:
            steering[w] = self._clamp(
                steering[w], -self.max_steering_angle, self.max_steering_angle
            )

        return steering, velocities

    def _compute_crab(self, vx, vy):
        """All wheels same angle, front/rear opposed for lateral movement."""
        steering = {w: 0.0 for w in WHEELS}
        velocities = {w: 0.0 for w in WHEELS}

        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            return steering, velocities

        if abs(vy) > 1e-3:
            lateral_angle = math.atan2(abs(vy), abs(vx)) if abs(vx) > 1e-3 else math.pi / 2
            lateral_angle = self._clamp(lateral_angle, 0.0, self.max_steering_angle)
            front_angle = math.copysign(lateral_angle, vy)
            rear_angle = -front_angle
        else:
            front_angle = 0.0
            rear_angle = 0.0

        for w in WHEELS:
            steering[w] = front_angle if 'front' in w else rear_angle

        wheel_vel = math.hypot(vx, vy) / self.wheel_radius
        if vx < 0:
            wheel_vel = -wheel_vel
        velocities = {w: wheel_vel for w in WHEELS}

        return steering, velocities

    # -- Publishing --

    def _publish_commands(self, steering, velocities):
        """Publish steering positions and wheel velocities."""
        for w in WHEELS:
            msg = Float64()
            msg.data = steering[w]
            self.steering_pubs[w].publish(msg)

            msg = Float64()
            msg.data = self._clamp(velocities[w], -100.0, 100.0)
            self.wheel_pubs[w].publish(msg)

    # -- Utilities --

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _normalize_steering(angle):
        """Normalize steering angle to [-pi/2, pi/2], inverting wheel direction if needed."""
        angle = FourWSKinematicsNode._normalize_angle(angle)
        direction = 1.0
        if angle > math.pi / 2:
            angle -= math.pi
            direction = -1.0
        elif angle < -math.pi / 2:
            angle += math.pi
            direction = -1.0
        return angle, direction

    @staticmethod
    def _clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    node = FourWSKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
