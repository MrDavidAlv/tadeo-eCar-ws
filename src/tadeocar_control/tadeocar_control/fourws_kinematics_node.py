#!/usr/bin/env python3
"""
4WD4WS Kinematics Controller Node for TadeoeCar
Compatible with Gz Sim (Harmonic) via ros_gz_bridge
Implements: Omnidirectional, Ackermann, and Crab modes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import math


class FourWSKinematicsNode(Node):
    """
    4-Wheel Steering Kinematics Controller

    Modes:
    - omnidirectional: All wheels point in direction of motion (360 mobility)
    - ackermann: Front wheels steer, rear wheels straight (car-like)
    - crab: Lateral movement (all wheels same angle, perpendicular motion)
    """

    def __init__(self):
        super().__init__('fourws_kinematics_node')

        # Control mode
        self.mode = 'omnidirectional'  # Default mode
        self.get_logger().info('4WS Kinematics Node starting in omnidirectional mode')

        # Robot parameters (from TadeoeCar model)
        self.wheel_radius = 0.1  # meters
        self.wheel_base = 1.058  # meters (front to rear)
        self.track_width = 0.55  # meters (left to right)

        # Speed limits
        self.max_linear_speed = 2.0  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.max_steering_angle = 1.57  # radians (~90 degrees) for testing full range

        # Physical steering limits (270 degrees = +/-135 degrees = +/-2.356 rad)
        self.max_physical_steering = math.pi * 0.75  # 135 degrees in radians

        # Current joint states
        self.current_steering = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

        # Publishers for steering (Gz Sim JointController topics)
        self.steering_pubs = {
            'front_left': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/front_left_steering_joint/cmd_vel',
                10
            ),
            'front_right': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/front_right_steering_joint/cmd_vel',
                10
            ),
            'rear_left': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/rear_left_steering_joint/cmd_vel',
                10
            ),
            'rear_right': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/rear_right_steering_joint/cmd_vel',
                10
            )
        }

        # Publishers for wheels (Gz Sim JointController topics)
        self.wheel_pubs = {
            'front_left': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/front_left_wheel_joint/cmd_vel',
                10
            ),
            'front_right': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/front_right_wheel_joint/cmd_vel',
                10
            ),
            'rear_left': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/rear_left_wheel_joint/cmd_vel',
                10
            ),
            'rear_right': self.create_publisher(
                Float64,
                '/model/tadeocar/joint/rear_right_wheel_joint/cmd_vel',
                10
            )
        }

        # Control gains
        self.kp_steering = 15.0
        self.max_steering_vel = 20.0

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            lambda msg: self.cmd_vel_callback(msg, source='nav2'),
            10
        )

        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('4WS Kinematics Node initialized successfully')
        self.get_logger().info('Available modes: omnidirectional, ackermann, crab')

    def joint_state_callback(self, msg):
        """Update current steering angles from joint states"""
        try:
            for i, name in enumerate(msg.name):
                if 'steering' in name:
                    if 'front_left' in name:
                        self.current_steering['front_left'] = msg.position[i]
                    elif 'front_right' in name:
                        self.current_steering['front_right'] = msg.position[i]
                    elif 'rear_left' in name:
                        self.current_steering['rear_left'] = msg.position[i]
                    elif 'rear_right' in name:
                        self.current_steering['rear_right'] = msg.position[i]
        except Exception as e:
            self.get_logger().warn(f'Error reading joint states: {e}')

    def mode_callback(self, msg):
        """Change control mode"""
        new_mode = msg.data.lower()
        if new_mode in ['omnidirectional', 'ackermann', 'crab']:
            self.mode = new_mode
            self.get_logger().info(f'Mode changed to: {self.mode}')
        else:
            self.get_logger().warn(f'Unknown mode: {new_mode}')

    def cmd_vel_callback(self, msg, source='xbox'):
        """Process velocity commands and compute wheel commands"""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Apply speed limits
        linear_x = self.clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
        linear_y = self.clamp(linear_y, -self.max_linear_speed, self.max_linear_speed)
        angular_z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)

        # Select mode based on source
        if source == 'nav2':
            steering, velocities = self.compute_omnidirectional(linear_x, linear_y, angular_z)
        else:
            if self.mode == 'omnidirectional':
                steering, velocities = self.compute_omnidirectional(linear_x, linear_y, angular_z)
            elif self.mode == 'ackermann':
                steering, velocities = self.compute_ackermann(linear_x, angular_z)
            elif self.mode == 'crab':
                steering, velocities = self.compute_crab(linear_x, linear_y)
            else:
                self.get_logger().warn(f'Invalid mode: {self.mode}')
                return

        # Publish commands
        self.publish_commands(steering, velocities)

    def compute_omnidirectional(self, vx, vy, wz):
        """4WD4WS omnidirectional kinematics: each wheel has independent angle"""

        # Wheel positions relative to robot center
        wheel_positions = {
            'front_left':  (self.wheel_base/2,  self.track_width/2),
            'front_right': (self.wheel_base/2, -self.track_width/2),
            'rear_left':   (-self.wheel_base/2, self.track_width/2),
            'rear_right':  (-self.wheel_base/2, -self.track_width/2)
        }

        steering = {}
        velocities = {}

        for wheel, (x, y) in wheel_positions.items():
            # Instantaneous velocity of each wheel
            v_wheel_x = vx - wz * y
            v_wheel_y = vy + wz * x

            # Steering angle for this wheel
            if abs(v_wheel_x) < 0.001 and abs(v_wheel_y) < 0.001:
                steering[wheel] = 0.0
                velocities[wheel] = 0.0
            else:
                angle = math.atan2(v_wheel_y, v_wheel_x)
                angle, direction = self.normalize_steering_angle(angle)
                steering[wheel] = self.clamp(angle, -self.max_steering_angle, self.max_steering_angle)

                # Wheel speed
                speed = math.sqrt(v_wheel_x**2 + v_wheel_y**2)
                velocities[wheel] = (speed / self.wheel_radius) * direction

        return steering, velocities

    def compute_ackermann(self, vx, wz):
        """Ackermann mode: Front wheels steer, rear wheels straight (car-like)"""
        if abs(vx) < 0.001 and abs(wz) < 0.001:
            steering = {
                'front_left': 0.0, 'front_right': 0.0,
                'rear_left': 0.0, 'rear_right': 0.0
            }
            velocities = {
                'front_left': 0.0, 'front_right': 0.0,
                'rear_left': 0.0, 'rear_right': 0.0
            }
        else:
            if abs(wz) < 0.001:
                steering_angle = 0.0
            else:
                R = abs(vx / wz) if abs(wz) > 0.001 else 1000.0
                steering_angle = math.atan(self.wheel_base / R)
                if wz < 0:
                    steering_angle = -steering_angle

            if abs(steering_angle) > 0.001:
                if steering_angle > 0:
                    angle_inner = steering_angle * 1.1
                    angle_outer = steering_angle * 0.9
                    steering = {
                        'front_left': angle_inner, 'front_right': angle_outer,
                        'rear_left': 0.0, 'rear_right': 0.0
                    }
                else:
                    angle_inner = steering_angle * 1.1
                    angle_outer = steering_angle * 0.9
                    steering = {
                        'front_left': angle_outer, 'front_right': angle_inner,
                        'rear_left': 0.0, 'rear_right': 0.0
                    }
            else:
                steering = {
                    'front_left': 0.0, 'front_right': 0.0,
                    'rear_left': 0.0, 'rear_right': 0.0
                }

            base_vel = vx / self.wheel_radius

            if abs(wz) > 0.001:
                vel_diff = wz * self.track_width / (2.0 * self.wheel_radius)
                velocities = {
                    'front_left': base_vel - vel_diff,
                    'front_right': base_vel + vel_diff,
                    'rear_left': base_vel - vel_diff,
                    'rear_right': base_vel + vel_diff
                }
            else:
                velocities = {
                    'front_left': base_vel, 'front_right': base_vel,
                    'rear_left': base_vel, 'rear_right': base_vel
                }

        for key in steering:
            steering[key] = self.clamp(steering[key], -self.max_steering_angle, self.max_steering_angle)

        return steering, velocities

    def compute_crab(self, vx, vy):
        """Crab mode: Lateral movement with opposed front/rear wheels"""
        if abs(vx) < 0.001 and abs(vy) < 0.001:
            steering = {
                'front_left': 0.0, 'front_right': 0.0,
                'rear_left': 0.0, 'rear_right': 0.0
            }
            velocities = {
                'front_left': 0.0, 'front_right': 0.0,
                'rear_left': 0.0, 'rear_right': 0.0
            }
        else:
            total_speed = math.sqrt(vx**2 + vy**2)

            if abs(vy) > 0.001:
                lateral_angle = math.atan2(abs(vy), abs(vx)) if abs(vx) > 0.001 else math.pi / 2
                lateral_angle = self.clamp(lateral_angle, 0.0, self.max_steering_angle)

                if vy > 0:
                    front_angle = lateral_angle
                    rear_angle = -lateral_angle
                else:
                    front_angle = -lateral_angle
                    rear_angle = lateral_angle
            else:
                front_angle = 0.0
                rear_angle = 0.0

            steering = {
                'front_left': front_angle, 'front_right': front_angle,
                'rear_left': rear_angle, 'rear_right': rear_angle
            }

            wheel_vel = total_speed / self.wheel_radius
            if vx < 0:
                wheel_vel = -wheel_vel

            velocities = {
                'front_left': wheel_vel, 'front_right': wheel_vel,
                'rear_left': wheel_vel, 'rear_right': wheel_vel
            }

        return steering, velocities

    def publish_commands(self, steering, velocities):
        """Publish steering and velocity commands to all wheels via Gz Sim"""
        # Publish steering velocities (proportional control to reach target angle)
        for wheel, target_angle in steering.items():
            current_angle = self.current_steering[wheel]

            #TODO(olmerg) aqui se obtiene el angulo que deberia ser lo que se envie al carro directamente 

            error = self.compute_shortest_path(current_angle, target_angle)

            target_normalized = self.normalize_angle(target_angle)
            if abs(target_normalized) > self.max_physical_steering:
                self.get_logger().warn(f'{wheel} target angle {target_normalized:.2f} exceeds physical limit')
                error = 0.0

            steering_velocity = self.kp_steering * error
            steering_velocity = self.clamp(steering_velocity, -self.max_steering_vel, self.max_steering_vel)

            msg = Float64()
            msg.data = steering_velocity

            # REMOVE the velocity controller 
            self.steering_pubs[wheel].publish(msg)

        # Publish wheel velocities directly
        for wheel, velocity in velocities.items():
            velocity = self.clamp(velocity, -100.0, 100.0)

            msg = Float64()
            msg.data = velocity
            self.wheel_pubs[wheel].publish(msg)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def normalize_steering_angle(target_angle):
        """
        Normalize steering angle to [-pi/2, pi/2] range
        Returns: (normalized_angle, direction_multiplier)
        """
        angle = FourWSKinematicsNode.normalize_angle(target_angle)
        direction = 1.0

        if angle > math.pi / 2:
            angle = angle - math.pi
            direction = -1.0
        elif angle < -math.pi / 2:
            angle = angle + math.pi
            direction = -1.0

        return angle, direction

    def compute_shortest_path(self, current_angle, target_angle):
        """Compute shortest path from current to target angle"""
        current = self.normalize_angle(current_angle)
        target = self.normalize_angle(target_angle)

        error = target - current
        error = self.normalize_angle(error)

        return error

    @staticmethod
    def clamp(value, min_val, max_val):
        """Clamp value between min and max"""
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
