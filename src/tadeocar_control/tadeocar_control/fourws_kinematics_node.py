#!/usr/bin/env python3
"""
4WD4WS Kinematics Controller Node for TadeoeCar
Compatible with Gazebo Classic + ros2_control
Implements: Omnidirectional, Ackermann, and Crab modes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
import math


class FourWSKinematicsNode(Node):
    """
    4-Wheel Steering Kinematics Controller

    Modes:
    - omnidirectional: All wheels point in direction of motion (360° mobility)
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
        self.max_steering_angle = 0.5  # radians (~28.6 degrees)

        # Current joint states
        self.current_steering = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

        # Publishers for steering (ros2_control velocity controllers)
        self.steering_pubs = {
            'front_left': self.create_publisher(
                Float64MultiArray,
                '/front_left_steering_controller/commands',
                10
            ),
            'front_right': self.create_publisher(
                Float64MultiArray,
                '/front_right_steering_controller/commands',
                10
            ),
            'rear_left': self.create_publisher(
                Float64MultiArray,
                '/rear_left_steering_controller/commands',
                10
            ),
            'rear_right': self.create_publisher(
                Float64MultiArray,
                '/rear_right_steering_controller/commands',
                10
            )
        }

        # Publishers for wheels (ros2_control velocity controllers)
        self.wheel_pubs = {
            'front_left': self.create_publisher(
                Float64MultiArray,
                '/front_left_wheel_controller/commands',
                10
            ),
            'front_right': self.create_publisher(
                Float64MultiArray,
                '/front_right_wheel_controller/commands',
                10
            ),
            'rear_left': self.create_publisher(
                Float64MultiArray,
                '/rear_left_wheel_controller/commands',
                10
            ),
            'rear_right': self.create_publisher(
                Float64MultiArray,
                '/rear_right_wheel_controller/commands',
                10
            )
        }

        # PID gains
        self.kp_steering = 50.0  # Proportional gain for steering
        self.kp_wheel = 10.0      # Proportional gain for wheels

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
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
        self.get_logger().info(f'Available modes: omnidirectional, ackermann, crab')

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

    def cmd_vel_callback(self, msg):
        """Process velocity commands and compute wheel commands"""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Apply speed limits
        linear_x = self.clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
        linear_y = self.clamp(linear_y, -self.max_linear_speed, self.max_linear_speed)
        angular_z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)

        # Compute kinematics based on mode
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
        """
        Omnidirectional mode: All wheels point in direction of motion

        Args:
            vx: Linear velocity in x (forward/backward)
            vy: Linear velocity in y (left/right)
            wz: Angular velocity (rotation)

        Returns:
            steering: Dict of steering angles for each wheel
            velocities: Dict of wheel velocities
        """
        # Compute direction angle from velocity components
        if abs(vx) < 0.001 and abs(vy) < 0.001:
            # Pure rotation or stopped
            if abs(wz) > 0.001:
                # Spin in place - 45 degree diamond pattern
                steering = {
                    'front_left': math.pi / 4,    # 45°
                    'front_right': -math.pi / 4,  # -45°
                    'rear_left': -math.pi / 4,    # -45°
                    'rear_right': math.pi / 4     # 45°
                }

                # Velocity proportional to distance from center
                wheel_distance = math.sqrt((self.wheel_base/2)**2 + (self.track_width/2)**2)
                base_vel = wz * wheel_distance / self.wheel_radius

                velocities = {
                    'front_left': base_vel,
                    'front_right': base_vel,
                    'rear_left': base_vel,
                    'rear_right': base_vel
                }
            else:
                # Stopped
                steering = {
                    'front_left': 0.0,
                    'front_right': 0.0,
                    'rear_left': 0.0,
                    'rear_right': 0.0
                }
                velocities = {
                    'front_left': 0.0,
                    'front_right': 0.0,
                    'rear_left': 0.0,
                    'rear_right': 0.0
                }
        else:
            # Translational motion (with possible rotation)
            angle = math.atan2(vy, vx)
            speed = math.sqrt(vx**2 + vy**2)

            # All wheels point in same direction
            steering = {
                'front_left': angle,
                'front_right': angle,
                'rear_left': angle,
                'rear_right': angle
            }

            # Convert linear speed to wheel angular velocity
            wheel_vel = speed / self.wheel_radius

            # Add rotational component
            if abs(wz) > 0.001:
                wheel_distance = math.sqrt((self.wheel_base/2)**2 + (self.track_width/2)**2)
                rot_contribution = wz * wheel_distance / self.wheel_radius

                velocities = {
                    'front_left': wheel_vel + rot_contribution,
                    'front_right': wheel_vel + rot_contribution,
                    'rear_left': wheel_vel + rot_contribution,
                    'rear_right': wheel_vel + rot_contribution
                }
            else:
                velocities = {
                    'front_left': wheel_vel,
                    'front_right': wheel_vel,
                    'rear_left': wheel_vel,
                    'rear_right': wheel_vel
                }

        # Clamp steering angles
        for key in steering:
            steering[key] = self.clamp(steering[key], -self.max_steering_angle, self.max_steering_angle)

        return steering, velocities

    def compute_ackermann(self, vx, wz):
        """
        Ackermann mode: Front wheels steer, rear wheels straight (car-like)

        Args:
            vx: Linear velocity (forward/backward)
            wz: Angular velocity (turning rate)

        Returns:
            steering: Dict of steering angles for each wheel
            velocities: Dict of wheel velocities
        """
        if abs(vx) < 0.001 and abs(wz) < 0.001:
            # Stopped
            steering = {
                'front_left': 0.0,
                'front_right': 0.0,
                'rear_left': 0.0,
                'rear_right': 0.0
            }
            velocities = {
                'front_left': 0.0,
                'front_right': 0.0,
                'rear_left': 0.0,
                'rear_right': 0.0
            }
        else:
            # Compute steering angle from Ackermann geometry
            if abs(wz) < 0.001:
                # Straight line
                steering_angle = 0.0
            else:
                # Turning radius
                R = abs(vx / wz) if abs(wz) > 0.001 else 1000.0

                # Basic Ackermann angle (simplified)
                steering_angle = math.atan(self.wheel_base / R)
                if wz < 0:
                    steering_angle = -steering_angle

            # Apply Ackermann steering geometry (inner wheel steers more)
            if abs(steering_angle) > 0.001:
                # Left turn
                if steering_angle > 0:
                    angle_inner = steering_angle * 1.1
                    angle_outer = steering_angle * 0.9
                    steering = {
                        'front_left': angle_inner,
                        'front_right': angle_outer,
                        'rear_left': 0.0,
                        'rear_right': 0.0
                    }
                # Right turn
                else:
                    angle_inner = steering_angle * 1.1
                    angle_outer = steering_angle * 0.9
                    steering = {
                        'front_left': angle_outer,
                        'front_right': angle_inner,
                        'rear_left': 0.0,
                        'rear_right': 0.0
                    }
            else:
                # Straight
                steering = {
                    'front_left': 0.0,
                    'front_right': 0.0,
                    'rear_left': 0.0,
                    'rear_right': 0.0
                }

            # Compute wheel velocities (differential for turning)
            base_vel = vx / self.wheel_radius

            if abs(wz) > 0.001:
                # Differential velocities for turning
                vel_diff = wz * self.track_width / (2.0 * self.wheel_radius)
                velocities = {
                    'front_left': base_vel - vel_diff,
                    'front_right': base_vel + vel_diff,
                    'rear_left': base_vel - vel_diff,
                    'rear_right': base_vel + vel_diff
                }
            else:
                # Straight line
                velocities = {
                    'front_left': base_vel,
                    'front_right': base_vel,
                    'rear_left': base_vel,
                    'rear_right': base_vel
                }

        # Clamp steering angles
        for key in steering:
            steering[key] = self.clamp(steering[key], -self.max_steering_angle, self.max_steering_angle)

        return steering, velocities

    def compute_crab(self, vx, vy):
        """
        Crab mode: Lateral movement, all wheels at same angle

        Args:
            vx: Linear velocity in x (forward/backward)
            vy: Linear velocity in y (lateral)

        Returns:
            steering: Dict of steering angles for each wheel
            velocities: Dict of wheel velocities
        """
        if abs(vx) < 0.001 and abs(vy) < 0.001:
            # Stopped
            steering = {
                'front_left': 0.0,
                'front_right': 0.0,
                'rear_left': 0.0,
                'rear_right': 0.0
            }
            velocities = {
                'front_left': 0.0,
                'front_right': 0.0,
                'rear_left': 0.0,
                'rear_right': 0.0
            }
        else:
            # Compute direction for crab motion
            angle = math.atan2(vy, vx)
            speed = math.sqrt(vx**2 + vy**2)

            # All wheels point in same direction (crab walk)
            steering = {
                'front_left': angle,
                'front_right': angle,
                'rear_left': angle,
                'rear_right': angle
            }

            # All wheels same velocity
            wheel_vel = speed / self.wheel_radius
            velocities = {
                'front_left': wheel_vel,
                'front_right': wheel_vel,
                'rear_left': wheel_vel,
                'rear_right': wheel_vel
            }

        # Clamp steering angles
        for key in steering:
            steering[key] = self.clamp(steering[key], -self.max_steering_angle, self.max_steering_angle)

        return steering, velocities

    def publish_commands(self, steering, velocities):
        """Publish steering and velocity commands to all wheels via ros2_control"""
        # Publish steering velocities (proportional control to reach target angle)
        for wheel, target_angle in steering.items():
            # Get current steering angle
            current_angle = self.current_steering[wheel]

            # Calculate error
            error = target_angle - current_angle

            # Calculate steering velocity (proportional control)
            # Higher gain = faster steering response
            steering_velocity = 5.0 * error  # Gain of 5.0 rad/s per radian of error

            # Clamp steering velocity
            steering_velocity = self.clamp(steering_velocity, -10.0, 10.0)

            # Publish as Float64MultiArray
            msg = Float64MultiArray()
            msg.data = [steering_velocity]
            self.steering_pubs[wheel].publish(msg)

        # Publish wheel velocities directly
        for wheel, velocity in velocities.items():
            # Clamp wheel velocity
            velocity = self.clamp(velocity, -100.0, 100.0)

            # Publish as Float64MultiArray
            msg = Float64MultiArray()
            msg.data = [velocity]
            self.wheel_pubs[wheel].publish(msg)

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
