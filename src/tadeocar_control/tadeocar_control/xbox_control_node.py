#!/usr/bin/env python3
"""
Xbox Controller Node for TadeoeCar 4WD4WS
Dual joystick control:
- Left joystick → Omnidirectional mode
- Right joystick → Crab mode
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class XboxControlNode(Node):
    """
    Xbox controller for dual joystick 4WS control

    Xbox Controller Layout (standard mapping):
    Axes:
      0: Left stick horizontal (left=-1, right=+1)
      1: Left stick vertical (up=+1, down=-1)
      2: Right stick horizontal (left=-1, right=+1)
      3: Right stick vertical (up=+1, down=-1)
      4: Left trigger (unpressed=1, pressed=-1)
      5: Right trigger (unpressed=1, pressed=-1)

    Buttons:
      0: A
      1: B
      2: X
      3: Y
      4: LB (Left Bumper)
      5: RB (Right Bumper)
      6: Back
      7: Start
      8: Xbox button
      9: Left stick press
      10: Right stick press
    """

    def __init__(self):
        super().__init__('xbox_control_node')

        # Control parameters
        self.max_linear_speed = 2.0  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.deadzone = 0.1  # Ignore small joystick movements

        # Current mode
        self.current_mode = 'omnidirectional'

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Mode switch button state (to detect press, not hold)
        self.last_mode_button_state = 0

        self.get_logger().info('Xbox Control Node initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left joystick → Omnidirectional movement')
        self.get_logger().info('  Right joystick → Crab movement')
        self.get_logger().info('  A button → Toggle to Omnidirectional mode')
        self.get_logger().info('  B button → Toggle to Ackermann mode')
        self.get_logger().info('  X button → Toggle to Crab mode')

    def joy_callback(self, msg):
        """Process joystick input and publish velocity commands"""

        # Check if we have enough axes and buttons (need at least 5 axes for Xbox 360)
        if len(msg.axes) < 5 or len(msg.buttons) < 3:
            self.get_logger().warn('Invalid joy message format')
            return

        # Read joystick axes (Xbox 360 Controller mapping)
        left_horizontal = msg.axes[0]   # Left stick X
        left_vertical = msg.axes[1]     # Left stick Y
        right_horizontal = msg.axes[3]  # Right stick X (was axes[2] = LT trigger!)
        right_vertical = msg.axes[4]    # Right stick Y (was axes[3] = Right stick X!)

        # Apply deadzone
        left_horizontal = self.apply_deadzone(left_horizontal)
        left_vertical = self.apply_deadzone(left_vertical)
        right_horizontal = self.apply_deadzone(right_horizontal)
        right_vertical = self.apply_deadzone(right_vertical)

        # Mode selection buttons
        button_a = msg.buttons[0] if len(msg.buttons) > 0 else 0  # Omnidirectional
        button_b = msg.buttons[1] if len(msg.buttons) > 1 else 0  # Ackermann
        button_x = msg.buttons[2] if len(msg.buttons) > 2 else 0  # Crab

        # Handle mode switching (on button press, not hold)
        if button_a == 1 and self.last_mode_button_state == 0:
            self.switch_mode('omnidirectional')
        elif button_b == 1 and self.last_mode_button_state == 0:
            self.switch_mode('ackermann')
        elif button_x == 1 and self.last_mode_button_state == 0:
            self.switch_mode('crab')

        self.last_mode_button_state = button_a or button_b or button_x

        # Determine which joystick is being used
        left_magnitude = math.sqrt(left_horizontal**2 + left_vertical**2)
        right_magnitude = math.sqrt(right_horizontal**2 + right_vertical**2)

        # Create Twist message
        twist = Twist()

        # Priority: if both joysticks are active, left takes priority
        if left_magnitude > 0.0:
            # Left joystick → Omnidirectional mode
            if self.current_mode != 'omnidirectional':
                self.switch_mode('omnidirectional')

            # Map to velocity
            twist.linear.x = left_vertical * self.max_linear_speed
            twist.linear.y = left_horizontal * self.max_linear_speed
            twist.angular.z = 0.0

        elif right_magnitude > 0.0:
            # Right joystick → Crab mode
            if self.current_mode != 'crab':
                self.switch_mode('crab')

            # Map to velocity
            twist.linear.x = right_vertical * self.max_linear_speed
            twist.linear.y = right_horizontal * self.max_linear_speed
            twist.angular.z = 0.0

        else:
            # No joystick input → stop
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

    def switch_mode(self, new_mode):
        """Switch control mode and publish to /robot_mode"""
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            mode_msg = String()
            mode_msg.data = new_mode
            self.mode_pub.publish(mode_msg)
            self.get_logger().info(f'Mode switched to: {new_mode}')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        else:
            # Scale to remove deadzone discontinuity
            if value > 0:
                return (value - self.deadzone) / (1.0 - self.deadzone)
            else:
                return (value + self.deadzone) / (1.0 - self.deadzone)


def main(args=None):
    rclpy.init(args=args)
    node = XboxControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
