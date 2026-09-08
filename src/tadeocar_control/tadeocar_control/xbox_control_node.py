#!/usr/bin/env python3
"""Xbox pad teleoperation for the 4WD4WS platform.

    Left stick        body translation: up/down is vx, left/right is vy
    Right stick, X    yaw rate, wz
    RT                throttle, analogue: nothing moves until it is pulled
    LT or LB          brake, commands zero immediately
    A / B / X         omnidirectional / Ackermann / crab

Published on /cmd_vel_joy, which twist_mux gives the highest priority of the
four inputs, so the pad overrides the web interface and Nav2 while it is being
held. The mode goes out on /robot_mode, which fourws_kinematics_node consumes.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# Linux xpad axis order. The right stick is 3 and 4, NOT 2 and 3: axis 2 is the
# left trigger, sitting between the two sticks.
AX_LEFT_X, AX_LEFT_Y = 0, 1
AX_LT = 2
AX_RIGHT_X, AX_RIGHT_Y = 3, 4
AX_RT = 5

BTN_A, BTN_B, BTN_X = 0, 1, 2
BTN_LB, BTN_RB = 4, 5

MODE_BUTTONS = ((BTN_A, 'omnidirectional'), (BTN_B, 'ackermann'),
                (BTN_X, 'crab'))


class XboxControlNode(Node):
    """Maps an Xbox pad to body twists, within the robot's own limits."""

    def __init__(self):
        super().__init__('xbox_control_node')

        # The same ceilings fourws_kinematics_node clamps to. They used to be
        # hardcoded at 2.0 m/s here while the robot clamps at 1.0, so the stick
        # reached full speed at half its travel and the outer half did nothing.
        # This is the bug the web interface had, fixed there and missed here.
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('deadzone', 0.1)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value

        self.current_mode = 'omnidirectional'
        self.last_buttons = {}

        # An untouched trigger axis reads exactly 0.0, while a released one
        # reads +1.0; the driver only starts reporting the real range once the
        # trigger has moved for the first time. Scaling 0.0 as if it were a
        # position would hand out half throttle before the pad is touched, so
        # the analogue reading is ignored until it has proved itself live, and
        # RB stands in as a digital throttle until then.
        self.trigger_live = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info(
            f'xbox pad up: {self.max_linear_speed:.2f} m/s, '
            f'{self.max_angular_speed:.2f} rad/s, deadzone {self.deadzone:.2f}. '
            'RT throttles, LT or LB brakes, A/B/X pick the mode.')

    # ------------------------------------------------------------------ input

    @staticmethod
    def axis(msg, index):
        return msg.axes[index] if len(msg.axes) > index else 0.0

    @staticmethod
    def button(msg, index):
        return msg.buttons[index] if len(msg.buttons) > index else 0

    def apply_deadzone(self, value):
        """Zero the centre, and rescale so the live range still reaches 1."""
        if abs(value) < self.deadzone:
            return 0.0
        span = 1.0 - self.deadzone
        return (value - math.copysign(self.deadzone, value)) / span

    def pressed(self, msg, index):
        """True on the transition into a press, not while it is held."""
        now = self.button(msg, index)
        was = self.last_buttons.get(index, 0)
        self.last_buttons[index] = now
        return now == 1 and was == 0

    def throttle_of(self, msg):
        """0 to 1 from RT, falling back to RB before the trigger is first used."""
        rt = self.axis(msg, AX_RT)
        if rt != 0.0:
            self.trigger_live = True
        if self.trigger_live:
            return max(0.0, min(1.0, (1.0 - rt) / 2.0))
        return 1.0 if self.button(msg, BTN_RB) else 0.0

    def braking(self, msg):
        lt = self.axis(msg, AX_LT)
        return self.button(msg, BTN_LB) == 1 or (lt != 0.0 and lt < 0.5)

    # --------------------------------------------------------------- callback

    def joy_callback(self, msg):
        # The mode is chosen with the face buttons and nothing else. It used to
        # be forced by whichever stick was moved, which left Ackermann
        # unreachable: pressing B selected it and the next touch of a stick
        # took it straight back out again.
        for index, mode in MODE_BUTTONS:
            if self.pressed(msg, index):
                self.switch_mode(mode)

        twist = Twist()
        throttle = self.throttle_of(msg)

        if not self.braking(msg) and throttle > 0.0:
            vx = self.apply_deadzone(self.axis(msg, AX_LEFT_Y))
            vy = self.apply_deadzone(self.axis(msg, AX_LEFT_X))
            wz = self.apply_deadzone(self.axis(msg, AX_RIGHT_X))

            # Held on the diagonal, the two axes would otherwise ask for
            # sqrt(2) times the top speed and the kinematics would clamp each
            # one separately, bending the commanded direction.
            magnitude = math.hypot(vx, vy)
            if magnitude > 1.0:
                vx, vy = vx / magnitude, vy / magnitude

            twist.linear.x = vx * self.max_linear_speed * throttle
            twist.linear.y = vy * self.max_linear_speed * throttle
            # Yaw was published as a constant zero, so the pad could translate
            # and never turn: Ackermann steers from wz and had nothing to steer
            # with, and omnidirectional could not rotate.
            twist.angular.z = wz * self.max_angular_speed * throttle

        self.cmd_vel_pub.publish(twist)

    def switch_mode(self, new_mode):
        if new_mode == self.current_mode:
            return
        self.current_mode = new_mode
        self.mode_pub.publish(String(data=new_mode))
        self.get_logger().info(f'mode: {new_mode}')


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
