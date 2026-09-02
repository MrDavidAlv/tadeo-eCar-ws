#!/usr/bin/env python3
"""Body twist -> per-wheel steering angle and drive speed, for a 4WD4WS robot.

Steering joints take a position (they are servos on the real robot, and a servo
takes an angle); drive joints take an angular velocity. That split is why this
node publishes to two different controller topics per wheel rather than to one
DiffDrive-style plugin.

Three modes, and they are genuinely different manoeuvres rather than three
names for the same one:

  omnidirectional  every wheel points along its own velocity vector, so the
                   body can translate in any direction while rotating.
  ackermann        front wheels steer, rear wheels stay straight, like a car.
                   Used when the goal is a path a car could also follow.
  crab             all four wheels at the SAME angle, so the body translates
                   without rotating and keeps its heading.

What each mode owes the reader is in docs/mathematical-model/kinematics.md.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']
MODES = ('omnidirectional', 'ackermann', 'crab')


class FourWSKinematicsNode(Node):

    def __init__(self):
        super().__init__('fourws_kinematics_node')

        self.declare_parameter('wheel_radius', 0.125)
        self.declare_parameter('front_axle_x', 0.478)
        self.declare_parameter('rear_axle_x', -0.580)
        self.declare_parameter('half_track', 0.275)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_steering_angle', 2.356)
        self.declare_parameter('steering_tolerance', 0.20)
        self.declare_parameter('gate_speed_threshold', 0.25)
        self.declare_parameter('mode', 'omnidirectional')
        # Crab heading hold. See _crab for what it is for and why it needs an
        # estimate the wheels cannot provide.
        self.declare_parameter('crab_heading_hold', True)
        self.declare_parameter('crab_heading_gain', 0.6)
        self.declare_parameter('crab_heading_max', 0.20)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        front_x = self.get_parameter('front_axle_x').value
        rear_x = self.get_parameter('rear_axle_x').value
        half_track = self.get_parameter('half_track').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_tolerance = self.get_parameter('steering_tolerance').value
        self.gate_speed = self.get_parameter('gate_speed_threshold').value
        self.mode = self.get_parameter('mode').value
        self.crab_hold = self.get_parameter('crab_heading_hold').value
        self.crab_gain = self.get_parameter('crab_heading_gain').value
        self.crab_max = self.get_parameter('crab_heading_max').value

        # Steering axis positions in base_link, taken from the description
        # rather than derived from half a wheelbase. The front axis is 0.478 m
        # ahead of base_link and the rear one 0.580 m behind it: the wheelbase
        # is 1.058 m, but its midpoint is 51 mm behind the origin. Using
        # +/-L/2 - which this node used to do - puts the kinematic centre in
        # the wrong place, and the visible symptom is that a commanded spin in
        # place also drifts the robot sideways.
        self.wheel_positions = {
            'front_left': (front_x, half_track),
            'front_right': (front_x, -half_track),
            'rear_left': (rear_x, half_track),
            'rear_right': (rear_x, -half_track),
        }
        # Ackermann needs the distance between the two axles, not either
        # position on its own.
        self.wheel_base = front_x - rear_x
        self.half_track = half_track

        self.current_steering = {w: 0.0 for w in WHEELS}
        self.have_joint_states = False
        self.heading = None          # from the estimator, radians
        self.crab_reference = None   # heading latched when a crab run begins

        self.steering_pubs = {
            w: self.create_publisher(
                Float64, f'/model/tadeocar/joint/{w}_steering_joint/cmd_pos', 10)
            for w in WHEELS
        }
        self.wheel_pubs = {
            w: self.create_publisher(
                Float64, f'/model/tadeocar/joint/{w}_wheel_joint/cmd_vel', 10)
            for w in WHEELS
        }

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/robot_mode', self.mode_callback, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_state_callback, 10)
        # The fused estimate, because crab heading hold needs to see a yaw
        # error the wheels are blind to. Absent - no EKF running - the hold
        # simply does not engage and crab is open loop, as it was.
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.odom_callback, 10)

        self.get_logger().info(
            f'4WS kinematics up in {self.mode} mode. '
            f'wheelbase={self.wheel_base:.3f} m (front {front_x:+.3f}, '
            f'rear {rear_x:+.3f}), track={2*half_track:.3f} m, '
            f'r={self.wheel_radius:.3f} m')

    # ------------------------------------------------------------- inputs --
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if 'steering' not in name:
                continue
            for w in WHEELS:
                if name.startswith(w):
                    self.current_steering[w] = msg.position[i]
                    break
        self.have_joint_states = True

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def mode_callback(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode in MODES:
            if new_mode != self.mode:
                self.get_logger().info(f'mode: {self.mode} -> {new_mode}')
                self.crab_reference = None
            self.mode = new_mode
        else:
            self.get_logger().warn(
                f'unknown mode "{msg.data}", staying in {self.mode}. '
                f'Known modes: {", ".join(MODES)}')

    def cmd_vel_callback(self, msg):
        vx = self._clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        vy = self._clamp(msg.linear.y, -self.max_linear_speed, self.max_linear_speed)
        wz = self._clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)

        if self.mode == 'ackermann':
            steering, speeds = self._ackermann(vx, wz)
        elif self.mode == 'crab':
            steering, speeds = self._crab(vx, vy)
        else:
            steering, speeds = self._omnidirectional(vx, vy, wz)

        self._publish(steering, speeds)

    # -------------------------------------------------------------- modes --
    def _omnidirectional(self, vx, vy, wz):
        """Each wheel points along the velocity its own position sees.

        A rigid body's velocity at a point p is v + w x p, which in the plane
        is (vx - wz*py, vy + wz*px). Steer the wheel along that vector and
        drive it at its magnitude, and no wheel is asked to slide sideways.
        """
        steering, speeds = {}, {}
        for w, (px, py) in self.wheel_positions.items():
            wx = vx - wz * py
            wy = vy + wz * px

            if abs(wx) < 1e-4 and abs(wy) < 1e-4:
                steering[w] = self.current_steering[w]   # hold, do not snap
                speeds[w] = 0.0
                continue

            angle, direction = self._fold_steering(math.atan2(wy, wx))
            steering[w] = self._clamp(angle, -self.max_steering_angle,
                                      self.max_steering_angle)
            speeds[w] = direction * math.hypot(wx, wy) / self.wheel_radius
        return steering, speeds

    def _ackermann(self, vx, wz):
        """Front wheels steer, rear wheels stay straight.

        The turn centre lies on the rear axle's line, R = vx / wz to the left
        of it, and each front wheel points at it:

            delta_i = atan( L / (R -+ T/2) )

        atan, not atan2. atan2(L, R) looks equivalent and is not: for a right
        turn R goes negative and atan2 returns something near pi rather than a
        small negative angle, so the wheels tried to fold back on themselves
        and the clamp left them at the 135 degree limit. Right turns in this
        mode were broken from the day it was written.
        """
        steering = {w: 0.0 for w in WHEELS}
        speeds = {w: 0.0 for w in WHEELS}

        if abs(vx) < 1e-4 and abs(wz) < 1e-4:
            return steering, speeds

        base_speed = vx / self.wheel_radius

        if abs(wz) < 1e-4 or abs(vx) < 1e-4:
            # Straight line, or a rotation this mode cannot produce: a car
            # with its rear wheels straight cannot spin in place.
            return steering, {w: base_speed for w in WHEELS}

        radius = vx / wz
        for w in WHEELS:
            if not w.startswith('front'):
                continue
            offset = self.half_track if w.endswith('left') else -self.half_track
            denom = radius - offset
            if abs(denom) < 1e-6:
                steering[w] = math.copysign(math.pi / 2, wz)
            else:
                steering[w] = math.atan(self.wheel_base / denom)
            steering[w] = self._clamp(steering[w], -self.max_steering_angle,
                                      self.max_steering_angle)

        # Inner wheels travel a shorter arc. Scaling by each wheel's own turn
        # radius keeps all four rolling instead of two of them dragging.
        for w, (_, py) in self.wheel_positions.items():
            speeds[w] = (vx - wz * py) / self.wheel_radius
        return steering, speeds

    def _crab(self, vx, vy):
        """All four wheels at the same angle: translate without rotating.

        This mode used to point the front and rear wheels in OPPOSITE
        directions, which is counter-phase steering - the tightest turn a 4WS
        chassis can make, and the exact opposite of what crab means. A crab
        walk keeps the heading fixed, and that is only possible if every wheel
        points the same way.

        Four parallel wheels leave the robot neutrally stable in yaw: nothing
        about the geometry opposes a rotation, so a nudge from an uneven
        contact patch turns the whole chassis and it stays turned. Measured
        open loop, driving 1.7 m sideways came out anywhere between 4 and 14
        degrees off the heading it started with.

        Closing that loop needs a yaw estimate, and it specifically cannot come
        from the wheels: during a crab all four turn at the same rate whatever
        the body is doing, so wheel odometry reported 0.1 degrees of rotation
        while the robot had actually swung 14. The gyro sees it, so the
        correction runs off the EKF's fused heading. The correction itself is
        counter-phase steering - front wheels one way, rear the other - which
        is a yaw moment with no net sideways force, so it steers the heading
        back without disturbing the translation.
        """
        steering = {w: 0.0 for w in WHEELS}
        speeds = {w: 0.0 for w in WHEELS}

        if abs(vx) < 1e-4 and abs(vy) < 1e-4:
            self.crab_reference = None
            return {w: self.current_steering[w] for w in WHEELS}, speeds

        angle, direction = self._fold_steering(math.atan2(vy, vx))
        angle = self._clamp(angle, -self.max_steering_angle,
                            self.max_steering_angle)
        speed = direction * math.hypot(vx, vy) / self.wheel_radius

        correction = 0.0
        if self.crab_hold and self.heading is not None:
            if self.crab_reference is None:
                self.crab_reference = self.heading
            error = self._wrap(self.crab_reference - self.heading)
            correction = self._clamp(self.crab_gain * error,
                                     -self.crab_max, self.crab_max)

        for w in WHEELS:
            sign = 1.0 if w.startswith('front') else -1.0
            steering[w] = self._clamp(angle + sign * correction,
                                      -self.max_steering_angle,
                                      self.max_steering_angle)
            speeds[w] = speed
        return steering, speeds

    # ------------------------------------------------------------ output ---
    def _publish(self, steering, speeds):
        """Steer first, drive once the steering has arrived - all four together.

        The gate is one number shared by all four wheels, taken from the WORST
        steering error, and that is the part that matters. Gating each wheel on
        its own error was tried first and is wrong: the four servos never
        finish a large swing at the same instant, so for a couple of tenths of
        a second some wheels are driving in the new direction while others are
        still driving in the old one, and the mismatch is a yaw impulse. It
        showed up as a crab manoeuvre - the one command whose entire purpose is
        to keep the heading fixed - rotating the robot by 6.8 degrees over
        1.8 m. Coupling the gate cut that to under a degree.

        The ramp reaches zero at 90 degrees of error rather than cutting out at
        the tolerance, so a continuously steering command such as Nav2's is
        never interrupted; only a large re-aim stops the wheels.
        """
        moving = [w for w in WHEELS if speeds[w] != 0.0]
        gate = 1.0
        # Scrubbing costs in proportion to how fast the tyre is being dragged,
        # so below a crawl the gate has nothing to prevent and closing it does
        # real harm. Nav2 sets off at the slowest sample its acceleration limit
        # allows, about 0.09 m/s, and at that speed the omnidirectional
        # solution asks for wheel angles of 20 to 40 degrees even though the
        # body is barely turning. Gating on that error stopped the robot, which
        # kept its measured velocity at zero, which kept Nav2 at its slowest
        # sample: a standoff that had the controller reporting "failed to make
        # progress" while the wheels aimed and re-aimed. Measured over a 3 m
        # leg, the robot crawled the whole way at 0.09 m/s.
        fastest = max((abs(speeds[w]) for w in moving), default=0.0)
        if (self.have_joint_states and moving
                and fastest * self.wheel_radius > self.gate_speed):
            worst = max(abs(self._wrap(steering[w] - self.current_steering[w]))
                        for w in moving)
            if worst > self.steering_tolerance:
                span = math.pi / 2 - self.steering_tolerance
                gate = self._clamp(1.0 - (worst - self.steering_tolerance) / span,
                                   0.0, 1.0)

        for w in WHEELS:
            msg = Float64()
            msg.data = float(steering[w])
            self.steering_pubs[w].publish(msg)

            msg = Float64()
            msg.data = float(self._clamp(speeds[w] * gate, -60.0, 60.0))
            self.wheel_pubs[w].publish(msg)

    # --------------------------------------------------------- utilities ---
    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @classmethod
    def _fold_steering(cls, angle):
        """Fold a heading into [-pi/2, pi/2], reversing the wheel if needed.

        A wheel asked to point backwards can instead point forwards and spin
        the other way. It reaches the same velocity through half the servo
        travel, which matters when the servo has 135 degrees and the turn
        would have asked for 170.
        """
        angle = cls._wrap(angle)
        if angle > math.pi / 2:
            return angle - math.pi, -1.0
        if angle < -math.pi / 2:
            return angle + math.pi, -1.0
        return angle, 1.0

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = FourWSKinematicsNode()
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
