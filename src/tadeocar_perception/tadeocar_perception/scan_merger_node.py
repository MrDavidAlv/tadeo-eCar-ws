#!/usr/bin/env python3
"""Two half-scans -> one 360 degree scan, with the robot itself taken out.

The TadeoeCar carries the same YDLIDAR X2 at each end of the deck, both 58 mm
above the tread plate, and each uses only the 180 degrees it faces: the other
half of either sweep looks across the deck, at the upper chassis's posts, the
other sensor and any load. Together they see everything ahead of the front
sensor and behind the rear one; the strip beside the robot between them is
seen by neither, and its bins here stay at infinity, which every consumer
reads as no return.

Everything downstream - SLAM Toolbox, AMCL, both Nav2 costmaps - takes one
LaserScan on /scan. This node builds that scan from /scan_front and
/scan_rear, so none of their configuration changes:

  * Points are moved into a virtual sensor frame, base_scan_merged, midway
    between the two sensors at their height.

  * The rear scan is motion-compensated. The two sensors spin unsynchronised
    at 7 Hz, so a rear scan can be up to 140 ms older than the front one it is
    merged with, which at 1 m/s is 14 cm of drive. Its points are transformed
    through odom at their own timestamp, not at the front scan's.

  * Anything inside the robot's own outline is discarded. With each sensor
    held to its own half nothing of the robot should reach this point, so
    this is the guard for a driver configured with the full sweep: the arc's
    posts, each sensor seeing the other and whatever is loaded on the deck
    all move with the robot, and scan matching treats a return that moves
    with the robot as the world moving the other way.

  * Where both sensors see the same direction the nearer return wins, which
    is the safe answer for every consumer.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _rotation(q):
    """Rotation matrix of a quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ScanMerger(Node):

    def __init__(self):
        super().__init__('scan_merger')
        self.declare_parameter('front_topic', '/scan_front')
        self.declare_parameter('rear_topic', '/scan_rear')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('output_frame', 'base_scan_merged')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('bins', 720)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('max_rear_age', 0.25)
        # The robot's own outline in base_link: the deck, 1.418 x 0.815 m,
        # spanning x [-0.757, 0.661], plus 3 cm all round for lamps, brackets
        # and the noise of a return off its own edge.
        self.declare_parameter('self_box', [-0.79, 0.69, -0.44, 0.44])

        def g(name):
            return self.get_parameter(name).value

        self.out_frame = g('output_frame')
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.bins = int(g('bins'))
        self.rmin, self.rmax = float(g('range_min')), float(g('range_max'))
        self.max_rear_age = float(g('max_rear_age'))
        self.box = [float(v) for v in g('self_box')]

        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        # Its own thread: a callback that waits for a transform on the same
        # executor that delivers transforms waits for nothing and times out,
        # which held the merged scan to under 2 Hz.
        self.tf_listener = TransformListener(self.tf, self, spin_thread=True)
        self.rear = None
        self.pub = self.create_publisher(LaserScan, g('output_topic'), 10)
        self.create_subscription(LaserScan, g('rear_topic'),
                                 lambda m: setattr(self, 'rear', m),
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('front_topic'), self.on_front,
                                 qos_profile_sensor_data)
        self.warned = set()
        self.get_logger().info(
            f'merging {g("front_topic")} + {g("rear_topic")} -> '
            f'{g("output_topic")} in {self.out_frame}, {self.bins} bins, '
            f'self box x [{self.box[0]}, {self.box[1]}] y [{self.box[2]}, {self.box[3]}]')

    def _warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def _points(self, scan, stamp_out, moving):
        """Scan endpoints expressed in the output frame at stamp_out."""
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        lo, hi = max(scan.range_min, self.rmin), min(scan.range_max, self.rmax)
        ok = np.isfinite(r) & (r >= lo) & (r <= hi)
        if not ok.any():
            return np.zeros((0, 3))
        p = np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())]
        # Through odom: the sensor at its own time, the output frame at the
        # front scan's. For the front scan the two times are the same and the
        # chain reduces to the static mount.
        #
        # Neither time may be later than the newest odometry. A scan is
        # stamped when it was taken and odom -> base_footprint arrives at
        # 50 Hz, so the front scan is usually a few milliseconds ahead of the
        # newest transform. Waiting for the next one held this callback up to
        # its timeout, and on a simulation running slower than real time the
        # timeout won every time: the rear scan was dropped from every merged
        # scan. Using the newest transform instead costs at most one
        # odometry period of motion, 2 cm at 1 m/s.
        if moving:
            newest = Time.from_msg(self.tf.lookup_transform(
                self.odom_frame, self.base_frame, Time()).header.stamp)
            t = self.tf.lookup_transform_full(
                self.out_frame, min(stamp_out, newest), scan.header.frame_id,
                min(Time.from_msg(scan.header.stamp), newest), self.odom_frame)
        else:
            t = self.tf.lookup_transform(self.out_frame, scan.header.frame_id, Time())
        R = _rotation(t.transform.rotation)
        T = np.array([t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z])
        return p @ R.T + T

    def on_front(self, front):
        stamp = Time.from_msg(front.header.stamp)
        clouds = []
        try:
            clouds.append(self._points(front, stamp, moving=False))
        except TransformException as exc:
            self._warn_once('front', f'front scan not transformable yet: {exc}')
            return
        rear = self.rear
        if rear is not None:
            age = (stamp - Time.from_msg(rear.header.stamp)).nanoseconds * 1e-9
            if abs(age) <= self.max_rear_age:
                try:
                    clouds.append(self._points(rear, stamp, moving=True))
                except TransformException as exc:
                    self._warn_once('rear', f'rear scan not transformable yet: {exc}')
            else:
                self._warn_once('age', f'rear scan {age:.2f} s off the front one; '
                                       'publishing the front alone until they line up')
        pts = np.concatenate(clouds) if clouds else np.zeros((0, 3))

        # Drop the robot itself. The box is in base_link; the output frame is
        # a pure translation of it, so shift the box rather than the points.
        try:
            off = self.tf.lookup_transform(self.base_frame, self.out_frame, Time())
            ox, oy = off.transform.translation.x, off.transform.translation.y
        except TransformException:
            ox = oy = 0.0
        bx, by = pts[:, 0] + ox, pts[:, 1] + oy
        own = (bx > self.box[0]) & (bx < self.box[1]) & (by > self.box[2]) & (by < self.box[3])
        pts = pts[~own]

        out = LaserScan()
        out.header.stamp = front.header.stamp
        out.header.frame_id = self.out_frame
        out.angle_min = -math.pi
        out.angle_increment = 2 * math.pi / self.bins
        out.angle_max = math.pi - out.angle_increment
        out.time_increment = 0.0
        out.scan_time = front.scan_time
        out.range_min = self.rmin
        out.range_max = self.rmax
        ranges = np.full(self.bins, np.inf)
        if len(pts):
            r = np.hypot(pts[:, 0], pts[:, 1])
            bearing = np.arctan2(pts[:, 1], pts[:, 0]) + math.pi
            idx = (bearing / out.angle_increment).astype(int) % self.bins
            keep = (r >= self.rmin) & (r <= self.rmax)
            np.minimum.at(ranges, idx[keep], r[keep])
        out.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
