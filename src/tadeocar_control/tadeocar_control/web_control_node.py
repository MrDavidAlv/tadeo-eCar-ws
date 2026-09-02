#!/usr/bin/env python3
"""
Web Control Node for TadeoeCar.
WebSocket server that receives commands from a web interface and publishes
Twist messages on /cmd_vel and mode on /robot_mode.

The actual kinematics (cmd_pos / cmd_vel to joints) is handled by
fourws_kinematics_node. This node only translates web inputs to cmd_vel + mode.
"""

import asyncio
import json
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import websockets
except ImportError:
    websockets = None


class TadeoCarWebControl(Node):

    def __init__(self):
        super().__init__('tadeocar_web_control')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_web', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        self.connected_clients = set()
        self._loop = None

        self.get_logger().info('TadeoeCar Web Control Node started (cmd_vel mode)')

    def _publish_mode(self, mode_name):
        """Publish robot mode."""
        msg = String()
        msg.data = mode_name
        self.mode_pub.publish(msg)

    def _publish_twist(self, vx=0.0, vy=0.0, wz=0.0):
        """Publish Twist on /cmd_vel."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    # The robot's own limits, from tadeocar_control/config/robot_params.yaml.
    # The web interface used to scale its inputs to 3.0 m/s and 3.0 rad/s
    # against a robot that clamps at 1.0 of each, so a joystick was saturated
    # at a third of its travel and the speed slider did nothing above 34 %.
    MAX_LINEAR = 1.0
    MAX_ANGULAR = 1.0

    def process_command(self, command):
        try:
            mode = command.get('mode', '')
            speed_factor = command.get('speed', 100.0) / 100.0

            if mode == 'omnidirectional':
                x = command.get('x', 0.0)
                y = command.get('y', 0.0)
                vel = math.hypot(x, y) * speed_factor * self.MAX_LINEAR
                angle = math.atan2(y, x) if (x != 0 or y != 0) else 0.0
                vx = vel * math.cos(angle)
                vy = vel * math.sin(angle)
                self._publish_mode('omnidirectional')
                self._publish_twist(vx=vx, vy=vy)

            elif mode == 'ackermann':
                steering = command.get('steering', 0.0)
                throttle = command.get('throttle', 0.0)
                vx = throttle * speed_factor * self.MAX_LINEAR
                # Convert steering input to angular velocity
                # steering is normalized [-1, 1], scale to max angular speed
                wz = steering * speed_factor * self.MAX_ANGULAR
                self._publish_mode('ackermann')
                self._publish_twist(vx=vx, wz=wz)

            elif mode == 'halo':
                global_angle = command.get('globalAngle', 0.0)
                speed = command.get('speed', 0.0)
                angle_rad = math.radians(global_angle)
                vel = speed * speed_factor * self.MAX_LINEAR
                vx = vel * math.cos(angle_rad)
                vy = vel * math.sin(angle_rad)
                self._publish_mode('crab')
                self._publish_twist(vx=vx, vy=vy)

            elif mode == 'spin':
                spin_speed = command.get('spinSpeed', 0.0)
                wz = spin_speed * speed_factor * self.MAX_ANGULAR
                self._publish_mode('omnidirectional')
                self._publish_twist(wz=wz)

            elif mode == 'stop':
                self._publish_twist()

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    async def websocket_handler(self, websocket):
        self.connected_clients.add(websocket)
        self.get_logger().info(
            f'Client connected. Total: {len(self.connected_clients)}'
        )
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.process_command(data)
                except json.JSONDecodeError:
                    self.get_logger().error('Invalid JSON received')
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connected_clients.discard(websocket)
            self.get_logger().info(
                f'Client disconnected. Total: {len(self.connected_clients)}'
            )

    async def start_websocket_server(self):
        server = await websockets.serve(self.websocket_handler, '0.0.0.0', 8765)
        self.get_logger().info('WebSocket server on ws://0.0.0.0:8765')
        await server.wait_closed()


def main(args=None):
    if websockets is None:
        print('ERROR: websockets package not installed. Run: pip3 install websockets')
        return

    rclpy.init(args=args)
    node = TadeoCarWebControl()

    loop = asyncio.new_event_loop()
    node._loop = loop

    def run_asyncio_loop():
        asyncio.set_event_loop(loop)
        loop.run_until_complete(node.start_websocket_server())

    ws_thread = threading.Thread(target=run_asyncio_loop, daemon=True)
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if loop.is_running():
            loop.call_soon_threadsafe(loop.stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
