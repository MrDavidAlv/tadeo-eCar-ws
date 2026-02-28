#!/usr/bin/env python3
"""
Web Control Node for TadeoeCar.
WebSocket server that receives commands from a web interface and publishes
steering positions and wheel velocities via ros_gz_bridge.
"""

import asyncio
import json
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

try:
    import websockets
except ImportError:
    websockets = None


WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']


class TadeoCarWebControl(Node):

    def __init__(self):
        super().__init__('tadeocar_web_control')

        # Steering publishers (position control)
        self.steering_pubs = {
            w: self.create_publisher(
                Float64,
                f'/model/tadeocar/joint/{w}_steering_joint/cmd_pos',
                10,
            )
            for w in WHEELS
        }

        # Wheel publishers (velocity control)
        self.wheel_pubs = {
            w: self.create_publisher(
                Float64,
                f'/model/tadeocar/joint/{w}_wheel_joint/cmd_vel',
                10,
            )
            for w in WHEELS
        }

        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.current_joint_states = {}
        self.connected_clients = set()
        self._loop = None

        self.get_logger().info('TadeoeCar Web Control Node started')

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]

    def _publish_steering(self, angles):
        """Publish steering position commands (radians)."""
        for w, angle in zip(WHEELS, angles):
            msg = Float64()
            msg.data = float(angle)
            self.steering_pubs[w].publish(msg)

    def _publish_wheels(self, velocities):
        """Publish wheel velocity commands (rad/s)."""
        for w, vel in zip(WHEELS, velocities):
            msg = Float64()
            msg.data = float(vel)
            self.wheel_pubs[w].publish(msg)

    def process_command(self, command):
        try:
            mode = command.get('mode', '')
            speed_factor = command.get('speed', 100.0) / 100.0

            if mode == 'omnidirectional':
                x = command.get('x', 0.0)
                y = command.get('y', 0.0)
                angle = math.atan2(y, x) if (x != 0 or y != 0) else 0.0
                vel = math.hypot(x, y) * speed_factor * 3.0
                self._publish_steering([angle] * 4)
                self._publish_wheels([vel] * 4)

            elif mode == 'ackermann':
                steering = command.get('steering', 0.0)
                throttle = command.get('throttle', 0.0)
                front_angle = steering * 0.5 * speed_factor
                vel = throttle * speed_factor * 3.0
                self._publish_steering([front_angle, front_angle, 0.0, 0.0])
                self._publish_wheels([vel] * 4)

            elif mode == 'halo':
                global_angle = command.get('globalAngle', 0.0)
                speed = command.get('speed', 0.0)
                angle_rad = math.radians(global_angle)
                vel = speed * speed_factor * 3.0
                self._publish_steering([angle_rad] * 4)
                self._publish_wheels([vel] * 4)

            elif mode == 'spin':
                spin_speed = command.get('spinSpeed', 0.0)
                vel = spin_speed * speed_factor * 3.0
                self._publish_steering([0.785, -0.785, -0.785, 0.785])
                self._publish_wheels([vel, -vel, vel, -vel])

            elif mode == 'stop':
                self._publish_wheels([0.0] * 4)

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
