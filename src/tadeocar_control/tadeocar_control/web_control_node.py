#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import asyncio
import websockets
import json
import threading
import math

class TadeoCarWebControl(Node):
    def __init__(self):
        super().__init__('tadeocar_web_control')

        # Publishers for direct joint effort control
        self.fl_steering_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/front_left_steering_joint/cmd_effort', 10)
        self.fl_wheel_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/front_left_wheel_joint/cmd_effort', 10)
        self.fr_steering_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/front_right_steering_joint/cmd_effort', 10)
        self.fr_wheel_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/front_right_wheel_joint/cmd_effort', 10)
        self.rl_steering_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/rear_left_steering_joint/cmd_effort', 10)
        self.rl_wheel_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/rear_left_wheel_joint/cmd_effort', 10)
        self.rr_steering_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/rear_right_steering_joint/cmd_effort', 10)
        self.rr_wheel_pub = self.create_publisher(Float64, '/model/tadeocar_v1/joint/rear_right_wheel_joint/cmd_effort', 10)

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.current_joint_states = {}
        self.connected_clients = set()

        self.get_logger().info('TadeoeCar Web Control Node Started')

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]

    def set_steering_effort(self, fl, fr, rl, rr):
        """Set steering efforts to reach target angles"""
        # PID-like control: calculate effort based on error
        target_angles = {
            'front_left_steering_joint': fl,
            'front_right_steering_joint': fr,
            'rear_left_steering_joint': rl,
            'rear_right_steering_joint': rr
        }

        kp = 20.0  # Proportional gain for effort control

        for joint_name, target in target_angles.items():
            current = self.current_joint_states.get(joint_name, 0.0)
            error = target - current
            effort = kp * error

            # Clamp effort
            effort = max(-50.0, min(50.0, effort))

            if 'front_left' in joint_name:
                self.publish_effort(self.fl_steering_pub, effort)
            elif 'front_right' in joint_name:
                self.publish_effort(self.fr_steering_pub, effort)
            elif 'rear_left' in joint_name:
                self.publish_effort(self.rl_steering_pub, effort)
            elif 'rear_right' in joint_name:
                self.publish_effort(self.rr_steering_pub, effort)

    def set_wheel_effort(self, fl, fr, rl, rr):
        """Set wheel efforts (torque)"""
        self.publish_effort(self.fl_wheel_pub, fl)
        self.publish_effort(self.fr_wheel_pub, fr)
        self.publish_effort(self.rl_wheel_pub, rl)
        self.publish_effort(self.rr_wheel_pub, rr)

    def publish_effort(self, publisher, effort):
        """Publish effort command"""
        msg = Float64()
        msg.data = float(effort)
        publisher.publish(msg)

    def process_command(self, command):
        """Process command from web interface"""
        try:
            mode = command.get('mode', '')
            speed_factor = command.get('speed', 100.0) / 100.0

            if mode == 'omnidirectional':
                x = command.get('x', 0.0)
                y = command.get('y', 0.0)

                # Omnidirectional: all wheels point in direction of motion
                angle = math.atan2(y, x) if (x != 0 or y != 0) else 0.0
                effort = math.sqrt(x*x + y*y) * speed_factor * 3.0

                self.set_steering_effort(angle, angle, angle, angle)
                self.set_wheel_effort(effort, effort, effort, effort)

            elif mode == 'ackermann':
                steering = command.get('steering', 0.0)
                throttle = command.get('throttle', 0.0)

                # Ackermann: front wheels steer, rear wheels straight
                front_angle = steering * 0.5 * speed_factor
                effort = throttle * speed_factor * 3.0

                self.set_steering_effort(front_angle, front_angle, 0.0, 0.0)
                self.set_wheel_effort(effort, effort, effort, effort)

            elif mode == 'halo':
                global_angle = command.get('globalAngle', 0.0)
                speed = command.get('speed', 0.0)

                # Halo: all wheels point same direction
                angle_rad = math.radians(global_angle)
                effort = speed * speed_factor * 3.0

                self.set_steering_effort(angle_rad, angle_rad, angle_rad, angle_rad)
                self.set_wheel_effort(effort, effort, effort, effort)

            elif mode == 'spin':
                spin_speed = command.get('spinSpeed', 0.0)

                # Spin: wheels point tangent to circle, opposite velocities
                effort = spin_speed * speed_factor * 3.0
                self.set_steering_effort(0.785, -0.785, -0.785, 0.785)  # ±45 degrees
                self.set_wheel_effort(effort, -effort, effort, -effort)

            elif mode == 'stop':
                self.set_wheel_effort(0.0, 0.0, 0.0, 0.0)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    async def websocket_handler(self, websocket):
        """Handle WebSocket connections"""
        self.connected_clients.add(websocket)
        self.get_logger().info(f'Client connected. Total clients: {len(self.connected_clients)}')

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
            self.connected_clients.remove(websocket)
            self.get_logger().info(f'Client disconnected. Total clients: {len(self.connected_clients)}')

    async def start_websocket_server(self):
        """Start WebSocket server"""
        server = await websockets.serve(self.websocket_handler, '0.0.0.0', 8765)
        self.get_logger().info('WebSocket server started on ws://0.0.0.0:8765')
        await server.wait_closed()

def main(args=None):
    rclpy.init(args=args)
    node = TadeoCarWebControl()

    # Run WebSocket server in separate thread
    loop = asyncio.new_event_loop()
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
