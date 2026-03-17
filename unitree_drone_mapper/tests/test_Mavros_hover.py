#!/usr/bin/env python3
"""Minimal hover-in-place test: arms, takes off to 1.5m, hovers 10s, lands."""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped


class HoverTest(Node):
    def __init__(self):
        super().__init__('hover_test')
        
        self.state = State()
        self.target_altitude = 1.5  # meters
        self.hover_duration = 10.0  # seconds
        
        # State machine
        self.phase = 'WAITING'  # WAITING -> SENDING_SETPOINTS -> OFFBOARD -> ARMING -> HOVERING -> LANDING -> DONE
        self.setpoint_count = 0
        self.hover_start_time = None
        
        # Sub/Pub
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.arming_client.wait_for_service(timeout_sec=5.0)
        self.set_mode_client.wait_for_service(timeout_sec=5.0)
        
        # Main loop at 20Hz
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Hover test node started. Waiting for connection...')

    def state_cb(self, msg):
        self.state = msg

    def publish_setpoint(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.target_altitude
        self.local_pos_pub.publish(pose)

    def loop(self):
        # Always publish setpoint once we start (keeps OFFBOARD alive)
        if self.phase != 'WAITING' and self.phase != 'DONE':
            self.publish_setpoint()
        
        if self.phase == 'WAITING':
            if self.state.connected:
                self.get_logger().info('Connected! Sending setpoints before OFFBOARD...')
                self.phase = 'SENDING_SETPOINTS'
        
        elif self.phase == 'SENDING_SETPOINTS':
            self.publish_setpoint()
            self.setpoint_count += 1
            # Send ~100 setpoints (5 sec at 20Hz) before switching mode
            if self.setpoint_count >= 100:
                self.get_logger().info(f'Sent {self.setpoint_count} setpoints. Switching to OFFBOARD...')
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.offboard_response)
                self.phase = 'OFFBOARD'
        
        elif self.phase == 'OFFBOARD':
            if self.state.mode == 'OFFBOARD':
                self.get_logger().info('OFFBOARD confirmed! Arming...')
                req = CommandBool.Request()
                req.value = True
                future = self.arming_client.call_async(req)
                future.add_done_callback(self.arming_response)
                self.phase = 'ARMING'
        
        elif self.phase == 'ARMING':
            if self.state.armed:
                self.get_logger().info(f'Armed! Hovering at {self.target_altitude}m for {self.hover_duration}s...')
                self.hover_start_time = self.get_clock().now()
                self.phase = 'HOVERING'
        
        elif self.phase == 'HOVERING':
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
            if elapsed >= self.hover_duration:
                self.get_logger().info('Hover complete! Landing...')
                req = SetMode.Request()
                req.custom_mode = 'AUTO.LAND'
                self.set_mode_client.call_async(req)
                self.phase = 'LANDING'
        
        elif self.phase == 'LANDING':
            if not self.state.armed:
                self.get_logger().info('Landed and disarmed. Done!')
                self.phase = 'DONE'
                self.timer.cancel()

    def offboard_response(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'SetMode response: {resp.mode_sent}')
        except Exception as e:
            self.get_logger().error(f'SetMode failed: {e}')

    def arming_response(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'Arming response: {resp.success}')
        except Exception as e:
            self.get_logger().error(f'Arming failed: {e}')


def main():
    rclpy.init()
    node = HoverTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted! Landing...')
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        node.set_mode_client.call_async(req)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()