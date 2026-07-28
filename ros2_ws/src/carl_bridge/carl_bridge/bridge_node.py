"""Bridge node: ROS2 topics <-> NUCLEO master over serial or UDP."""
from __future__ import annotations

import struct

import rclpy
from rclpy.node import Node

from carl_msgs.msg import (
    CarlEncoderFeedback,
    CarlEstop,
    CarlHeartbeat,
    CarlThrottleCmd,
    CarlThrottleStatus,
)

from carl_bridge import protocol
from carl_bridge.transports import make_transport


class BridgeNode(Node):
    def __init__(self):
        super().__init__('carl_bridge')

        self.declare_parameter('transport', 'serial')
        self.declare_parameter('serial.port', '/dev/ttyACM0')
        self.declare_parameter('serial.baud', 921600)
        self.declare_parameter('udp.remote_host', '192.168.1.10')
        self.declare_parameter('udp.remote_port', 5000)
        self.declare_parameter('udp.local_port', 5001)
        self.declare_parameter('poll_rate_hz', 200.0)

        kind = self.get_parameter('transport').value
        if kind == 'serial':
            self._transport = make_transport(
                'serial',
                port=self.get_parameter('serial.port').value,
                baud=self.get_parameter('serial.baud').value,
            )
        else:
            self._transport = make_transport(
                'udp',
                remote_host=self.get_parameter('udp.remote_host').value,
                remote_port=self.get_parameter('udp.remote_port').value,
                local_port=self.get_parameter('udp.local_port').value,
            )

        self._parser = protocol.Parser()

        self.create_subscription(CarlThrottleCmd, 'carl/throttle_cmd', self._on_throttle_cmd, 10)
        self.create_subscription(CarlEstop, 'carl/estop', self._on_estop, 10)

        self._pub_throttle_status = self.create_publisher(CarlThrottleStatus, 'carl/throttle_status', 10)
        self._pub_encoders = self.create_publisher(CarlEncoderFeedback, 'carl/encoders', 10)
        self._pub_heartbeat = self.create_publisher(CarlHeartbeat, 'carl/heartbeat', 10)

        poll_hz = self.get_parameter('poll_rate_hz').value
        self.create_timer(1.0 / poll_hz, self._poll)

        self.get_logger().info(f'carl_bridge up on transport={kind}')

    def _on_throttle_cmd(self, msg: CarlThrottleCmd):
        payload = protocol.pack_throttle_cmd(msg.front_left, msg.front_right, msg.rear_left, msg.rear_right)
        self._transport.send(protocol.frame(protocol.MSG_THROTTLE_CMD, payload))

    def _on_estop(self, msg: CarlEstop):
        payload = protocol.pack_estop(msg.reason)
        self._transport.send(protocol.frame(protocol.MSG_ESTOP, payload))

    def _poll(self):
        data = self._transport.poll()
        if not data:
            return
        for msg_id, payload in self._parser.feed(data):
            self._dispatch(msg_id, payload)

    def _dispatch(self, msg_id: int, payload: bytes):
        stamp = self.get_clock().now().to_msg()
        if msg_id == protocol.MSG_HEARTBEAT and len(payload) == 3:
            counter, flags, mode = struct.unpack('<BBB', payload)
            m = CarlHeartbeat()
            m.header.stamp = stamp
            m.counter, m.flags, m.mode = counter, flags, mode
            self._pub_heartbeat.publish(m)
        elif msg_id == protocol.MSG_THROTTLE_STATUS and len(payload) == 6:
            state, faults, ap_l, ap_r = struct.unpack('<BBhh', payload)
            m = CarlThrottleStatus()
            m.header.stamp = stamp
            m.state, m.faults = state, faults
            m.applied_left = ap_l / 1000.0
            m.applied_right = ap_r / 1000.0
            self._pub_throttle_status.publish(m)
        elif msg_id == protocol.MSG_ENCODER_FEEDBACK and len(payload) == 8:
            rl, rr = struct.unpack('<ii', payload)
            m = CarlEncoderFeedback()
            m.header.stamp = stamp
            m.rear_left_counts, m.rear_right_counts = rl, rr
            self._pub_encoders.publish(m)


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
