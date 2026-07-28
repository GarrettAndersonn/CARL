"""Convert sensor_msgs/Joy to CARL throttle + estop commands.

Pure skid-steer: throttle axis sets forward power, steer axis biases L vs R wheels.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from carl_msgs.msg import CarlEstop, CarlThrottleCmd


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('carl_joy_teleop')

        self.declare_parameter('axis_throttle', 1)
        self.declare_parameter('axis_steer', 3)
        self.declare_parameter('button_estop', 1)
        self.declare_parameter('button_deadman', 4)
        self.declare_parameter('max_throttle', 0.4)
        self.declare_parameter('max_turn', 0.4)

        self._prev_estop = 0

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._pub_throttle = self.create_publisher(CarlThrottleCmd, 'carl/throttle_cmd', 10)
        self._pub_estop = self.create_publisher(CarlEstop, 'carl/estop', 10)

        self.get_logger().info('joy teleop ready — hold deadman to drive')

    def _on_joy(self, msg: Joy):
        estop_btn = int(msg.buttons[self.get_parameter('button_estop').value])
        if estop_btn and not self._prev_estop:
            m = CarlEstop()
            m.reason = CarlEstop.REASON_OPERATOR
            self._pub_estop.publish(m)
        self._prev_estop = estop_btn

        deadman = msg.buttons[self.get_parameter('button_deadman').value]
        if not deadman:
            self._publish_zero()
            return

        throttle = msg.axes[self.get_parameter('axis_throttle').value] * self.get_parameter('max_throttle').value
        turn = msg.axes[self.get_parameter('axis_steer').value] * self.get_parameter('max_turn').value

        left = _clamp(throttle - turn, -1.0, 1.0)
        right = _clamp(throttle + turn, -1.0, 1.0)

        t = CarlThrottleCmd()
        t.header.stamp = self.get_clock().now().to_msg()
        t.front_left = t.rear_left = left
        t.front_right = t.rear_right = right
        self._pub_throttle.publish(t)

    def _publish_zero(self):
        t = CarlThrottleCmd()
        t.header.stamp = self.get_clock().now().to_msg()
        self._pub_throttle.publish(t)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main():
    rclpy.init()
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
