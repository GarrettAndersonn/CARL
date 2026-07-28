"""Keyboard teleop for CARL — publishes CarlThrottleCmd directly.

Hold-to-drive semantics via terminal key repeat. When you hold w/s/a/d the
key auto-repeats; when you release, repeats stop and the node zeros the
corresponding axis within ~hold_timeout seconds.

Controls
  1 2 3 4   speed = 25% / 50% / 75% / 100%
  w s       throttle forward / reverse   (hold)
  a d       turn left / right            (hold; both sides spin opposite)
  space     soft stop — releases all held keys immediately
  n         NEUTRAL — motors disabled regardless of keys
  g         DRIVE   — motors follow keys
  x         E-STOP  — latches, forces zero (master + local)
  r         RESET E-STOP — clears the latch on master AND locally
  q         quit

Caveats
- Your OS has an initial ~500ms delay before a held key starts repeating.
  hold_timeout defaults to 0.6s to keep first-press motion alive until the
  repeats kick in. Set lower for tighter release feel once you're moving.
- E-stop cannot be cleared from here. Restart the node AND power-cycle the
  NUCLEO master to clear the firmware latch.
"""
from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from carl_msgs.msg import CarlEstop, CarlThrottleCmd


SPEED_LEVELS = {1: 0.25, 2: 0.50, 3: 0.75, 4: 1.00}

HELP = """
CARL keyboard teleop
  1/2/3/4: speed   w/s: fwd/rev (hold)   a/d: turn L/R (hold)
  space: soft stop   n: NEUTRAL   g: DRIVE   x: E-STOP   r: RESET e-stop   q: quit
"""


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('carl_keyboard_teleop')
        self.declare_parameter('publish_rate_hz', 20.0)
        # Adaptive: wide grace period bridges the ~500ms initial repeat delay;
        # tight release fires once continuous auto-repeat is detected.
        self.declare_parameter('grace_timeout_s', 0.55)
        self.declare_parameter('release_timeout_s', 0.10)
        self.declare_parameter('repeat_detect_s', 0.10)

        self._speed_level = 1
        self._mode = 'DRIVE'          # DRIVE or NEUTRAL
        self._estop = False
        # deadline (monotonic seconds) — key is "active" while now < deadline
        self._deadline = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
        # last press timestamp — used to detect auto-repeat
        self._last_press = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}

        self._pub_throttle = self.create_publisher(CarlThrottleCmd, 'carl/throttle_cmd', 10)
        self._pub_estop = self.create_publisher(CarlEstop, 'carl/estop', 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish)

        self._stdin_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(HELP)
        self._print_state()

    def _get_key(self, timeout: float = 0.0) -> str:
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ''

    def poll_key(self):
        key = self._get_key(0.0)
        if not key:
            return True

        now = time.monotonic()

        if key in '1234':
            self._speed_level = int(key)
        elif key in 'wsad':
            interval = now - self._last_press[key]
            repeat_detect = self.get_parameter('repeat_detect_s').value
            if interval < repeat_detect:
                # auto-repeat mode → tight release
                timeout = self.get_parameter('release_timeout_s').value
            else:
                # fresh press → wide grace to bridge OS auto-repeat delay
                timeout = self.get_parameter('grace_timeout_s').value
            self._deadline[key] = now + timeout
            self._last_press[key] = now
        elif key == ' ':
            for k in self._deadline:
                self._deadline[k] = 0.0
        elif key == 'n':
            self._mode = 'NEUTRAL'
            for k in self._deadline:
                self._deadline[k] = 0.0
        elif key == 'g':
            self._mode = 'DRIVE'
        elif key == 'x':
            self._estop = True
            for k in self._deadline:
                self._deadline[k] = 0.0
            m = CarlEstop()
            m.reason = CarlEstop.REASON_OPERATOR
            self._pub_estop.publish(m)
            self.get_logger().warn('E-STOP LATCHED')
        elif key == 'r':
            self._estop = False
            m = CarlEstop()
            m.reason = CarlEstop.REASON_RESET
            self._pub_estop.publish(m)
            self.get_logger().info('E-STOP RESET sent')
        elif key == 'q' or key == '\x03':
            return False

        self._print_state()
        return True

    def _active(self, key: str, now: float) -> bool:
        return now < self._deadline[key]

    def _publish(self):
        m = CarlThrottleCmd()
        m.header.stamp = self.get_clock().now().to_msg()

        if self._estop or self._mode == 'NEUTRAL':
            self._pub_throttle.publish(m)
            return

        now = time.monotonic()
        speed = SPEED_LEVELS[self._speed_level]

        throttle_dir = 0
        if self._active('w', now):
            throttle_dir = 1
        elif self._active('s', now):
            throttle_dir = -1

        turn_dir = 0
        if self._active('a', now):
            turn_dir = 1   # left turn: right wheels forward, left wheels reverse
        elif self._active('d', now):
            turn_dir = -1  # right turn: left wheels forward, right wheels reverse

        throttle = throttle_dir * speed
        turn = turn_dir * speed

        left = _clamp(throttle - turn, -1.0, 1.0)
        right = _clamp(throttle + turn, -1.0, 1.0)

        m.front_left = m.rear_left = left
        m.front_right = m.rear_right = right
        self._pub_throttle.publish(m)

    def _print_state(self):
        speed = SPEED_LEVELS[self._speed_level]
        estop = ' ESTOP' if self._estop else ''
        sys.stdout.write(
            f'\r[{self._mode}{estop}]  speed={self._speed_level} ({int(speed * 100)}%)   '
        )
        sys.stdout.flush()

    def shutdown(self):
        zero = CarlThrottleCmd()
        zero.header.stamp = self.get_clock().now().to_msg()
        self._pub_throttle.publish(zero)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
        sys.stdout.write('\n')


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main():
    rclpy.init()
    node = KeyboardTeleopNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not node.poll_key():
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
