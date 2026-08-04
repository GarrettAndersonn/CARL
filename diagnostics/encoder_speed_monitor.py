#!/usr/bin/env python3

"""
CARL rear-wheel encoder speed monitor.

Subscribes to:
    /carl/encoders

Displays:
    - cumulative encoder counts
    - counts per second
    - wheel RPM
    - wheel linear speed in meters/second
    - left/right speed mismatch

This script does not command the vehicle.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from carl_msgs.msg import CarlEncoderFeedback


# Measured wheel encoder calibration.
LEFT_COUNTS_PER_REV = 12031.0
RIGHT_COUNTS_PER_REV = 12181.0

# Measured wheel diameter.
WHEEL_DIAMETER_INCHES = 13.0
INCHES_TO_METERS = 0.0254
WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * INCHES_TO_METERS
WHEEL_CIRCUMFERENCE_METERS = math.pi * WHEEL_DIAMETER_METERS

# Ignore updates with an extremely small time interval.
MIN_DT_SECONDS = 0.001

# Simple low-pass filtering.
FILTER_ALPHA = 0.25


class EncoderSpeedMonitor(Node):
    def __init__(self) -> None:
        super().__init__("carl_encoder_speed_monitor")

        self.create_subscription(
            CarlEncoderFeedback,
            "/carl/encoders",
            self.encoder_callback,
            10,
        )

        self.previous_left_count: int | None = None
        self.previous_right_count: int | None = None
        self.previous_time: float | None = None

        self.filtered_left_cps = 0.0
        self.filtered_right_cps = 0.0

        self.last_left_count = 0
        self.last_right_count = 0

        self.create_timer(0.2, self.display_status)

        self.get_logger().info("CARL encoder speed monitor started")
        self.get_logger().info(
            f"Left calibration: {LEFT_COUNTS_PER_REV:.1f} counts/rev"
        )
        self.get_logger().info(
            f"Right calibration: {RIGHT_COUNTS_PER_REV:.1f} counts/rev"
        )
        self.get_logger().info(
            f"Wheel circumference: {WHEEL_CIRCUMFERENCE_METERS:.4f} m"
        )

    def encoder_callback(self, msg: CarlEncoderFeedback) -> None:
        current_time = time.monotonic()

        left_count = int(msg.rear_left_counts)
        right_count = int(msg.rear_right_counts)

        self.last_left_count = left_count
        self.last_right_count = right_count

        if (
            self.previous_left_count is None
            or self.previous_right_count is None
            or self.previous_time is None
        ):
            self.previous_left_count = left_count
            self.previous_right_count = right_count
            self.previous_time = current_time
            return

        dt = current_time - self.previous_time

        if dt < MIN_DT_SECONDS:
            return

        left_delta = left_count - self.previous_left_count
        right_delta = right_count - self.previous_right_count

        raw_left_cps = left_delta / dt
        raw_right_cps = right_delta / dt

        self.filtered_left_cps = (
            FILTER_ALPHA * raw_left_cps
            + (1.0 - FILTER_ALPHA) * self.filtered_left_cps
        )

        self.filtered_right_cps = (
            FILTER_ALPHA * raw_right_cps
            + (1.0 - FILTER_ALPHA) * self.filtered_right_cps
        )

        self.previous_left_count = left_count
        self.previous_right_count = right_count
        self.previous_time = current_time

    def display_status(self) -> None:
        left_rps = self.filtered_left_cps / LEFT_COUNTS_PER_REV
        right_rps = self.filtered_right_cps / RIGHT_COUNTS_PER_REV

        left_rpm = left_rps * 60.0
        right_rpm = right_rps * 60.0

        left_mps = left_rps * WHEEL_CIRCUMFERENCE_METERS
        right_mps = right_rps * WHEEL_CIRCUMFERENCE_METERS

        average_abs_rpm = (
            abs(left_rpm) + abs(right_rpm)
        ) / 2.0

        if average_abs_rpm > 0.01:
            mismatch_percent = (
                abs(abs(left_rpm) - abs(right_rpm))
                / average_abs_rpm
                * 100.0
            )
        else:
            mismatch_percent = 0.0

        print("\033[2J\033[H", end="")
        print("==============================================")
        print("          CARL WHEEL SPEED MONITOR")
        print("==============================================")
        print(f"Left count:        {self.last_left_count:12d}")
        print(f"Right count:       {self.last_right_count:12d}")
        print("----------------------------------------------")
        print(f"Left counts/sec:   {self.filtered_left_cps:12.2f}")
        print(f"Right counts/sec:  {self.filtered_right_cps:12.2f}")
        print("----------------------------------------------")
        print(f"Left RPM:          {left_rpm:12.2f}")
        print(f"Right RPM:         {right_rpm:12.2f}")
        print("----------------------------------------------")
        print(f"Left speed:        {left_mps:12.3f} m/s")
        print(f"Right speed:       {right_mps:12.3f} m/s")
        print("----------------------------------------------")
        print(f"Speed mismatch:    {mismatch_percent:12.2f} %")
        print("==============================================")
        print("Press Ctrl+C to stop the monitor.")


def main() -> None:
    rclpy.init()

    node = EncoderSpeedMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
