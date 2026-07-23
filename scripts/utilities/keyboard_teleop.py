#!/usr/bin/env python3

import argparse
import select
import signal
import sys
import termios
import threading
import time
import tty

import serial
from serial import SerialException


COMMAND_MIN = -1000
COMMAND_MAX = 1000
DEFAULT_BAUD = 115200
DEFAULT_RATE_HZ = 20.0
DEFAULT_FORWARD_SPEED = 500
DEFAULT_TURN_SPEED = 400


class KeyboardTeleop:
    def __init__(
        self,
        port: str,
        baud: int,
        forward_speed: int,
        turn_speed: int,
        rate_hz: float,
    ) -> None:
        self.port_name = port
        self.baud = baud
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.rate_hz = rate_hz

        self.serial_port: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.reader_thread: threading.Thread | None = None

        self.vx = 0
        self.wz = 0

        self.original_terminal_settings = None

    def open(self) -> None:
        print(f"Opening STM32 serial port: {self.port_name}")

        self.serial_port = serial.Serial(
            port=self.port_name,
            baudrate=self.baud,
            timeout=0.05,
            write_timeout=1.0,
        )

        time.sleep(1.5)

        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self) -> None:
        if self.original_terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.original_terminal_settings,
            )

    def close(self) -> None:
        self.stop_event.set()

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)

        self.send_stop()

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

        self.restore_terminal()

    def send_line(self, text: str) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        self.serial_port.write(f"{text}\n".encode("ascii"))
        self.serial_port.flush()

    def send_stop(self) -> None:
        self.vx = 0
        self.wz = 0

        if self.serial_port is None or not self.serial_port.is_open:
            return

        for _ in range(10):
            try:
                self.send_line("S")
                time.sleep(0.03)
            except (SerialException, OSError):
                break

    def telemetry_reader(self) -> None:
        assert self.serial_port is not None

        while not self.stop_event.is_set():
            try:
                raw_line = self.serial_port.readline()

                if not raw_line:
                    continue

                line = raw_line.decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                if line.startswith("TLM") or line.startswith("IMU"):
                    print(f"\r{line[:110]:<110}", end="", flush=True)

            except (SerialException, OSError):
                self.stop_event.set()
                return

    def print_controls(self) -> None:
        print()
        print("Carl Differential-Drive Keyboard Teleoperation")
        print()
        print("Controls:")
        print("  W = Forward")
        print("  S = Backward")
        print("  A = Turn left")
        print("  D = Turn right")
        print("  X or Space = Stop")
        print("  + = Increase forward speed")
        print("  - = Decrease forward speed")
        print("  ] = Increase turn speed")
        print("  [ = Decrease turn speed")
        print("  Q = Stop and quit")
        print()
        print(f"Forward speed: {self.forward_speed}")
        print(f"Turn speed:    {self.turn_speed}")
        print()

    def process_key(self, key: str) -> bool:
        key = key.lower()

        if key == "w":
            self.vx = self.forward_speed
            self.wz = 0
            print(
                f"\nFORWARD   vx={self.vx} wz={self.wz}",
                flush=True,
            )

        elif key == "s":
            self.vx = -self.forward_speed
            self.wz = 0
            print(
                f"\nBACKWARD  vx={self.vx} wz={self.wz}",
                flush=True,
            )

        elif key == "a":
            self.vx = 0
            self.wz = self.turn_speed
            print(
                f"\nLEFT      vx={self.vx} wz={self.wz}",
                flush=True,
            )

        elif key == "d":
            self.vx = 0
            self.wz = -self.turn_speed
            print(
                f"\nRIGHT     vx={self.vx} wz={self.wz}",
                flush=True,
            )

        elif key in ("x", " "):
            self.vx = 0
            self.wz = 0
            self.send_stop()
            print("\nSTOP", flush=True)

        elif key == "+" or key == "=":
            self.forward_speed = min(
                COMMAND_MAX,
                self.forward_speed + 50,
            )
            print(
                f"\nForward speed increased to "
                f"{self.forward_speed}",
                flush=True,
            )

        elif key == "-":
            self.forward_speed = max(
                50,
                self.forward_speed - 50,
            )
            print(
                f"\nForward speed decreased to "
                f"{self.forward_speed}",
                flush=True,
            )

        elif key == "]":
            self.turn_speed = min(
                COMMAND_MAX,
                self.turn_speed + 50,
            )
            print(
                f"\nTurn speed increased to "
                f"{self.turn_speed}",
                flush=True,
            )

        elif key == "[":
            self.turn_speed = max(
                50,
                self.turn_speed - 50,
            )
            print(
                f"\nTurn speed decreased to "
                f"{self.turn_speed}",
                flush=True,
            )

        elif key == "q":
            print("\nQuit requested.", flush=True)
            self.send_stop()
            return False

        return True

    def run(self) -> None:
        assert self.serial_port is not None

        self.reader_thread = threading.Thread(
            target=self.telemetry_reader,
            daemon=True,
        )
        self.reader_thread.start()

        self.print_controls()

        period = 1.0 / self.rate_hz
        running = True

        while running and not self.stop_event.is_set():
            readable, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                period,
            )

            if readable:
                key = sys.stdin.read(1)
                running = self.process_key(key)

            if self.vx == 0 and self.wz == 0:
                self.send_line("S")
            else:
                self.send_line(f"D {self.vx} {self.wz}")

        self.send_stop()


def bounded_command(value: str) -> int:
    parsed = int(value)

    if not 50 <= parsed <= COMMAND_MAX:
        raise argparse.ArgumentTypeError(
            "speed must be between 50 and 1000"
        )

    return parsed


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Keyboard teleoperation for the Carl "
            "differential-drive platform."
        )
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="STM32 serial port. Default: /dev/ttyACM0",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate. Default: {DEFAULT_BAUD}",
    )

    parser.add_argument(
        "--speed",
        type=bounded_command,
        default=DEFAULT_FORWARD_SPEED,
        help="Forward/reverse command. Default: 500",
    )

    parser.add_argument(
        "--turn-speed",
        type=bounded_command,
        default=DEFAULT_TURN_SPEED,
        help="Turning command. Default: 400",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="Command refresh frequency. Default: 20 Hz",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    teleop = KeyboardTeleop(
        port=args.port,
        baud=args.baud,
        forward_speed=args.speed,
        turn_speed=args.turn_speed,
        rate_hz=args.rate,
    )

    def request_stop(_signum=None, _frame=None) -> None:
        teleop.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        teleop.open()
        teleop.run()
        return 0

    except SerialException as exc:
        print(f"\nSerial error: {exc}", file=sys.stderr)
        return 1

    except Exception as exc:
        print(f"\nUnexpected error: {exc}", file=sys.stderr)
        return 1

    finally:
        teleop.close()
        print("\nTeleoperation stopped. STOP command sent.")


if __name__ == "__main__":
    raise SystemExit(main())
