#!/usr/bin/env python3

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import serial
from serial import SerialException


COMMAND_MIN = -1000
COMMAND_MAX = 1000
DEFAULT_BAUD = 115200
DEFAULT_RATE_HZ = 20.0
MAX_DURATION_SECONDS = 10.0


class DriveTester:
    def __init__(
        self,
        port: str,
        baud: int,
        vx: int,
        wz: int,
        duration: float,
        rate_hz: float,
        log_file: Path | None,
    ) -> None:
        self.port_name = port
        self.baud = baud
        self.vx = vx
        self.wz = wz
        self.duration = duration
        self.rate_hz = rate_hz
        self.log_file = log_file

        self.serial_port: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.reader_thread: threading.Thread | None = None
        self.log_handle = None

    def open(self) -> None:
        print(f"Opening STM32 serial port: {self.port_name}")

        self.serial_port = serial.Serial(
            port=self.port_name,
            baudrate=self.baud,
            timeout=0.10,
            write_timeout=1.0,
        )

        # Opening the ST-LINK virtual COM port may reset or reconnect the board.
        time.sleep(1.5)

        if self.log_file is not None:
            self.log_file.parent.mkdir(parents=True, exist_ok=True)
            self.log_handle = self.log_file.open(
                mode="a",
                encoding="utf-8",
                buffering=1,
            )
            print(f"Logging telemetry to: {self.log_file}")

    def close(self) -> None:
        self.stop_event.set()

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

        if self.log_handle is not None:
            self.log_handle.close()

    def send_line(self, text: str) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        payload = f"{text}\n".encode("ascii")
        self.serial_port.write(payload)
        self.serial_port.flush()

    def send_stop(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return

        print("Sending repeated STOP commands.")

        for _ in range(10):
            try:
                self.send_line("S")
                time.sleep(0.05)
            except (SerialException, OSError):
                break

    def read_telemetry(self) -> None:
        assert self.serial_port is not None

        while not self.stop_event.is_set():
            try:
                raw_line = self.serial_port.readline()

                if not raw_line:
                    continue

                line = raw_line.decode(
                    "utf-8",
                    errors="replace",
                ).rstrip()

                if not line:
                    continue

                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                output = f"[{timestamp}] {line}"

                print(output)

                if self.log_handle is not None:
                    self.log_handle.write(output + "\n")

            except (SerialException, OSError) as exc:
                if not self.stop_event.is_set():
                    print(
                        f"Serial read error: {exc}",
                        file=sys.stderr,
                    )
                    self.stop_event.set()
                return

    def run(self) -> None:
        assert self.serial_port is not None

        self.reader_thread = threading.Thread(
            target=self.read_telemetry,
            daemon=True,
        )
        self.reader_thread.start()

        period = 1.0 / self.rate_hz
        deadline = time.monotonic() + self.duration
        command = f"D {self.vx} {self.wz}"

        print()
        print("Drive test starting")
        print(f"Command: {command}")
        print(f"Duration: {self.duration:.2f} seconds")
        print(f"Refresh rate: {self.rate_hz:.1f} Hz")
        print("Press Ctrl+C at any time to stop.")
        print()

        while (
            time.monotonic() < deadline
            and not self.stop_event.is_set()
        ):
            self.send_line(command)
            time.sleep(period)

        self.send_stop()


def bounded_command(value: str) -> int:
    parsed = int(value)

    if not COMMAND_MIN <= parsed <= COMMAND_MAX:
        raise argparse.ArgumentTypeError(
            f"value must be from {COMMAND_MIN} to {COMMAND_MAX}"
        )

    return parsed


def bounded_duration(value: str) -> float:
    parsed = float(value)

    if not 0.1 <= parsed <= MAX_DURATION_SECONDS:
        raise argparse.ArgumentTypeError(
            f"duration must be from 0.1 to "
            f"{MAX_DURATION_SECONDS:.1f} seconds"
        )

    return parsed


def positive_rate(value: str) -> float:
    parsed = float(value)

    if not 5.0 <= parsed <= 50.0:
        raise argparse.ArgumentTypeError(
            "rate must be from 5 to 50 Hz"
        )

    return parsed


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send a continuously refreshed, time-limited "
            "differential-drive command to the Carl STM32 master."
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
        "--vx",
        type=bounded_command,
        required=True,
        help="Forward/reverse command from -1000 to 1000.",
    )

    parser.add_argument(
        "--wz",
        type=bounded_command,
        required=True,
        help="Turning command from -1000 to 1000.",
    )

    parser.add_argument(
        "--duration",
        type=bounded_duration,
        default=2.0,
        help="Test duration in seconds. Default: 2.0",
    )

    parser.add_argument(
        "--rate",
        type=positive_rate,
        default=DEFAULT_RATE_HZ,
        help="Command refresh rate in Hz. Default: 20",
    )

    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="Optional path for saving received telemetry.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    tester = DriveTester(
        port=args.port,
        baud=args.baud,
        vx=args.vx,
        wz=args.wz,
        duration=args.duration,
        rate_hz=args.rate,
        log_file=args.log,
    )

    def request_stop(_signum=None, _frame=None) -> None:
        print("\nStop requested.")
        tester.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        tester.open()
        tester.run()
        return 0

    except SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        tester.stop_event.set()
        return 130

    except Exception as exc:
        print(f"Unexpected error: {exc}", file=sys.stderr)
        return 1

    finally:
        tester.send_stop()
        tester.close()
        print("Drive test finished. STOP command sent.")


if __name__ == "__main__":
    raise SystemExit(main())
