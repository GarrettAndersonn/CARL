#!/usr/bin/env python3
"""
AVL_CARL teleop — keyboard drive-by-wire test console.

Talks to the MASTER node (NUCLEO-F767ZI) over its USB serial Virtual COM Port
(the same cable that flashes the board). The master mixes what we send into
per-wheel differential-drive commands and puts them on the CAN bus; the throttle
node drives the motors. There are NO brakes — "stop" means command zero.

Controls
--------
  1 2 3 4   set speed level (25% / 50% / 75% / 100%)
  w         drive forward
  s         drive reverse
  a         steer left        d   steer right
  c         center steering (go straight)
  x / SPACE STOP — zero throttle and hold (this is the "brake")
  e         E-STOP (latch bus-wide safe state)
  r         reset E-STOP
  q         quit (sends STOP first)

  Bring-up single-wheel test (spin ONE motor forward at the current speed level):
  7 = FL    8 = FR    9 = RL    0 = RR     (any of w/a/s/d/x exits test mode)

Safety
------
This script streams the current command at 20 Hz. The master runs a teleop
watchdog: if these packets stop (script crash / USB unplug), the master forces
zero and the kart stops. Always keep a hardware kill switch within reach.

Usage
-----
  python teleop.py --port COM5           (Windows)
  python teleop.py --port /dev/ttyACM0   (Linux)
  python teleop.py --list                list serial ports and exit

Requires: pyserial  ->  pip install pyserial
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# ── Command scaling (per-mille, matches firmware THROTTLE_MIN/MAX) ────────────
SPEED_BY_LEVEL = {1: 250, 2: 500, 3: 750, 4: 1000}  # forward/reverse magnitude
TURN_BY_LEVEL = {1: 150, 2: 300, 3: 450, 4: 600}    # turn magnitude (gentler)

SEND_PERIOD = 0.05  # 20 Hz command stream (must beat the master teleop timeout)


# ── Cross-platform single-key non-blocking reader ────────────────────────────
class KeyReader:
    def __init__(self):
        self._win = sys.platform.startswith("win")
        if self._win:
            import msvcrt  # noqa: F401
            self._msvcrt = __import__("msvcrt")
        else:
            import termios
            import tty
            self._termios = termios
            self._tty = tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        if not self._win:
            self._tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if not self._win:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)

    def get_keys(self):
        """Return all keys pressed since the last call (possibly empty)."""
        keys = []
        if self._win:
            while self._msvcrt.kbhit():
                ch = self._msvcrt.getwch()
                keys.append(ch)
        else:
            import select
            while select.select([sys.stdin], [], [], 0)[0]:
                keys.append(sys.stdin.read(1))
        return keys


def pick_port(arg_port):
    if arg_port:
        return arg_port
    ports = list(list_ports.comports())
    if len(ports) == 1:
        print(f"Auto-selected the only serial port: {ports[0].device}")
        return ports[0].device
    print("Multiple/zero serial ports found. Use --port. Available:")
    for p in ports:
        print(f"  {p.device:12s} {p.description}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description="AVL_CARL keyboard teleop")
    ap.add_argument("--port", help="serial port of the master (e.g. COM5, /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device:12s} {p.description}")
        return

    port = pick_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=0)
    time.sleep(0.2)
    ser.reset_input_buffer()
    print(f"Connected to {port} @ {args.baud}. Press keys to drive, 'q' to quit.\n")

    level = 1
    vx = 0   # forward(+)/reverse(-) per-mille
    wz = 0   # turn right(+)/left(-) per-mille
    test_idx = None  # bring-up: spin one wheel (0=FL 1=FR 2=RL 3=RR); None = normal
    estop = False
    last_send = 0.0
    last_tlm = ""
    rx = b""

    def send(line):
        ser.write((line + "\n").encode("ascii"))

    with KeyReader() as kr:
        try:
            while True:
                # 1) Handle keys.
                for ch in kr.get_keys():
                    c = ch.lower()
                    if c == "q":
                        send("S")
                        print("\nquit.")
                        return
                    elif c in "1234":
                        level = int(c)
                    elif c in "7890":  # bring-up: spin ONE wheel (7=FL 8=FR 9=RL 0=RR)
                        test_idx = {"7": 0, "8": 1, "9": 2, "0": 3}[c]
                    elif c == "w":
                        vx = SPEED_BY_LEVEL[level]
                        test_idx = None
                    elif c == "s":
                        vx = -SPEED_BY_LEVEL[level]
                        test_idx = None
                    elif c == "a":
                        wz = -TURN_BY_LEVEL[level]
                        test_idx = None
                    elif c == "d":
                        wz = TURN_BY_LEVEL[level]
                        test_idx = None
                    elif c == "c":
                        wz = 0
                    elif c == "x" or ch == " ":
                        vx, wz = 0, 0
                        test_idx = None
                        send("S")
                    elif c == "e":
                        estop = True
                        vx, wz = 0, 0
                        send("E")
                    elif c == "r":
                        estop = False
                        send("R")

                # 2) Stream the current command at a fixed rate (feeds the
                #    master's teleop watchdog).
                now = time.time()
                if now - last_send >= SEND_PERIOD:
                    last_send = now
                    if not estop:
                        if test_idx is not None:
                            send(f"M {test_idx} {SPEED_BY_LEVEL[level]}")
                        else:
                            send(f"D {vx} {wz}")

                # 3) Read + show telemetry from the master.
                data = ser.read(256)
                if data:
                    rx += data
                    while b"\n" in rx:
                        line, rx = rx.split(b"\n", 1)
                        text = line.decode("ascii", "replace").strip()
                        if text.startswith("TLM"):
                            last_tlm = text

                # 4) One-line status display.
                if estop:
                    state = "E-STOP"
                elif test_idx is not None:
                    state = f"TEST-{['FL','FR','RL','RR'][test_idx]}"
                elif vx or wz:
                    state = "DRIVE"
                else:
                    state = "idle  "
                status = (f"\r[L{level} {state}] vx={vx:+5d} wz={wz:+5d} | "
                          f"{last_tlm[:88]:<88}")
                sys.stdout.write(status)
                sys.stdout.flush()

                time.sleep(0.005)
        except KeyboardInterrupt:
            send("S")
            print("\ninterrupted — sent STOP.")
        finally:
            try:
                send("S")
                ser.flush()
            except Exception:
                pass
            ser.close()


if __name__ == "__main__":
    main()
