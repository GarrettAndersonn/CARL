"""Framing and packing for the master <-> companion computer link.

Frame format (mirrors CAN semantics so firmware translation is trivial):

    [SOF=0xAA][MSG_ID: u16 LE][LEN: u8][PAYLOAD ...][CRC8]

MSG_IDs reuse the CAN IDs from docs/can-bus.md so the master just proxies
between wire format and CAN frames.
"""
from __future__ import annotations

import struct

SOF = 0xAA

MSG_ESTOP = 0x000
MSG_HEARTBEAT = 0x010
MSG_THROTTLE_CMD = 0x100
MSG_THROTTLE_STATUS = 0x200
MSG_ENCODER_FEEDBACK = 0x210


def crc8(data: bytes, poly: int = 0x07) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(msg_id: int, payload: bytes) -> bytes:
    header = struct.pack('<BHB', SOF, msg_id, len(payload))
    body = header + payload
    return body + bytes([crc8(body)])


def pack_throttle_cmd(fl: float, fr: float, rl: float, rr: float) -> bytes:
    """Wheel commands normalized [-1.0, 1.0] -> int16 per-mille."""
    def q(x: float) -> int:
        return max(-1000, min(1000, int(round(x * 1000))))
    return struct.pack('<hhhh', q(fl), q(fr), q(rl), q(rr))


def pack_estop(reason: int) -> bytes:
    return struct.pack('<B', reason)


class Parser:
    """Incremental byte-stream parser. Feed bytes, yields (msg_id, payload)."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)
        while True:
            # scan for SOF
            i = self._buf.find(SOF)
            if i < 0:
                self._buf.clear()
                return
            if i > 0:
                del self._buf[:i]
            if len(self._buf) < 4:
                return
            msg_id, length = struct.unpack('<HB', bytes(self._buf[1:4]))
            frame_len = 4 + length + 1
            if len(self._buf) < frame_len:
                return
            body = bytes(self._buf[:frame_len - 1])
            expected = self._buf[frame_len - 1]
            if crc8(body) == expected:
                payload = bytes(self._buf[4:4 + length])
                del self._buf[:frame_len]
                yield msg_id, payload
            else:
                # bad CRC — drop SOF byte and retry
                del self._buf[0]
