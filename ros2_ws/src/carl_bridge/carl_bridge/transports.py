"""Swappable transport layer for talking to the CARL NUCLEO master.

Both transports expose the same interface: send(bytes) and a poll() that
returns any bytes available. The bridge_node stays agnostic to whether we're
running over USB serial or UDP/Ethernet.
"""
from __future__ import annotations

import socket
from abc import ABC, abstractmethod


class Transport(ABC):
    @abstractmethod
    def send(self, data: bytes) -> None: ...

    @abstractmethod
    def poll(self) -> bytes: ...

    @abstractmethod
    def close(self) -> None: ...


class SerialTransport(Transport):
    def __init__(self, port: str, baud: int):
        import serial
        self._ser = serial.Serial(port, baud, timeout=0)

    def send(self, data: bytes) -> None:
        self._ser.write(data)

    def poll(self) -> bytes:
        n = self._ser.in_waiting
        return self._ser.read(n) if n else b''

    def close(self) -> None:
        self._ser.close()


class UdpTransport(Transport):
    def __init__(self, remote_host: str, remote_port: int, local_port: int):
        self._remote = (remote_host, remote_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._sock.bind(('0.0.0.0', local_port))

    def send(self, data: bytes) -> None:
        self._sock.sendto(data, self._remote)

    def poll(self) -> bytes:
        try:
            data, _ = self._sock.recvfrom(1500)
            return data
        except BlockingIOError:
            return b''

    def close(self) -> None:
        self._sock.close()


def make_transport(kind: str, **kwargs) -> Transport:
    if kind == 'serial':
        return SerialTransport(kwargs['port'], kwargs['baud'])
    if kind == 'udp':
        return UdpTransport(kwargs['remote_host'], kwargs['remote_port'], kwargs['local_port'])
    raise ValueError(f'unknown transport: {kind}')
