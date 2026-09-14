"""Serialize continuous local transmissions with the emergency latch.

No RC override, parameter or mode logic. Closing the latch waits for any
in-progress send; a subsequent queued setpoint cannot pass this boundary.
"""
import socket
from typing import Callable, Any


class GuardedControlSocket:
    def __init__(self, guard: Callable, family: int, kind: int,
                 socket_factory: Callable = socket.socket) -> None:
        self._guard = guard
        self._socket = socket_factory(family, kind)
        self._socket.settimeout(0.1)

    def sendto(self, packet: bytes, endpoint: Any) -> int:
        try:
            return self._guard(lambda: self._socket.sendto(packet, endpoint))
        except ValueError as exc:
            raise OSError("本地安全门拒绝连续设定值发送") from exc

    def close(self) -> None:
        self._socket.close()
