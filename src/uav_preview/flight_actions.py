from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import socket
from threading import Lock
from time import monotonic
from typing import Any, Callable

from .config import GroundOffboardTestConfig
from .types import TelemetrySnapshot


_MANUAL_ARM_MODES = {"MANUAL", "STABILIZED", "ALTCTL", "POSCTL", "POSITION"}


@dataclass(slots=True)
class ExplicitFlightActionState:
    available: bool = False
    commands_sent: int = 0
    last_action: str = ""
    last_command: int | None = None
    last_sent_monotonic: float | None = None
    error: str = ""

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["last_sent_age_seconds"] = (
            None
            if self.last_sent_monotonic is None
            else max(0.0, monotonic() - self.last_sent_monotonic)
        )
        result.pop("last_sent_monotonic", None)
        return result


class ExplicitFlightActionSender:
    """Send only user-confirmed ARM/DISARM and PX4 pause commands.

    This is deliberately separate from keyboard/vision control. It does not
    send MANUAL_CONTROL, RC overrides, setpoints, mode changes, takeoff, land,
    or RTL. Every call is one MAVLink COMMAND_LONG datagram initiated by an
    explicit web request and guarded by live telemetry.
    """

    def __init__(
        self,
        config: GroundOffboardTestConfig,
        telemetry_snapshot: Callable[[], TelemetrySnapshot],
        ground_sender_enabled: Callable[[], bool],
        estop_latched: Callable[[], bool],
        router_ready: Callable[[], bool],
        stale_after_seconds: float,
        ground_takeoff_ready: Callable[[], bool] | None = None,
        require_ground_takeoff_prestream: bool = False,
        socket_factory: Callable[[int, int], Any] = socket.socket,
    ) -> None:
        self.config = config
        self._telemetry_snapshot = telemetry_snapshot
        self._ground_sender_enabled = ground_sender_enabled
        self._estop_latched = estop_latched
        self._router_ready = router_ready
        self._stale_after_seconds = stale_after_seconds
        self._ground_takeoff_ready = ground_takeoff_ready or (lambda: False)
        self._require_ground_takeoff_prestream = require_ground_takeoff_prestream
        self._socket_factory = socket_factory
        self._state = ExplicitFlightActionState(available=config.available)
        self._lock = Lock()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._state.to_dict()

    def _common_check(self, telemetry: TelemetrySnapshot) -> None:
        if not self.config.available:
            raise ValueError("真实飞控动作未开放")
        if not self._router_ready():
            raise ValueError("MAVLink路由尚未就绪，拒绝发送真实飞控动作")
        age = telemetry.age_seconds()
        if not telemetry.connected or age is None or age > self._stale_after_seconds:
            raise ValueError("飞控遥测离线或已过期，拒绝发送真实飞控动作")

    def set_armed(self, requested: bool) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        if requested:
            if self._estop_latched():
                raise ValueError("本地急停仍处于锁存状态，拒绝ARM")
            if telemetry.armed is not False:
                raise ValueError("只有飞控明确处于未解锁状态时才能发送ARM")
            mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
            sender_enabled = self._ground_sender_enabled()
            if self._require_ground_takeoff_prestream and not sender_enabled:
                raise ValueError("电脑起飞ARM要求先开启并完成Offboard零速度预备")
            if sender_enabled:
                if not self._ground_takeoff_ready():
                    raise ValueError("Offboard零速度预备尚未达到稳定时长，拒绝ARM")
                if mode != "OFFBOARD":
                    raise ValueError(f"零速度预备已开启，但当前模式为{mode}；请由飞手手动切入OFFBOARD")
                if telemetry.landed_state != "ON_GROUND":
                    raise ValueError(
                        f"电脑起飞ARM只允许飞控明确报告ON_GROUND；当前为{telemetry.landed_state}"
                    )
                if telemetry.rc_channel_6_pwm is None:
                    raise ValueError("尚未收到实体Kill Switch通道值，拒绝电脑起飞ARM")
                if telemetry.rc_channel_6_pwm >= 1800:
                    raise ValueError("实体Kill Switch仍处于触发状态，请由飞手解除后再ARM")
            elif mode not in _MANUAL_ARM_MODES:
                raise ValueError(
                    f"当前模式为{mode}；为保留实体遥控器接管，ARM前必须切到Position/Altitude/Manual"
                )
            return self._send_command(400, "ARM", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), telemetry)

        if telemetry.armed is not True:
            raise ValueError("飞控当前没有解锁，无需发送DISARM")
        if telemetry.landed_state != "ON_GROUND":
            raise ValueError(
                f"当前落地状态为{telemetry.landed_state}；只允许在飞控明确报告ON_GROUND时DISARM"
            )
        return self._send_command(
            400,
            "DISARM",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            telemetry,
            allow_immediate=True,
        )

    def safety_disarm_on_ground(self) -> dict[str, Any]:
        """Send a normal DISARM while the emergency latch is engaged on ground.

        Unlike set_armed(False), this deliberately sends even if the latest
        heartbeat still says disarmed. That cancels a just-sent/pending ARM
        request without using PX4's force-disarm magic value.
        """
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        if telemetry.landed_state != "ON_GROUND":
            raise ValueError(
                f"安全锁存DISARM只允许在ON_GROUND发送；当前为{telemetry.landed_state}"
            )
        return self._send_command(
            400,
            "DISARM-SAFETY-LATCH",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            telemetry,
            allow_immediate=True,
        )

    def brake(self) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        if telemetry.landed_state == "IN_AIR" and telemetry.armed is not True:
            raise ValueError("飞控报告IN_AIR但当前未解锁，状态不一致，拒绝发送紧急制动")
        if telemetry.landed_state not in {"ON_GROUND", "IN_AIR"}:
            raise ValueError(
                f"紧急制动只允许在飞控明确报告ON_GROUND，或ARMED且IN_AIR时发送；当前为{telemetry.landed_state}"
            )
        # Match QGroundControl PX4FirmwarePlugin::pauseVehicle():
        # MAV_CMD_DO_REPOSITION, param1=-1, param2=CHANGE_MODE, remaining
        # position parameters empty (NaN).
        return self._send_command(
            192,
            "BRAKE/PAUSE-GROUND-TEST" if telemetry.landed_state == "ON_GROUND" else "BRAKE/PAUSE",
            (-1.0, 1.0, 0.0, math.nan, math.nan, math.nan, math.nan),
            telemetry,
            allow_immediate=True,
        )

    def land(self) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        if telemetry.armed is not True or telemetry.landed_state != "IN_AIR":
            raise ValueError("真实降落只允许在飞控明确报告ARMED且IN_AIR时发送")
        return self._send_command(
            21,
            "LAND",
            (0.0, 0.0, 0.0, math.nan, math.nan, math.nan, math.nan),
            telemetry,
            allow_immediate=True,
        )

    def rtl(self) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        if telemetry.armed is not True or telemetry.landed_state != "IN_AIR":
            raise ValueError("真实返航只允许在飞控明确报告ARMED且IN_AIR时发送")
        return self._send_command(
            20,
            "RTL",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            telemetry,
            allow_immediate=True,
        )

    def _send_command(
        self,
        command: int,
        action: str,
        params: tuple[float, float, float, float, float, float, float],
        telemetry: TelemetrySnapshot,
        allow_immediate: bool = False,
    ) -> dict[str, Any]:
        with self._lock:
            if (
                not allow_immediate
                and
                self._state.last_sent_monotonic is not None
                and monotonic() - self._state.last_sent_monotonic < 0.75
            ):
                raise ValueError("真实飞控命令发送过快，请等待后重试")

        try:
            from pymavlink.dialects.v20 import common as mavlink2

            mav = mavlink2.MAVLink(
                None,
                srcSystem=self.config.source_system,
                srcComponent=self.config.source_component,
            )
            message = mav.command_long_encode(
                telemetry.system_id or self.config.target_system,
                telemetry.component_id or self.config.target_component,
                command,
                0,
                *params,
            )
            packet = message.pack(mav)
            sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.sendto(packet, (self.config.uplink_host, self.config.uplink_port))
            finally:
                sock.close()
        except OSError as exc:
            with self._lock:
                self._state.error = f"真实飞控命令发送失败：{exc}"
            raise ValueError(self._state.error) from exc

        with self._lock:
            self._state.commands_sent += 1
            self._state.last_action = action
            self._state.last_command = command
            self._state.last_sent_monotonic = monotonic()
            self._state.error = ""
            return self._state.to_dict()
