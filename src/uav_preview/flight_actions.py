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
    """Send only user-confirmed, telemetry-guarded high-level commands.

    This is deliberately separate from keyboard/vision control. It does not
    send MANUAL_CONTROL, RC overrides, continuous setpoints or mode changes.
    Every call is one MAVLink command datagram initiated by an explicit web
    request (or its guarded takeoff state machine) and checked against live
    telemetry.
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
        physical_offboard_switch_pwm_min: int = 1800,
        socket_factory: Callable[[int, int], Any] = socket.socket,
        unlatched_action_guard: Callable[[Callable[[], Any]], Any] | None = None,
    ) -> None:
        self.config = config
        self._telemetry_snapshot = telemetry_snapshot
        self._ground_sender_enabled = ground_sender_enabled
        self._estop_latched = estop_latched
        self._router_ready = router_ready
        self._stale_after_seconds = stale_after_seconds
        self._ground_takeoff_ready = ground_takeoff_ready or (lambda: False)
        self._require_ground_takeoff_prestream = require_ground_takeoff_prestream
        self._physical_offboard_switch_pwm_min = physical_offboard_switch_pwm_min
        self._socket_factory = socket_factory
        self._unlatched_action_guard = (
            unlatched_action_guard or self._run_if_unlatched_fallback
        )
        self._state = ExplicitFlightActionState(available=config.available)
        self._lock = Lock()

    def _run_if_unlatched_fallback(self, action: Callable[[], Any]) -> Any:
        """Compatibility guard for isolated users of this sender.

        Runtime supplies SafetyGate.run_if_unlatched, which is atomic with the
        latch transition.  This fallback still fails closed, but callers that
        need the concurrency guarantee must supply the shared gate guard.
        """

        if self._estop_latched():
            raise ValueError("网页安全处置仍处于锁存状态，拒绝发送该飞控动作")
        return action()

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

    def _require_fresh_fields(
        self,
        telemetry: TelemetrySnapshot,
        fields: tuple[tuple[str, str], ...],
    ) -> None:
        now = monotonic()
        for attribute, label in fields:
            timestamp = getattr(telemetry, attribute, None)
            if (
                timestamp is None
                or max(0.0, now - float(timestamp)) > self._stale_after_seconds
            ):
                raise ValueError(f"{label}遥测缺失或已过期，拒绝发送真实飞控动作")

    def set_armed(self, requested: bool) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        if requested:
            self._require_fresh_fields(
                telemetry,
                (
                    ("last_heartbeat_monotonic", "解锁状态"),
                    ("last_extended_state_monotonic", "落地状态"),
                    ("last_rc_channels_monotonic", "实体RC通道"),
                ),
            )
            if self._estop_latched():
                raise ValueError("本地急停仍处于锁存状态，拒绝ARM")
            if telemetry.armed is not False:
                raise ValueError("只有飞控明确处于未解锁状态时才能发送ARM")
            mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
            sender_enabled = self._ground_sender_enabled()
            if sender_enabled:
                raise ValueError("ARM前必须关闭所有电脑Offboard设定值发送器")
            if self._require_ground_takeoff_prestream:
                raise ValueError("浏览器从地面Offboard起飞已被安全策略禁用")
            if telemetry.landed_state != "ON_GROUND":
                raise ValueError(
                    f"网页ARM只允许飞控明确报告ON_GROUND；当前为{telemetry.landed_state}"
                )
            if mode not in _MANUAL_ARM_MODES:
                raise ValueError(
                    f"当前模式为{mode}；为保留实体遥控器接管，ARM前必须切到Position/Altitude/Manual"
                )
            if telemetry.rc_channel_6_pwm is None:
                raise ValueError("尚未收到实体Kill Switch（CH6）值，拒绝网页ARM")
            if telemetry.rc_channel_6_pwm >= 1800:
                raise ValueError("实体Kill Switch（CH6）仍处于触发状态，拒绝网页ARM")
            if telemetry.rc_channel_8_pwm is None:
                raise ValueError("尚未收到实体模式开关（CH8）值，拒绝网页ARM")
            if telemetry.rc_channel_8_pwm >= self._physical_offboard_switch_pwm_min:
                raise ValueError("实体CH8仍选择Offboard；请先切回Position/手动辅助模式再ARM")
            return self._send_command(400, "ARM", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), telemetry)

        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
            ),
        )
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
            allow_when_latched=True,
        )

    def arm_for_local_offboard_takeoff(self, prestream_ready: bool) -> dict[str, Any]:
        """ARM only for the dedicated, already-streaming local takeoff flow.

        The public ARM endpoint intentionally keeps rejecting ground Offboard
        arming.  This narrower entry point is called only by the local takeoff
        coordinator after it has streamed a fixed local-position hold and the
        pilot has physically selected Offboard on channel 8.
        """

        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
                ("last_rc_channels_monotonic", "实体RC通道"),
            ),
        )
        if not prestream_ready:
            raise ValueError("本地Offboard起飞预发送尚未稳定，拒绝ARM")
        if self._estop_latched():
            raise ValueError("网页安全处置仍处于锁存状态，拒绝ARM")
        if telemetry.armed is not False or telemetry.landed_state != "ON_GROUND":
            raise ValueError("本地Offboard ARM只允许从DISARMED + ON_GROUND开始")
        mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
        if mode != "OFFBOARD":
            raise ValueError(f"飞手尚未用实体CH8切入Offboard；当前为{mode}")
        if telemetry.rc_channel_6_pwm is None or telemetry.rc_channel_6_pwm >= 1800:
            raise ValueError("实体Kill Switch（CH6）必须已解除且有实时数值")
        if (
            telemetry.rc_channel_8_pwm is None
            or telemetry.rc_channel_8_pwm < self._physical_offboard_switch_pwm_min
        ):
            raise ValueError("实体CH8没有确认选择Offboard，拒绝ARM")
        return self._send_command(
            400,
            "ARM-LOCAL-OFFBOARD",
            (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            telemetry,
        )

    def safety_disarm_on_ground(self) -> dict[str, Any]:
        """Send a normal DISARM while the emergency latch is engaged on ground.

        Unlike set_armed(False), this deliberately sends even if the latest
        heartbeat still says disarmed. That cancels a just-sent/pending ARM
        request without using PX4's force-disarm magic value.
        """
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
            ),
        )
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
            allow_when_latched=True,
        )

    def brake(self) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
            ),
        )
        if self._estop_latched():
            raise ValueError("网页安全处置仍处于锁存状态，拒绝发送Pause")
        if telemetry.landed_state == "IN_AIR" and telemetry.armed is not True:
            raise ValueError("飞控报告IN_AIR但当前未解锁，状态不一致，拒绝发送空中暂停")
        if telemetry.landed_state not in {"ON_GROUND", "IN_AIR"}:
            raise ValueError(
                f"PX4 Pause只允许在飞控明确报告ON_GROUND，或ARMED且IN_AIR时发送；当前为{telemetry.landed_state}"
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
        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
            ),
        )
        if telemetry.armed is not True or telemetry.landed_state not in {"IN_AIR", "TAKEOFF"}:
            raise ValueError("真实降落只允许在飞控明确报告ARMED且IN_AIR/TAKEOFF时发送")
        return self._send_command(
            21,
            "LAND",
            (0.0, 0.0, 0.0, math.nan, math.nan, math.nan, math.nan),
            telemetry,
            allow_immediate=True,
            allow_when_latched=True,
        )

    def takeoff(self, target_height_m: float, ground_amsl_m: float) -> dict[str, Any]:
        """Send one native PX4 takeoff command without changing PX4 parameters.

        COMMAND_INT carries the current global position and an absolute AMSL
        target computed from the captured ground altitude.  The surrounding
        state machine owns ARM, ACK handling, timeout handling and monitoring.
        """

        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
                ("last_rc_channels_monotonic", "实体RC通道"),
                ("last_global_position_monotonic", "全球位置"),
            ),
        )
        height = float(target_height_m)
        if not 1.0 <= height <= 3.0:
            raise ValueError("指定起飞高度必须在1.0至3.0米之间")
        if telemetry.armed is not True or telemetry.landed_state != "ON_GROUND":
            raise ValueError("Takeoff只允许在飞控明确报告ARMED且ON_GROUND时发送")
        mode = (telemetry.flight_mode or "UNKNOWN").upper().replace(" ", "_")
        if mode not in _MANUAL_ARM_MODES:
            raise ValueError(f"Takeoff前必须保持Position/Altitude/Manual；当前为{mode}")
        if self._estop_latched():
            raise ValueError("本地安全处置仍处于锁存状态，拒绝Takeoff")
        if self._ground_sender_enabled():
            raise ValueError("Takeoff前必须关闭所有电脑Offboard设定值发送器")
        if telemetry.rc_channel_6_pwm is None or telemetry.rc_channel_6_pwm >= 1800:
            raise ValueError("实体Kill Switch（CH6）未确认解除，拒绝Takeoff")
        if (
            telemetry.rc_channel_8_pwm is None
            or telemetry.rc_channel_8_pwm >= self._physical_offboard_switch_pwm_min
        ):
            raise ValueError("实体CH8必须保持Position/手动辅助位置，拒绝Takeoff")
        values = (
            telemetry.latitude_deg,
            telemetry.longitude_deg,
            telemetry.global_altitude_amsl_m,
            ground_amsl_m,
        )
        if any(value is None or not math.isfinite(float(value)) for value in values):
            raise ValueError("缺少有效的全球位置/海拔数据，拒绝Takeoff")
        if abs(float(telemetry.latitude_deg)) > 90 or abs(float(telemetry.longitude_deg)) > 180:
            raise ValueError("飞控全球位置无效，拒绝Takeoff")
        if (
            abs(float(telemetry.latitude_deg)) < 1e-7
            and abs(float(telemetry.longitude_deg)) < 1e-7
        ):
            raise ValueError("飞控全球位置仍为0,0，拒绝Takeoff")

        return self._send_command_int_takeoff(
            telemetry,
            height,
            float(ground_amsl_m) + height,
        )

    def rtl(self) -> dict[str, Any]:
        telemetry = self._telemetry_snapshot()
        self._common_check(telemetry)
        self._require_fresh_fields(
            telemetry,
            (
                ("last_heartbeat_monotonic", "解锁状态"),
                ("last_extended_state_monotonic", "落地状态"),
            ),
        )
        if self._estop_latched():
            raise ValueError("网页安全处置仍处于锁存状态，拒绝发送RTL")
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
        allow_when_latched: bool = False,
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

            def transmit() -> None:
                sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sock.sendto(packet, (self.config.uplink_host, self.config.uplink_port))
                finally:
                    sock.close()

            if allow_when_latched:
                transmit()
            else:
                self._unlatched_action_guard(transmit)
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

    def _send_command_int_takeoff(
        self,
        telemetry: TelemetrySnapshot,
        target_height_m: float,
        target_amsl_m: float,
    ) -> dict[str, Any]:
        with self._lock:
            if (
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
            message = mav.command_int_encode(
                telemetry.system_id or self.config.target_system,
                telemetry.component_id or self.config.target_component,
                mavlink2.MAV_FRAME_GLOBAL_INT,
                mavlink2.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0.0,
                0.0,
                0.0,
                math.nan,
                int(round(float(telemetry.latitude_deg) * 10_000_000.0)),
                int(round(float(telemetry.longitude_deg) * 10_000_000.0)),
                target_amsl_m,
            )
            packet = message.pack(mav)

            def transmit() -> None:
                sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sock.sendto(packet, (self.config.uplink_host, self.config.uplink_port))
                finally:
                    sock.close()

            self._unlatched_action_guard(transmit)
        except OSError as exc:
            with self._lock:
                self._state.error = f"真实Takeoff命令发送失败：{exc}"
            raise ValueError(self._state.error) from exc

        with self._lock:
            self._state.commands_sent += 1
            self._state.last_action = f"TAKEOFF {target_height_m:.1f}m"
            self._state.last_command = 22
            self._state.last_sent_monotonic = monotonic()
            self._state.error = ""
            return self._state.to_dict()
