from __future__ import annotations

from dataclasses import replace
from ipaddress import ip_address
from math import degrees, isfinite
import socket
from threading import Event, Lock, Thread
from time import monotonic

from .config import TelemetryConfig
from .types import TelemetrySnapshot


def _stick_percent(value: float | int | None) -> float | None:
    if value is None or value == 32767:
        return None
    return max(-100.0, min(100.0, float(value) / 10.0))


def _throttle_percent(value: float | int | None) -> float | None:
    if value is None or value == 32767:
        return None
    numeric = float(value)
    if numeric < 0:
        return max(0.0, min(100.0, (numeric + 1000.0) / 20.0))
    return max(0.0, min(100.0, numeric / 10.0))


def _rc_axis_percent(pwm: float | int | None) -> float | None:
    if pwm is None or int(pwm) in (0, 65535):
        return None
    return max(-100.0, min(100.0, (float(pwm) - 1500.0) / 5.0))


def _rc_throttle_percent(pwm: float | int | None) -> float | None:
    if pwm is None or int(pwm) in (0, 65535):
        return None
    return max(0.0, min(100.0, (float(pwm) - 1000.0) / 10.0))


def _parameter_name(value: object) -> str:
    if isinstance(value, bytes):
        return value.split(b"\0", 1)[0].decode("ascii", errors="ignore")
    return str(value).split("\0", 1)[0]


def _flight_mode_string(message: object, mavutil: object) -> str:
    """Decode PX4 custom mode even when pymavlink returns UNKNOWN.

    Some pymavlink releases only recognize PX4 OFFBOARD when AUTO flag bits
    are also set in base_mode. PX4 legitimately reports OFFBOARD as custom
    main mode 6 with base_mode 0x11 while disarmed, so decode the PX4 custom
    main/sub-mode fields directly when the library cannot name the mode.
    """

    try:
        decoded = str(mavutil.mode_string_v10(message))
    except Exception:
        decoded = "UNKNOWN"
    if decoded.upper() != "UNKNOWN":
        return decoded

    mavlink = getattr(mavutil, "mavlink", object())
    px4_autopilot = int(getattr(mavlink, "MAV_AUTOPILOT_PX4", 12))
    if int(getattr(message, "autopilot", -1)) != px4_autopilot:
        return decoded

    custom_mode = int(getattr(message, "custom_mode", 0))
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    main_modes = {
        1: "MANUAL",
        2: "ALTCTL",
        3: "POSCTL",
        5: "ACRO",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE",
    }
    if main_mode == 4:  # PX4_CUSTOM_MAIN_MODE_AUTO
        return {
            1: "AUTO_READY",
            2: "TAKEOFF",
            3: "LOITER",
            4: "MISSION",
            5: "RTL",
            6: "LAND",
            7: "RTGS",
            8: "FOLLOWME",
        }.get(sub_mode, f"AUTO_{sub_mode}")
    return main_modes.get(main_mode, decoded)


class PassiveMavlinkReceiver:
    """Receive-only MAVLink parser with an optional local QGC telemetry tee.

    The only network output allowed here is a byte-for-byte copy of an accepted
    downlink datagram to a numeric loopback address. There is no socket or route
    from QGC back to the vehicle, and the parser has no MAVLink output stream.
    """

    def __init__(self, config: TelemetryConfig) -> None:
        self.config = config
        self._snapshot = TelemetrySnapshot(
            qgc_forward_enabled=config.forward_qgc,
            qgc_forward_endpoint=(
                f"{config.qgc_host}:{config.qgc_port}" if config.forward_qgc else ""
            ),
        )
        self._lock = Lock()
        self._stop = Event()
        self._ready = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if not self.config.enabled or self._thread is not None:
            return
        self._thread = Thread(target=self._run, name="passive-mavlink", daemon=True)
        self._thread.start()
        # Do not let the caller assume the UDP port is ready while pymavlink is
        # still importing in the worker thread. Otherwise the first datagram can
        # be lost on slower machines or immediately after a dependency update.
        self._ready.wait(timeout=5.0)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> TelemetrySnapshot:
        with self._lock:
            result = replace(self._snapshot)
        result.connected = bool(
            result.last_packet_monotonic is not None
            and result.age_seconds() is not None
            and result.age_seconds() <= self.config.stale_after_seconds
        )
        return result

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._snapshot.error = message
        self._ready.set()

    def _run(self) -> None:
        try:
            from pymavlink import mavutil
            from pymavlink.dialects.v20 import common as mavlink2
        except Exception as exc:
            self._set_error(f"pymavlink 未安装或加载失败：{exc}")
            return

        try:
            qgc_address = ip_address(self.config.qgc_host)
        except ValueError:
            self._set_error("QGC 分流目标必须是数字形式的本机回环地址")
            return
        if self.config.forward_qgc and not qgc_address.is_loopback:
            self._set_error("安全拒绝：QGC 分流只允许发送到本机回环地址")
            return

        parser = mavlink2.MAVLink(None)
        allowed_sources = set(self.config.allowed_source_ips)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc_sock: socket.socket | None = None
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.config.bind_host, self.config.bind_port))
            sock.settimeout(0.5)
        except OSError as exc:
            sock.close()
            self._set_error(
                f"无法监听 UDP {self.config.bind_host}:{self.config.bind_port}：{exc}。"
                "请关闭占用该端口的 QGC，或改用本地 MAVLink 分流端口。"
            )
            return

        self._ready.set()

        if self.config.forward_qgc:
            qgc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            while not self._stop.is_set():
                try:
                    packet, address = sock.recvfrom(65535)
                except TimeoutError:
                    continue
                except OSError as exc:
                    self._set_error(f"遥测接收失败：{exc}")
                    continue

                with self._lock:
                    self._snapshot.received_datagrams += 1

                if allowed_sources and address[0] not in allowed_sources:
                    with self._lock:
                        self._snapshot.dropped_datagrams += 1
                    continue

                if qgc_sock is not None:
                    try:
                        # The destination is validated as loopback above. This is a
                        # one-way telemetry copy, never a vehicle-control uplink.
                        qgc_sock.sendto(packet, (self.config.qgc_host, self.config.qgc_port))
                        with self._lock:
                            self._snapshot.forwarded_datagrams += 1
                    except OSError as exc:
                        self._set_error(f"向本机 QGC 分流遥测失败：{exc}")

                try:
                    messages = parser.parse_buffer(packet) or []
                except Exception as exc:
                    self._set_error(f"MAVLink 解析失败：{exc}")
                    continue
                for message in messages:
                    self._accept(message, address[0], mavutil)
        finally:
            sock.close()
            if qgc_sock is not None:
                qgc_sock.close()

    def _accept(self, message: object, source_ip: str, mavutil: object) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return
        now = monotonic()
        source_system = message.get_srcSystem()
        source_component = message.get_srcComponent()
        with self._lock:
            state = self._snapshot
            state.last_packet_monotonic = now
            state.packets += 1
            state.source = source_ip
            state.error = ""

            if message_type == "HEARTBEAT":
                # A QGC-forwarded stream can contain heartbeats from cameras,
                # gimbals or a GCS. Only component 1 is the flight controller;
                # other heartbeats must never overwrite PX4 mode/armed state.
                autopilot_component = int(
                    getattr(getattr(mavutil, "mavlink", object()), "MAV_COMP_ID_AUTOPILOT1", 1)
                )
                if source_component != autopilot_component:
                    return
                state.system_id = source_system
                state.component_id = source_component
                state.flight_mode = _flight_mode_string(message, mavutil)
                state.armed = bool(getattr(message, "base_mode", 0) & 128)
            elif message_type == "ATTITUDE":
                state.roll_deg = degrees(float(message.roll))
                state.pitch_deg = degrees(float(message.pitch))
                state.yaw_deg = degrees(float(message.yaw))
            elif message_type == "LOCAL_POSITION_NED":
                state.vx_m_s = float(message.vx)
                state.vy_m_s = float(message.vy)
                state.vz_m_s = float(message.vz)
                state.altitude_m = -float(message.z)
            elif message_type == "GLOBAL_POSITION_INT":
                state.vx_m_s = float(message.vx) / 100.0
                state.vy_m_s = float(message.vy) / 100.0
                state.vz_m_s = float(message.vz) / 100.0
                state.altitude_m = float(message.relative_alt) / 1000.0
            elif message_type == "DISTANCE_SENSOR":
                # Only a downward-facing sensor is a height-above-ground
                # measurement. Horizontal obstacle sensors must not be shown
                # as aircraft height.
                downward = int(
                    getattr(
                        getattr(mavutil, "mavlink", object()),
                        "MAV_SENSOR_ROTATION_PITCH_270",
                        25,
                    )
                )
                orientation = int(getattr(message, "orientation", -1))
                current_cm = int(getattr(message, "current_distance", 0))
                if orientation == downward and 0 < current_cm < 65535:
                    state.laser_height_m = current_cm / 100.0
                    state.laser_min_m = int(getattr(message, "min_distance", 0)) / 100.0
                    state.laser_max_m = int(getattr(message, "max_distance", 0)) / 100.0
                    state.laser_sensor_id = int(getattr(message, "id", 0))
                    state.last_laser_monotonic = now
            elif message_type == "RANGEFINDER":
                # Compatibility path for older flight stacks that publish the
                # dedicated RANGEFINDER message instead of DISTANCE_SENSOR.
                distance_m = float(getattr(message, "distance", float("nan")))
                if isfinite(distance_m) and distance_m >= 0.0:
                    state.laser_height_m = distance_m
                    state.last_laser_monotonic = now
            elif message_type == "EXTENDED_SYS_STATE":
                state.landed_state = {
                    0: "UNKNOWN",
                    1: "ON_GROUND",
                    2: "IN_AIR",
                    3: "TAKEOFF",
                    4: "LANDING",
                }.get(int(getattr(message, "landed_state", 0)), "UNKNOWN")
            elif message_type == "VFR_HUD":
                state.throttle_pct = float(message.throttle)
                if state.altitude_m is None:
                    state.altitude_m = float(message.alt)
            elif message_type == "MANUAL_CONTROL":
                state.stick_pitch_pct = _stick_percent(getattr(message, "x", None))
                state.stick_roll_pct = _stick_percent(getattr(message, "y", None))
                state.stick_throttle_pct = _throttle_percent(getattr(message, "z", None))
                state.stick_yaw_pct = _stick_percent(getattr(message, "r", None))
            elif message_type == "RC_CHANNELS":
                # Standard Mode-2 display convention only. This reads raw channel values
                # and does not inspect or modify the PX4 channel mapping parameters.
                state.stick_roll_pct = _rc_axis_percent(getattr(message, "chan1_raw", None))
                state.stick_pitch_pct = _rc_axis_percent(getattr(message, "chan2_raw", None))
                state.stick_throttle_pct = _rc_throttle_percent(getattr(message, "chan3_raw", None))
                state.stick_yaw_pct = _rc_axis_percent(getattr(message, "chan4_raw", None))
                state.rc_channel_5_pwm = int(getattr(message, "chan5_raw", 0)) or None
                state.rc_channel_6_pwm = int(getattr(message, "chan6_raw", 0)) or None
                state.rc_channel_7_pwm = int(getattr(message, "chan7_raw", 0)) or None
                state.rc_channel_8_pwm = int(getattr(message, "chan8_raw", 0)) or None
            elif message_type == "STATUSTEXT":
                text = getattr(message, "text", "")
                if isinstance(text, bytes):
                    text = text.split(b"\0", 1)[0].decode("utf-8", errors="replace")
                state.last_status_text = str(text).split("\0", 1)[0]
                state.last_status_severity = int(getattr(message, "severity", 0))
                state.last_status_monotonic = now
            elif message_type == "COMMAND_ACK":
                state.last_command_ack_command = int(getattr(message, "command", 0))
                state.last_command_ack_result = int(getattr(message, "result", 0))
                progress = int(getattr(message, "progress", 255))
                state.last_command_ack_progress = None if progress == 255 else progress
                state.last_command_ack_monotonic = now
            elif message_type == "PARAM_VALUE":
                name = _parameter_name(getattr(message, "param_id", ""))
                parameter_fields = {
                    "RC_MAP_OFFB_SW": "rc_map_offb_sw",
                    "RC_MAP_MODE_SW": "rc_map_mode_sw",
                    "COM_FLTMODE1": "com_fltmode1",
                    "COM_FLTMODE2": "com_fltmode2",
                    "COM_FLTMODE3": "com_fltmode3",
                    "COM_FLTMODE4": "com_fltmode4",
                    "COM_FLTMODE5": "com_fltmode5",
                    "COM_FLTMODE6": "com_fltmode6",
                }
                field_name = parameter_fields.get(name)
                if field_name is not None:
                    raw_value = float(getattr(message, "param_value", 0.0))
                    # Some PX4/QGC parameter streams contain NaN placeholders.
                    # They are not valid channel mappings and must not terminate
                    # the receive-only telemetry thread.
                    if isfinite(raw_value):
                        setattr(state, field_name, int(round(raw_value)))
