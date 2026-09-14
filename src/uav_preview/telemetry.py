from __future__ import annotations

from dataclasses import replace
from ipaddress import ip_address
from math import degrees, isfinite, isnan, sqrt
import socket
from threading import Event, Lock, Thread
from time import monotonic

from .config import TelemetryConfig
from .types import TelemetrySnapshot
from .navigation_health import estimator_ratio_reason
from .flow_health import FlowMonitor


def _estimator_ratio(value: object) -> tuple[float | None, str]:
    if value is None:
        return None, "missing"
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return None, "invalid"
    if not isfinite(numeric):
        return None, "nonfinite"
    return numeric, "invalid" if numeric < 0 else ("rejected" if numeric > 1 else "valid")


def _smoothed_rate(
    previous_hz: float | None,
    previous_time: float | None,
    now: float,
) -> float | None:
    if previous_time is None:
        return previous_hz
    period = now - previous_time
    if period <= 0.0 or period > 10.0:
        return previous_hz
    instantaneous_hz = min(100.0, 1.0 / period)
    if previous_hz is None:
        return instantaneous_hz
    return previous_hz * 0.65 + instantaneous_hz * 0.35


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
        self._flow = FlowMonitor(config.flow_minimum_quality, config.flow_sensor_id, config.flow_message_type)
        # Hardware's mavlink status identifies the internal sensor as 0/158.
        # If forwarded to the radio, inspect it without trusting it as FC state.
        self._peripheral_flow = FlowMonitor(config.flow_minimum_quality, config.flow_sensor_id, config.flow_message_type)
        self._snapshot = TelemetrySnapshot(
            flow_minimum_quality=config.flow_minimum_quality,
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
            result.flow_sources = self._flow.diagnostics(monotonic())
            result.flow_sources.update({key:{**value,'selected':False}
                for key,value in self._peripheral_flow.diagnostics(monotonic()).items()})
        result.connected = bool(
            result.last_heartbeat_monotonic is not None
            and monotonic() - result.last_heartbeat_monotonic
            <= self.config.stale_after_seconds
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
        # UDP telemetry can occasionally contain a truncated frame or a stray
        # byte (for example while the radio link is reconnecting).  With
        # pymavlink's default strict parser, one bad datagram leaves the parser
        # positioned inside that datagram and it may then reject one byte for
        # every later datagram.  In practice this makes the web telemetry look
        # permanently frozen even though the router is still receiving data.
        # Robust parsing skips bad bytes up to the next MAVLink v1/v2 marker and
        # lets the following valid frame update telemetry immediately.
        parser.robust_parsing = True
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
                    # Do not carry a partially decoded datagram into the next
                    # UDP packet.  Rebuilding the receive-only parser is safe:
                    # it has no output stream and cannot send anything to PX4.
                    parser = mavlink2.MAVLink(None)
                    parser.robust_parsing = True
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
            if (source_system,source_component)==(0,158) and message_type in {"OPTICAL_FLOW","OPTICAL_FLOW_RAD"}:
                # Diagnostic ONLY: no heartbeat, target ID, height, fusion or
                # navigation permission may be inferred from peripheral traffic.
                self._peripheral_flow.accept(message,now)
                return
            if source_component != 1 or (state.system_id is not None and source_system != state.system_id):
                return
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
                state.last_heartbeat_monotonic = now
            elif message_type == "ESTIMATOR_STATUS":
                sample_id = getattr(message, "time_usec", None)
                if type(sample_id) is not int or not 0 <= sample_id <= 0xFFFFFFFFFFFFFFFF:
                    return  # No source timestamp: cannot refresh safety evidence.
                previous_id = state.estimator_sample_id
                if previous_id is not None:
                    if sample_id == previous_id:
                        return  # Duplicates must not inflate frequency/freshness.
                    if sample_id < previous_id:
                        state.navigation_fault = "估计器源时间倒退或飞控重启，请重新核验定位"
                        return
                state.estimator_sample_id = sample_id
                state.estimator_hz = _smoothed_rate(state.estimator_hz, state.last_estimator_monotonic, now)
                state.estimator_flags = int(message.flags)
                state.estimator_velocity_ratio, state.estimator_velocity_ratio_status = _estimator_ratio(getattr(message, "vel_ratio", None))
                state.estimator_position_ratio, state.estimator_position_ratio_status = _estimator_ratio(getattr(message, "pos_horiz_ratio", None))
                raw_position_ratio = getattr(message, "pos_horiz_ratio", None)
                state.estimator_position_ratio_is_nan = isinstance(raw_position_ratio, (float, int)) and isnan(raw_position_ratio)
                state.last_estimator_monotonic = now
                if state.armed is True:
                    if state.estimator_flags & 15 != 15 or state.estimator_flags & (128 | 1024 | 2048):
                        state.navigation_fault = "飞行中估计器有效性异常，请飞手接管"
                    else:
                        for label, ratio in (("速度", state.estimator_velocity_ratio), ("水平位置", state.estimator_position_ratio)):
                            # Unavailable evidence is not an observed sensor failure.
                            # Strict navigation still refuses it in its own gate;
                            # fixed-hover checks PX4's relative estimate separately.
                            if ratio is None:
                                continue
                            reason = estimator_ratio_reason(ratio, label)
                            if reason:
                                state.navigation_fault = "飞行中" + reason + "，请飞手接管"
                                break
            elif message_type in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
                sample, error = self._flow.accept(message, now)
                if sample is not None:
                    state.flow_quality = sample.quality
                    state.flow_source = sample.source
                    state.flow_hz = self._flow.rate_hz
                    state.last_flow_monotonic = sample.received
                    state.flow_good_since_monotonic = self._flow.good_since
                    state.flow_good_samples = self._flow.good_samples
                    state.last_flow_bad_monotonic = self._flow.last_bad
                    state.flow_error = error
                elif error:
                    state.flow_error = error
                    state.flow_good_since_monotonic = None
                    state.flow_good_samples = 0
                    state.last_flow_bad_monotonic = now
                if state.armed is True and error:
                    state.navigation_fault = "飞行中" + error + "，撤销电脑导航，请飞手接管"
            elif message_type == "ATTITUDE":
                state.roll_deg = degrees(float(message.roll))
                state.pitch_deg = degrees(float(message.pitch))
                state.yaw_deg = degrees(float(message.yaw))
                state.last_attitude_monotonic = now
            elif message_type == "LOCAL_POSITION_NED":
                sample_id = getattr(message,"time_boot_ms",None)
                if type(sample_id) is int and 0 <= sample_id <= 0xFFFFFFFF:
                    old_id = state.local_position_sample_id
                    if old_id is not None:
                        advance = (sample_id-old_id) & 0xFFFFFFFF
                        if advance == 0:
                            return  # retransmission is not a new position observation
                        if advance >= 0x80000000:
                            state.navigation_fault = "本地位置源时间倒退或飞控重启，禁止沿用旧坐标"
                            return
                    state.local_position_sample_id = sample_id
                    state.last_distinct_position_monotonic = now
                else:
                    state.local_position_sample_id = None
                    state.last_distinct_position_monotonic = None
                previous = (state.local_x_m, state.local_y_m, state.local_z_m)
                incoming = (float(message.x), float(message.y), float(message.z))
                velocity = (float(message.vx), float(message.vy), float(message.vz))
                dt = None if state.last_local_position_monotonic is None else now - state.last_local_position_monotonic
                if state.armed is True:
                    if not all(isfinite(v) for v in incoming + velocity):
                        state.navigation_fault = "本地位置/速度出现非有限数值"
                    elif dt is not None and 0 < dt <= 0.5 and all(v is not None and isfinite(v) for v in previous):
                        # Heuristic discontinuity alarm, NOT an EKF reset counter.
                        residual = sqrt(sum((new - old - speed * dt) ** 2
                                            for new, old, speed in zip(incoming, previous, velocity)))
                        if residual > 0.20:
                            state.navigation_fault = "本地坐标疑似跳变（不等同已确认EKF重置），请接管"
                state.local_position_hz = _smoothed_rate(
                    state.local_position_hz,
                    state.last_local_position_monotonic,
                    now,
                )
                state.vx_m_s = float(message.vx)
                state.vy_m_s = float(message.vy)
                state.vz_m_s = float(message.vz)
                state.local_x_m = float(message.x)
                state.local_y_m = float(message.y)
                state.local_z_m = float(message.z)
                state.altitude_m = -state.local_z_m
                state.last_local_position_monotonic = now
            elif message_type == "GLOBAL_POSITION_INT":
                state.global_position_hz = _smoothed_rate(
                    state.global_position_hz,
                    state.last_global_position_monotonic,
                    now,
                )
                # Preserve local-NED velocity and its own freshness timestamp.
                state.latitude_deg = float(message.lat) / 10_000_000.0
                state.longitude_deg = float(message.lon) / 10_000_000.0
                state.global_altitude_amsl_m = float(message.alt) / 1000.0
                state.relative_altitude_m = float(message.relative_alt) / 1000.0
                state.altitude_m = state.relative_altitude_m
                state.last_global_position_monotonic = now
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
                sensor_id = getattr(message, "id", None)
                if orientation == downward and state.laser_sensor_id is not None and sensor_id != state.laser_sensor_id:
                    state.navigation_fault = "对地测距源身份变化，禁止自动替换高度来源"
                    return
                if orientation == downward and not 0 < current_cm < 65535:
                    state.last_laser_monotonic = None
                    if state.armed is True:
                        state.navigation_fault = "飞行中对地测距无效，请飞手接管"
                    return
                if orientation == downward and 0 < current_cm < 65535:
                    if type(sensor_id) is not int or not 0 <= sensor_id <= 255:
                        return
                    stamp = getattr(message, "time_boot_ms", None)
                    if type(stamp) is not int or not 0 <= stamp <= 0xFFFFFFFF:
                        return
                    if state.laser_sample_id is not None:
                        advance = (stamp-state.laser_sample_id)&0xFFFFFFFF
                        if advance == 0:
                            return
                        if advance >= 0x80000000:
                            state.navigation_fault = "测距源时间倒退，请核验飞控/传感器"
                            return
                    state.laser_sample_id = stamp
                    state.laser_hz = _smoothed_rate(state.laser_hz, state.last_laser_monotonic, now)
                    state.laser_height_m = current_cm / 100.0
                    state.laser_min_m = int(getattr(message, "min_distance", 0)) / 100.0
                    state.laser_max_m = int(getattr(message, "max_distance", 0)) / 100.0
                    state.laser_sensor_id = int(getattr(message, "id", 0))
                    state.last_laser_monotonic = now
            elif message_type == "RANGEFINDER":
                # This legacy message has neither sensor identity nor source
                # timestamp/range limits. It cannot refresh navigation evidence
                # or overwrite a pinned down-facing DISTANCE_SENSOR.
                pass
            elif message_type == "EXTENDED_SYS_STATE":
                state.extended_state_hz = _smoothed_rate(
                    state.extended_state_hz,
                    state.last_extended_state_monotonic,
                    now,
                )
                state.landed_state = {
                    0: "UNKNOWN",
                    1: "ON_GROUND",
                    2: "IN_AIR",
                    3: "TAKEOFF",
                    4: "LANDING",
                }.get(int(getattr(message, "landed_state", 0)), "UNKNOWN")
                state.last_extended_state_monotonic = now
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
                state.last_rc_channels_monotonic = now
            elif message_type == "STATUSTEXT":
                text = getattr(message, "text", "")
                if isinstance(text, bytes):
                    text = text.split(b"\0", 1)[0].decode("utf-8", errors="replace")
                state.last_status_text = str(text).split("\0", 1)[0]
                state.last_status_severity = int(getattr(message, "severity", 0))
                state.last_status_monotonic = now
                if state.armed is True and any(word in state.last_status_text.casefold() for word in
                    ("primary ekf changed", "filter fault", "invalid setpoints")):
                    state.navigation_fault = state.last_status_text
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
