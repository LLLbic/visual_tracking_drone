from __future__ import annotations

from dataclasses import dataclass, fields
from ipaddress import ip_address
from math import isfinite
from pathlib import Path
from typing import Any, TypeVar
import tomllib


@dataclass(slots=True)
class VideoConfig:
    source: str = "0"
    transport: str = "tcp"
    capture_backend: str = "opencv"
    ffmpeg_path: str = "ffmpeg"
    ffmpeg_decoder: str = "software"
    frame_width: int = 640
    frame_height: int = 360
    reconnect_seconds: float = 2.0
    jpeg_quality: int = 82
    low_latency_latest_frame: bool = True
    udp_buffer_size_bytes: int = 1_048_576
    udp_reorder_queue_size: int = 32
    udp_max_delay_ms: int = 100
    udp_read_timeout_ms: int = 1_000
    udp_failure_grace_seconds: float = 2.5


@dataclass(slots=True)
class FrameInterpolationConfig:
    """Experimental NVIDIA FRUC stage between capture and detection."""

    enabled: bool = False
    backend: str = "nvidia_fruc"
    sdk_root: str = "D:/NVToolKits/Optical_Flow_SDK_5.0.7/Optical_Flow_SDK_5.0.7"
    native_library: str = ".runtime/nvof-fruc/nvof_fruc_bridge.dll"
    device_id: int = 0
    multiplier: int = 2
    yolo_on_synthetic: bool = False


@dataclass(slots=True)
class VisionConfig:
    enabled: bool = True
    model_path: str = "yolo11n.pt"
    active_model: str = "default"
    device: str = "cpu"
    target_class_names: list[str] | None = None
    target_class_ids: list[int] | None = None
    confidence: float = 0.35
    iou: float = 0.55
    detector_every_n_frames: int = 3
    multi_object_tracker: str = "bytetrack.yaml"
    single_object_tracker: str = "csrt"
    dasiamrpn_model: str = ""
    dasiamrpn_kernel_cls1: str = ""
    dasiamrpn_kernel_r1: str = ""
    reacquire_max_center_distance: float = 0.22
    reacquire_max_height_change: float = 0.45
    max_target_lost_frames: int = 12

    def __post_init__(self) -> None:
        if self.target_class_names is None:
            self.target_class_names = ["person"]
        if self.target_class_ids is None:
            self.target_class_ids = []


@dataclass(slots=True)
class VisionModelProfile:
    id: str
    label: str
    model_path: str
    kind: str = "yolo"
    prompts: list[str] | None = None
    target_class_names: list[str] | None = None
    target_class_ids: list[int] | None = None
    red_color_filter: bool = False
    min_red_ratio: float = 0.08

    def __post_init__(self) -> None:
        if self.prompts is None:
            self.prompts = []
        if self.target_class_names is None:
            self.target_class_names = []
        if self.target_class_ids is None:
            self.target_class_ids = []


@dataclass(slots=True)
class TelemetryConfig:
    enabled: bool = True
    bind_host: str = "0.0.0.0"
    bind_port: int = 8080
    stale_after_seconds: float = 1.5
    allowed_source_ips: list[str] | None = None
    forward_qgc: bool = False
    qgc_host: str = "127.0.0.1"
    qgc_port: int = 14550
    flow_message_type: str = "OPTICAL_FLOW_RAD"
    flow_sensor_id: int = 0
    # Some radio/camera adapters populate MAVLink ``quality`` with a constant
    # placeholder.  It must be explicitly proven authoritative before it may
    # participate in a control gate.
    flow_quality_authoritative: bool = False
    flow_minimum_quality: int = 100  # Used only when the source is authoritative.

    def __post_init__(self) -> None:
        if self.allowed_source_ips is None:
            self.allowed_source_ips = []


@dataclass(slots=True)
class PositionStreamConfig:
    """Non-persistent MAVLink telemetry-rate requests made only on the ground."""

    enabled: bool = True
    local_position_hz: float = 5.0
    global_position_hz: float = 5.0
    extended_state_hz: float = 2.0
    estimator_hz: float = 5.0
    request_flow_range: bool = False
    flow_hz: float = 5.0
    range_hz: float = 5.0
    require_global_position: bool = True
    minimum_effective_hz: float = 2.5
    minimum_extended_state_hz: float = 1.0
    retry_seconds: float = 3.0
    max_attempts: int = 3
    uplink_host: str = "127.0.0.1"
    uplink_port: int = 14560
    target_system: int = 1
    target_component: int = 1
    source_system: int = 245
    source_component: int = 191


@dataclass(slots=True)
class MavlinkRouterConfig:
    """Computer-local bidirectional UDP router for the MiniHomer link.

    The vehicle-facing socket owns the one port accepted by the radio. Both
    QGC and the preview application live on loopback-only fan-out ports.
    """

    enabled: bool = False
    vehicle_bind_host: str = "0.0.0.0"
    vehicle_bind_port: int = 8080
    vehicle_host: str = "192.168.1.201"
    vehicle_port: int = 8080
    local_host: str = "127.0.0.1"
    local_ingress_port: int = 14560
    qgc_host: str = "127.0.0.1"
    qgc_port: int = 14551
    telemetry_host: str = "127.0.0.1"
    telemetry_port: int = 14550
    # Keep the receiver-connected RC as the only manual-control source. QGC
    # may still receive telemetry and issue explicit high-level actions such as
    # mode/arm/takeoff/land/RTL/brake, but virtual joystick and RC override never
    # reach PX4 or compete with the receiver-connected RC sticks.
    enforce_rc_priority: bool = True
    # Aircraft-originated frames reflected by QGC into local_ingress must not
    # be sent back to the radio, or they form a telemetry amplification loop.
    vehicle_system_id: int = 1
    approved_setpoint_system: int = 245
    approved_setpoint_component: int = 191


@dataclass(slots=True)
class ControlConfig:
    transmit_enabled: bool = False
    target_bbox_height_ratio: float = 0.42
    yaw_deadband: float = 0.04
    distance_deadband: float = 0.025
    yaw_kp: float = 36.0
    yaw_ki: float = 0.0
    yaw_kd: float = 4.0
    distance_kp: float = 1.35
    distance_ki: float = 0.0
    distance_kd: float = 0.12
    max_yaw_rate_deg_s: float = 28.0
    max_forward_speed_m_s: float = 1.0
    integral_limit: float = 0.5


@dataclass(slots=True)
class GroundOffboardTestConfig:
    """Narrow, manually enabled ground-test uplink.

    This channel is intentionally separate from the vision controller. It can
    only publish zero-velocity SET_POSITION_TARGET_LOCAL_NED messages and can
    never arm the vehicle or request a flight-mode change.
    """

    available: bool = False
    frequency_hz: float = 5.0
    arm_ready_after_seconds: float = 1.2
    uplink_host: str = "127.0.0.1"
    uplink_port: int = 14560
    target_system: int = 1
    target_component: int = 1
    source_system: int = 245
    source_component: int = 191


@dataclass(slots=True)
class KeyboardControlConfig:
    """Guarded body-frame velocity sender driven by the browser keyboard."""

    available: bool = False
    # Ground takeoff from the browser is deliberately forbidden.  Take off
    # with the physical RC in a manual assisted mode, then let the pilot move
    # the physical mode switch to Offboard before enabling this sender.
    allow_ground_takeoff: bool = False
    require_physical_offboard_switch: bool = True
    physical_offboard_switch_pwm_min: int = 1800
    frequency_hz: float = 10.0
    input_timeout_seconds: float = 0.4
    auto_stop_seconds: float = 1.5
    max_horizontal_speed_m_s: float = 0.25
    max_vertical_speed_m_s: float = 0.20
    max_yaw_rate_deg_s: float = 10.0


@dataclass(slots=True)
class TakeoffConfig:
    """One-shot, telemetry-guarded PX4 native takeoff state machine."""

    available: bool = False
    min_height_m: float = 1.0
    max_height_m: float = 3.0
    default_height_m: float = 1.5
    preflight_stable_seconds: float = 2.0
    arm_ack_timeout_seconds: float = 3.0
    arm_to_takeoff_delay_seconds: float = 1.0
    takeoff_ack_timeout_seconds: float = 3.0
    liftoff_timeout_seconds: float = 6.0
    climb_timeout_seconds: float = 20.0
    hover_stable_seconds: float = 2.0
    height_tolerance_m: float = 0.15
    vertical_speed_tolerance_m_s: float = 0.20
    liftoff_height_m: float = 0.15
    preflight_max_tilt_deg: float = 10.0
    max_tilt_deg: float = 20.0
    max_horizontal_drift_m: float = 1.0
    position_stale_seconds: float = 1.0


@dataclass(slots=True)
class LocalOffboardTakeoffConfig:
    """Local-NED Offboard takeoff that starts from the captured ground pose."""

    available: bool = False
    # Takeoff/hover validation comes first; handoff requires explicit offline opt-in.
    keyboard_handoff_enabled: bool = False
    navigation_profile: str = "detailed"
    min_height_m: float = 1.0
    max_height_m: float = 3.0
    default_height_m: float = 1.5
    frequency_hz: float = 10.0
    prestream_seconds: float = 1.5
    offboard_wait_timeout_seconds: float = 60.0
    arm_ack_timeout_seconds: float = 3.0
    liftoff_timeout_seconds: float = 8.0
    climb_timeout_seconds: float = 20.0
    climb_rate_m_s: float = 0.30
    # The scheduled climb target may never run farther ahead of measured
    # relative height than this.  It bounds position-controller demand if the
    # aircraft does not climb as quickly as the time ramp expects.
    max_climb_setpoint_lead_m: float = 0.25
    hover_stable_seconds: float = 2.0
    height_tolerance_m: float = 0.15
    vertical_speed_tolerance_m_s: float = 0.20
    liftoff_height_m: float = 0.15
    preflight_max_tilt_deg: float = 10.0
    max_tilt_deg: float = 20.0
    max_horizontal_drift_m: float = 1.0
    position_stale_seconds: float = 1.0


@dataclass(slots=True)
class ServerConfig:
    host: str = "127.0.0.1"
    port: int = 8765


@dataclass(slots=True)
class AppConfig:
    video: VideoConfig
    frame_interpolation: FrameInterpolationConfig
    vision: VisionConfig
    vision_models: dict[str, VisionModelProfile]
    mavlink_router: MavlinkRouterConfig
    telemetry: TelemetryConfig
    position_stream: PositionStreamConfig
    control: ControlConfig
    ground_offboard_test: GroundOffboardTestConfig
    keyboard_control: KeyboardControlConfig
    takeoff: TakeoffConfig
    local_offboard_takeoff: LocalOffboardTakeoffConfig
    server: ServerConfig


T = TypeVar("T")


def _load_section(cls: type[T], raw: dict[str, Any]) -> T:
    allowed = {field.name for field in fields(cls)}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"Unknown keys in [{cls.__name__}]: {', '.join(unknown)}")
    return cls(**raw)


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("rb") as handle:
        raw = tomllib.load(handle)

    video = _load_section(VideoConfig, raw.get("video", {}))
    video.transport = video.transport.casefold()
    if video.transport not in {"tcp", "udp"}:
        raise ValueError("video.transport must be either tcp or udp")
    video.capture_backend = video.capture_backend.casefold()
    if video.capture_backend not in {"opencv", "ffmpeg"}:
        raise ValueError("video.capture_backend must be either opencv or ffmpeg")
    video.ffmpeg_decoder = video.ffmpeg_decoder.casefold()
    if video.ffmpeg_decoder not in {"software", "hevc_cuvid", "h264_cuvid"}:
        raise ValueError(
            "video.ffmpeg_decoder must be software, hevc_cuvid, or h264_cuvid"
        )
    if not 16 <= video.frame_width <= 7_680 or not 16 <= video.frame_height <= 4_320:
        raise ValueError("video.frame_width/frame_height are outside the supported range")
    if not 65_536 <= video.udp_buffer_size_bytes <= 16_777_216:
        raise ValueError("video.udp_buffer_size_bytes must be between 65536 and 16777216")
    if not 0 <= video.udp_reorder_queue_size <= 512:
        raise ValueError("video.udp_reorder_queue_size must be between 0 and 512")
    if not 0 <= video.udp_max_delay_ms <= 1_000:
        raise ValueError("video.udp_max_delay_ms must be between 0 and 1000")
    if not 250 <= video.udp_read_timeout_ms <= 30_000:
        raise ValueError("video.udp_read_timeout_ms must be between 250 and 30000")
    if not 0.0 <= video.udp_failure_grace_seconds <= 30.0:
        raise ValueError("video.udp_failure_grace_seconds must be between 0 and 30")

    interpolation = _load_section(
        FrameInterpolationConfig, raw.get("frame_interpolation", {})
    )
    interpolation.backend = interpolation.backend.casefold()
    if interpolation.backend != "nvidia_fruc":
        raise ValueError("frame_interpolation.backend must be nvidia_fruc")
    if interpolation.multiplier != 2:
        raise ValueError("NVIDIA FRUC experiment currently supports multiplier=2 only")
    if interpolation.device_id < 0:
        raise ValueError("frame_interpolation.device_id must be non-negative")
    if interpolation.yolo_on_synthetic:
        raise ValueError(
            "Live YOLO/control on synthetic frames is forbidden; use the offline A/B benchmark"
        )

    control = _load_section(ControlConfig, raw.get("control", {}))
    if control.transmit_enabled:
        raise ValueError(
            "This is a passive-only build. [control].transmit_enabled must remain false."
        )

    vision = _load_section(VisionConfig, raw.get("vision", {}))
    if vision.detector_every_n_frames < 1:
        raise ValueError("detector_every_n_frames must be at least 1")
    if not 0.0 < control.target_bbox_height_ratio < 1.0:
        raise ValueError("target_bbox_height_ratio must be between 0 and 1")

    raw_profiles = raw.get("vision_models", {})
    if raw_profiles and not isinstance(raw_profiles, dict):
        raise ValueError("[vision_models] must contain named model tables")
    vision_models: dict[str, VisionModelProfile] = {}
    profile_fields = {field.name for field in fields(VisionModelProfile)} - {"id"}
    for profile_id, profile_raw in raw_profiles.items():
        if not isinstance(profile_raw, dict):
            raise ValueError(f"[vision_models.{profile_id}] must be a table")
        unknown = sorted(set(profile_raw) - profile_fields)
        if unknown:
            raise ValueError(
                f"Unknown keys in [vision_models.{profile_id}]: {', '.join(unknown)}"
            )
        profile = VisionModelProfile(id=profile_id, **profile_raw)
        profile.kind = profile.kind.casefold()
        if profile.kind not in {"yolo", "yoloe"}:
            raise ValueError(f"vision model {profile_id} kind must be yolo or yoloe")
        if profile.kind == "yoloe" and not profile.prompts:
            raise ValueError(f"vision model {profile_id} requires at least one text prompt")
        if not 0.0 <= profile.min_red_ratio <= 1.0:
            raise ValueError(f"vision model {profile_id} min_red_ratio must be between 0 and 1")
        vision_models[profile_id] = profile
    if not vision_models:
        vision_models["default"] = VisionModelProfile(
            id="default",
            label="默认 YOLO 模型",
            model_path=vision.model_path,
            target_class_names=list(vision.target_class_names or []),
            target_class_ids=list(vision.target_class_ids or []),
        )
    if vision.active_model not in vision_models:
        raise ValueError(
            f"vision.active_model '{vision.active_model}' is not defined in [vision_models]"
        )

    telemetry = _load_section(TelemetryConfig, raw.get("telemetry", {}))
    if not 1 <= telemetry.bind_port <= 65535:
        raise ValueError("telemetry.bind_port must be between 1 and 65535")
    if not 1 <= telemetry.qgc_port <= 65535:
        raise ValueError("telemetry.qgc_port must be between 1 and 65535")
    try:
        qgc_address = ip_address(telemetry.qgc_host)
    except ValueError as exc:
        raise ValueError("telemetry.qgc_host must be a numeric loopback IP address") from exc
    if not qgc_address.is_loopback:
        raise ValueError(
            "telemetry.qgc_host must be loopback-only; forwarding to the aircraft is forbidden"
        )
    for source_ip in telemetry.allowed_source_ips:
        try:
            ip_address(source_ip)
        except ValueError as exc:
            raise ValueError(f"Invalid telemetry.allowed_source_ips entry: {source_ip}") from exc
    if telemetry.forward_qgc and not telemetry.allowed_source_ips:
        raise ValueError(
            "telemetry.allowed_source_ips must not be empty when QGC forwarding is enabled"
        )

    router = _load_section(MavlinkRouterConfig, raw.get("mavlink_router", {}))
    for field_name in (
        "vehicle_bind_port",
        "vehicle_port",
        "local_ingress_port",
        "qgc_port",
        "telemetry_port",
    ):
        value = getattr(router, field_name)
        if not 1 <= value <= 65535:
            raise ValueError(f"mavlink_router.{field_name} must be between 1 and 65535")
    try:
        vehicle_address = ip_address(router.vehicle_host)
    except ValueError as exc:
        raise ValueError("mavlink_router.vehicle_host must be a numeric IPv4 address") from exc
    if vehicle_address.version != 4 or vehicle_address.is_multicast or vehicle_address.is_unspecified:
        raise ValueError("mavlink_router.vehicle_host must be a unicast IPv4 address")
    for field_name in ("local_host", "qgc_host", "telemetry_host"):
        try:
            address = ip_address(getattr(router, field_name))
        except ValueError as exc:
            raise ValueError(f"mavlink_router.{field_name} must be a numeric loopback IP") from exc
        if not address.is_loopback:
            raise ValueError(f"mavlink_router.{field_name} must be loopback-only")
    if len({router.local_ingress_port, router.qgc_port, router.telemetry_port}) != 3:
        raise ValueError("mavlink_router local ingress, QGC, and telemetry ports must be distinct")
    for field_name in (
        "vehicle_system_id",
        "approved_setpoint_system",
        "approved_setpoint_component",
    ):
        value = getattr(router, field_name)
        if not 1 <= value <= 255:
            raise ValueError(f"mavlink_router.{field_name} must be between 1 and 255")
    if router.enabled:
        if telemetry.bind_host not in {router.telemetry_host, "0.0.0.0"}:
            raise ValueError("telemetry.bind_host must receive the router loopback output")
        if telemetry.bind_port != router.telemetry_port:
            raise ValueError("telemetry.bind_port must match mavlink_router.telemetry_port")

    position_stream = _load_section(
        PositionStreamConfig, raw.get("position_stream", {})
    )
    if telemetry.flow_message_type not in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
        raise ValueError("telemetry.flow_message_type must be a known raw flow message")
    if type(telemetry.flow_sensor_id) is not int or not 0 <= telemetry.flow_sensor_id <= 255:
        raise ValueError("telemetry.flow_sensor_id must be a uint8")
    if type(telemetry.flow_quality_authoritative) is not bool:
        raise ValueError("telemetry.flow_quality_authoritative must be a boolean")
    if type(telemetry.flow_minimum_quality) is not int or not 1 <= telemetry.flow_minimum_quality <= 255:
        raise ValueError("telemetry.flow_minimum_quality must be within 1..255")
    if not 1 <= position_stream.uplink_port <= 65535:
        raise ValueError("position_stream.uplink_port must be between 1 and 65535")
    try:
        stream_uplink = ip_address(position_stream.uplink_host)
    except ValueError as exc:
        raise ValueError("position_stream.uplink_host must be a numeric loopback IP") from exc
    if not stream_uplink.is_loopback:
        raise ValueError("position_stream.uplink_host must be loopback-only")
    for field_name in (
        "target_system",
        "target_component",
        "source_system",
        "source_component",
    ):
        value = getattr(position_stream, field_name)
        if not 1 <= value <= 255:
            raise ValueError(f"position_stream.{field_name} must be between 1 and 255")
    for field_name in (
        "local_position_hz",
        "global_position_hz",
        "extended_state_hz",
        "estimator_hz",
        "flow_hz",
        "range_hz",
        "minimum_effective_hz",
        "minimum_extended_state_hz",
        "retry_seconds",
    ):
        if not isfinite(getattr(position_stream, field_name)) or getattr(position_stream, field_name) <= 0:
            raise ValueError(f"position_stream.{field_name} must be positive")
    if (
        position_stream.local_position_hz > 20
        or position_stream.global_position_hz > 20
        or position_stream.extended_state_hz > 20
        or position_stream.estimator_hz > 20
        or position_stream.flow_hz > 20
        or position_stream.range_hz > 20
    ):
        raise ValueError("position_stream requested rates must not exceed 20 Hz")
    if position_stream.minimum_effective_hz > min(
        position_stream.local_position_hz, position_stream.global_position_hz, position_stream.estimator_hz
    ):
        raise ValueError("position_stream.minimum_effective_hz exceeds a requested rate")
    if position_stream.minimum_extended_state_hz > position_stream.extended_state_hz:
        raise ValueError(
            "position_stream.minimum_extended_state_hz exceeds the requested rate"
        )
    if not 1 <= position_stream.max_attempts <= 5:
        raise ValueError("position_stream.max_attempts must be between 1 and 5")
    if type(position_stream.require_global_position) is not bool:
        raise ValueError("position_stream.require_global_position must be boolean")
    if type(position_stream.request_flow_range) is not bool:
        raise ValueError("position_stream.request_flow_range must be boolean")
    if position_stream.request_flow_range and position_stream.minimum_effective_hz > min(position_stream.flow_hz,position_stream.range_hz):
        raise ValueError("flow/range requested rates must meet minimum_effective_hz")
    if router.enabled:
        if position_stream.uplink_host != router.local_host:
            raise ValueError("position_stream must use the router loopback host")
        if position_stream.uplink_port != router.local_ingress_port:
            raise ValueError("position_stream must use the router local ingress port")

    ground_test = _load_section(
        GroundOffboardTestConfig, raw.get("ground_offboard_test", {})
    )
    if abs(ground_test.frequency_hz - 5.0) > 1e-9:
        raise ValueError("ground_offboard_test.frequency_hz must remain exactly 5.0 Hz")
    if not 1.0 <= ground_test.arm_ready_after_seconds <= 5.0:
        raise ValueError(
            "ground_offboard_test.arm_ready_after_seconds must be between 1 and 5 seconds"
        )
    if not 1 <= ground_test.uplink_port <= 65535:
        raise ValueError("ground_offboard_test.uplink_port must be between 1 and 65535")
    try:
        uplink_address = ip_address(ground_test.uplink_host)
    except ValueError as exc:
        raise ValueError("ground_offboard_test.uplink_host must be a numeric IPv4 address") from exc
    if uplink_address.version != 4 or uplink_address.is_multicast or uplink_address.is_unspecified:
        raise ValueError("ground_offboard_test.uplink_host must be a unicast IPv4 address")
    for field_name in (
        "target_system",
        "target_component",
        "source_system",
        "source_component",
    ):
        value = getattr(ground_test, field_name)
        if not 1 <= value <= 255:
            raise ValueError(f"ground_offboard_test.{field_name} must be between 1 and 255")
    if router.enforce_rc_priority and (
        router.approved_setpoint_system != ground_test.source_system
        or router.approved_setpoint_component != ground_test.source_component
    ):
        raise ValueError(
            "mavlink_router approved setpoint identity must match ground_offboard_test source identity"
        )

    keyboard = _load_section(KeyboardControlConfig, raw.get("keyboard_control", {}))
    if keyboard.allow_ground_takeoff:
        raise ValueError(
            "keyboard_control.allow_ground_takeoff must remain false; "
            "computer takeoff from the ground is disabled for safety"
        )
    if not 1600 <= keyboard.physical_offboard_switch_pwm_min <= 2100:
        raise ValueError(
            "keyboard_control.physical_offboard_switch_pwm_min must be between 1600 and 2100"
        )
    if not 2.0 <= keyboard.frequency_hz <= 50.0:
        raise ValueError("keyboard_control.frequency_hz must be between 2 and 50 Hz")
    if not 0.1 <= keyboard.input_timeout_seconds < keyboard.auto_stop_seconds:
        raise ValueError(
            "keyboard_control.input_timeout_seconds must be at least 0.1 and below auto_stop_seconds"
        )
    if not 0.5 <= keyboard.auto_stop_seconds <= 5.0:
        raise ValueError("keyboard_control.auto_stop_seconds must be between 0.5 and 5 seconds")
    for field_name in (
        "max_horizontal_speed_m_s",
        "max_vertical_speed_m_s",
        "max_yaw_rate_deg_s",
    ):
        if getattr(keyboard, field_name) <= 0:
            raise ValueError(f"keyboard_control.{field_name} must be positive")

    takeoff = _load_section(TakeoffConfig, raw.get("takeoff", {}))
    # The requested operating envelope is a hard safety boundary, not a
    # user-tunable suggestion.  A config edit must not silently widen it.
    if abs(takeoff.min_height_m - 1.0) > 1e-9:
        raise ValueError("takeoff.min_height_m must remain exactly 1.0 m")
    if abs(takeoff.max_height_m - 3.0) > 1e-9:
        raise ValueError("takeoff.max_height_m must remain exactly 3.0 m")
    if not takeoff.min_height_m <= takeoff.default_height_m <= takeoff.max_height_m:
        raise ValueError("takeoff.default_height_m must be between 1.0 and 3.0 m")
    for field_name in (
        "preflight_stable_seconds",
        "arm_ack_timeout_seconds",
        "arm_to_takeoff_delay_seconds",
        "takeoff_ack_timeout_seconds",
        "liftoff_timeout_seconds",
        "climb_timeout_seconds",
        "hover_stable_seconds",
        "height_tolerance_m",
        "vertical_speed_tolerance_m_s",
        "liftoff_height_m",
        "preflight_max_tilt_deg",
        "max_tilt_deg",
        "max_horizontal_drift_m",
        "position_stale_seconds",
    ):
        if getattr(takeoff, field_name) <= 0:
            raise ValueError(f"takeoff.{field_name} must be positive")
    if takeoff.preflight_max_tilt_deg > takeoff.max_tilt_deg:
        raise ValueError("takeoff.preflight_max_tilt_deg must not exceed max_tilt_deg")
    if takeoff.max_tilt_deg > 35.0:
        raise ValueError("takeoff.max_tilt_deg must not exceed 35 degrees")
    if takeoff.height_tolerance_m >= takeoff.min_height_m:
        raise ValueError("takeoff.height_tolerance_m must be below the minimum height")
    if takeoff.liftoff_height_m >= takeoff.min_height_m:
        raise ValueError("takeoff.liftoff_height_m must be below the minimum height")
    if takeoff.position_stale_seconds > telemetry.stale_after_seconds:
        raise ValueError(
            "takeoff.position_stale_seconds must not exceed telemetry.stale_after_seconds"
        )

    local_takeoff = _load_section(
        LocalOffboardTakeoffConfig, raw.get("local_offboard_takeoff", {})
    )
    if type(local_takeoff.keyboard_handoff_enabled) is not bool:
        raise ValueError("local_offboard_takeoff.keyboard_handoff_enabled must be a boolean")
    if local_takeoff.navigation_profile not in {"detailed", "px4_fixed_hover", "px4_position_keyboard"}:
        raise ValueError(
            "local_offboard_takeoff.navigation_profile must be detailed, "
            "px4_fixed_hover or px4_position_keyboard"
        )
    if local_takeoff.navigation_profile == "px4_fixed_hover" and local_takeoff.keyboard_handoff_enabled:
        raise ValueError("px4_fixed_hover forbids keyboard handoff")
    if local_takeoff.navigation_profile == "px4_position_keyboard" and not local_takeoff.keyboard_handoff_enabled:
        raise ValueError("px4_position_keyboard requires keyboard handoff")
    if local_takeoff.navigation_profile == "px4_position_keyboard" and keyboard.available:
        raise ValueError(
            "px4_position_keyboard requires keyboard_control.available=false; "
            "the legacy BODY_NED velocity sender must remain disabled"
        )
    if abs(local_takeoff.min_height_m - 1.0) > 1e-9:
        raise ValueError("local_offboard_takeoff.min_height_m must remain exactly 1.0 m")
    if abs(local_takeoff.max_height_m - 3.0) > 1e-9:
        raise ValueError("local_offboard_takeoff.max_height_m must remain exactly 3.0 m")
    if not (
        local_takeoff.min_height_m
        <= local_takeoff.default_height_m
        <= local_takeoff.max_height_m
    ):
        raise ValueError(
            "local_offboard_takeoff.default_height_m must be between 1.0 and 3.0 m"
        )
    for field_name in (
        "frequency_hz",
        "prestream_seconds",
        "offboard_wait_timeout_seconds",
        "arm_ack_timeout_seconds",
        "liftoff_timeout_seconds",
        "climb_timeout_seconds",
        "climb_rate_m_s",
        "max_climb_setpoint_lead_m",
        "hover_stable_seconds",
        "height_tolerance_m",
        "vertical_speed_tolerance_m_s",
        "liftoff_height_m",
        "preflight_max_tilt_deg",
        "max_tilt_deg",
        "max_horizontal_drift_m",
        "position_stale_seconds",
    ):
        if getattr(local_takeoff, field_name) <= 0:
            raise ValueError(f"local_offboard_takeoff.{field_name} must be positive")
    if not 5.0 <= local_takeoff.frequency_hz <= 20.0:
        raise ValueError("local_offboard_takeoff.frequency_hz must be between 5 and 20 Hz")
    if local_takeoff.prestream_seconds < 1.0:
        raise ValueError("local_offboard_takeoff.prestream_seconds must be at least 1 second")
    if not 0.10 <= local_takeoff.max_climb_setpoint_lead_m <= 0.50:
        raise ValueError(
            "local_offboard_takeoff.max_climb_setpoint_lead_m must be between 0.10 and 0.50 m"
        )
    if local_takeoff.preflight_max_tilt_deg > local_takeoff.max_tilt_deg:
        raise ValueError(
            "local_offboard_takeoff.preflight_max_tilt_deg must not exceed max_tilt_deg"
        )
    if local_takeoff.max_tilt_deg > 35.0:
        raise ValueError("local_offboard_takeoff.max_tilt_deg must not exceed 35 degrees")
    if local_takeoff.height_tolerance_m >= local_takeoff.min_height_m:
        raise ValueError(
            "local_offboard_takeoff.height_tolerance_m must be below the minimum height"
        )
    if local_takeoff.liftoff_height_m >= local_takeoff.min_height_m:
        raise ValueError(
            "local_offboard_takeoff.liftoff_height_m must be below the minimum height"
        )
    if local_takeoff.position_stale_seconds > telemetry.stale_after_seconds:
        raise ValueError(
            "local_offboard_takeoff.position_stale_seconds must not exceed telemetry.stale_after_seconds"
        )
    if router.enforce_rc_priority and (
        router.approved_setpoint_system != ground_test.source_system
        or router.approved_setpoint_component != ground_test.source_component
    ):
        raise ValueError(
            "local Offboard takeoff must use the router-approved setpoint identity"
        )

    return AppConfig(
        video=video,
        frame_interpolation=interpolation,
        vision=vision,
        vision_models=vision_models,
        mavlink_router=router,
        telemetry=telemetry,
        position_stream=position_stream,
        control=control,
        ground_offboard_test=ground_test,
        keyboard_control=keyboard,
        takeoff=takeoff,
        local_offboard_takeoff=local_takeoff,
        server=_load_section(ServerConfig, raw.get("server", {})),
    )
