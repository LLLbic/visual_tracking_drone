from __future__ import annotations

from dataclasses import dataclass, fields
from ipaddress import ip_address
from pathlib import Path
from typing import Any, TypeVar
import tomllib


@dataclass(slots=True)
class VideoConfig:
    source: str = "0"
    transport: str = "tcp"
    reconnect_seconds: float = 2.0
    jpeg_quality: int = 82


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

    def __post_init__(self) -> None:
        if self.allowed_source_ips is None:
            self.allowed_source_ips = []


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
    allow_ground_takeoff: bool = False
    frequency_hz: float = 10.0
    input_timeout_seconds: float = 0.4
    auto_stop_seconds: float = 1.5
    max_horizontal_speed_m_s: float = 0.25
    max_vertical_speed_m_s: float = 0.20
    max_yaw_rate_deg_s: float = 10.0


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
    control: ControlConfig
    ground_offboard_test: GroundOffboardTestConfig
    keyboard_control: KeyboardControlConfig
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

    return AppConfig(
        video=video,
        frame_interpolation=interpolation,
        vision=vision,
        vision_models=vision_models,
        mavlink_router=router,
        telemetry=telemetry,
        control=control,
        ground_offboard_test=ground_test,
        keyboard_control=keyboard,
        server=_load_section(ServerConfig, raw.get("server", {})),
    )
