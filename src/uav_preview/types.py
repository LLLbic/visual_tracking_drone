from __future__ import annotations

from dataclasses import asdict, dataclass, field
from time import monotonic
from typing import Any


@dataclass(slots=True)
class TelemetrySnapshot:
    # Trusted adapter evidence only; absent by default. Never infer from quality.
    flow_fusion_active: bool | None = None
    flow_innovation_rejected: bool | None = None
    flow_fusion_instance: int | None = None
    flow_innovation_x_ratio: float | None = None
    flow_innovation_y_ratio: float | None = None
    estimator_dead_reckoning: bool | None = None
    last_flow_fusion_monotonic: float | None = None
    # (primary instance, xy reset, vxy reset, z reset, vz reset, heading reset)
    estimator_reset_signature: tuple[int, int, int, int, int, int] | None = None
    last_reset_evidence_monotonic: float | None = None
    navigation_fault: str = ""
    flow_quality: int | None = None
    flow_minimum_quality: int = 100
    flow_source: str = ""
    flow_hz: float | None = None
    flow_error: str = ""
    flow_good_since_monotonic: float | None = None
    flow_good_samples: int = 0
    last_flow_bad_monotonic: float | None = None
    flow_sources: dict[str, Any] = field(default_factory=dict)
    laser_hz: float | None = None
    laser_sample_id: int | None = None
    last_flow_monotonic: float | None = None
    estimator_flags: int | None = None
    estimator_velocity_ratio: float | None = None
    estimator_position_ratio: float | None = None
    estimator_velocity_ratio_status: str = "missing"
    estimator_position_ratio_status: str = "missing"
    estimator_position_ratio_is_nan: bool = False
    estimator_hz: float | None = None
    estimator_sample_id: int | None = None  # ESTIMATOR_STATUS.time_usec
    last_estimator_monotonic: float | None = None
    connected: bool = False
    source: str = ""
    system_id: int | None = None
    component_id: int | None = None
    flight_mode: str = "UNKNOWN"
    armed: bool | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    vx_m_s: float | None = None
    vy_m_s: float | None = None
    vz_m_s: float | None = None
    altitude_m: float | None = None
    local_x_m: float | None = None
    local_y_m: float | None = None
    local_z_m: float | None = None
    relative_altitude_m: float | None = None
    global_altitude_amsl_m: float | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    last_local_position_monotonic: float | None = None
    local_position_sample_id: int | None = None  # FC time_boot_ms, not packet receive count
    last_distinct_position_monotonic: float | None = None
    last_global_position_monotonic: float | None = None
    local_position_hz: float | None = None
    global_position_hz: float | None = None
    extended_state_hz: float | None = None
    last_heartbeat_monotonic: float | None = None
    last_attitude_monotonic: float | None = None
    last_extended_state_monotonic: float | None = None
    last_rc_channels_monotonic: float | None = None
    laser_height_m: float | None = None
    laser_min_m: float | None = None
    laser_max_m: float | None = None
    laser_sensor_id: int | None = None
    last_laser_monotonic: float | None = None
    landed_state: str = "UNKNOWN"
    throttle_pct: float | None = None
    stick_roll_pct: float | None = None
    stick_pitch_pct: float | None = None
    stick_yaw_pct: float | None = None
    stick_throttle_pct: float | None = None
    rc_channel_5_pwm: int | None = None
    rc_channel_6_pwm: int | None = None
    rc_channel_7_pwm: int | None = None
    rc_channel_8_pwm: int | None = None
    rc_map_offb_sw: int | None = None
    rc_map_mode_sw: int | None = None
    com_fltmode1: int | None = None
    com_fltmode2: int | None = None
    com_fltmode3: int | None = None
    com_fltmode4: int | None = None
    com_fltmode5: int | None = None
    com_fltmode6: int | None = None
    last_status_text: str = ""
    last_status_severity: int | None = None
    last_status_monotonic: float | None = None
    last_command_ack_command: int | None = None
    last_command_ack_result: int | None = None
    last_command_ack_progress: int | None = None
    last_command_ack_monotonic: float | None = None
    last_packet_monotonic: float | None = None
    packets: int = 0
    received_datagrams: int = 0
    forwarded_datagrams: int = 0
    dropped_datagrams: int = 0
    qgc_forward_enabled: bool = False
    qgc_forward_endpoint: str = ""
    error: str = ""

    def age_seconds(self, now: float | None = None) -> float | None:
        if self.last_packet_monotonic is None:
            return None
        return max(0.0, (now if now is not None else monotonic()) - self.last_packet_monotonic)

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["age_seconds"] = self.age_seconds()
        result["laser_age_seconds"] = (
            None
            if self.last_laser_monotonic is None
            else max(0.0, monotonic() - self.last_laser_monotonic)
        )
        result["last_status_age_seconds"] = (
            None
            if self.last_status_monotonic is None
            else max(0.0, monotonic() - self.last_status_monotonic)
        )
        result["last_command_ack_age_seconds"] = (
            None
            if self.last_command_ack_monotonic is None
            else max(0.0, monotonic() - self.last_command_ack_monotonic)
        )
        result["local_position_age_seconds"] = (
            None
            if self.last_local_position_monotonic is None
            else max(0.0, monotonic() - self.last_local_position_monotonic)
        )
        result["global_position_age_seconds"] = (
            None
            if self.last_global_position_monotonic is None
            else max(0.0, monotonic() - self.last_global_position_monotonic)
        )
        for public_name, field_name in (
            ("heartbeat_age_seconds", "last_heartbeat_monotonic"),
            ("attitude_age_seconds", "last_attitude_monotonic"),
            ("extended_state_age_seconds", "last_extended_state_monotonic"),
            ("rc_channels_age_seconds", "last_rc_channels_monotonic"),
            ("estimator_age_seconds", "last_estimator_monotonic"),
            ("flow_age_seconds", "last_flow_monotonic"),
            ("flow_last_bad_age_seconds", "last_flow_bad_monotonic"),
            ("flow_fusion_age_seconds", "last_flow_fusion_monotonic"),
            ("reset_evidence_age_seconds", "last_reset_evidence_monotonic"),
        ):
            value = getattr(self, field_name)
            result[public_name] = None if value is None else max(0.0, monotonic() - value)
        # A silent stream cannot accumulate "good" time just because UI polls.
        result["flow_good_seconds"] = (None if self.flow_good_since_monotonic is None or self.last_flow_monotonic is None
            else max(0.,self.last_flow_monotonic-self.flow_good_since_monotonic))
        result.pop("last_packet_monotonic", None)
        result.pop("last_laser_monotonic", None)
        result.pop("last_status_monotonic", None)
        result.pop("last_command_ack_monotonic", None)
        result.pop("last_local_position_monotonic", None)
        result.pop("last_global_position_monotonic", None)
        result.pop("last_heartbeat_monotonic", None)
        result.pop("last_attitude_monotonic", None)
        result.pop("last_extended_state_monotonic", None)
        result.pop("last_rc_channels_monotonic", None)
        return result


@dataclass(slots=True)
class DetectedTrack:
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    red_ratio: float | None = None

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class TargetSnapshot:
    locked: bool = False
    track_id: int | None = None
    class_name: str = ""
    confidence: float | None = None
    bbox: tuple[float, float, float, float] | None = None
    center_error_x: float | None = None
    bbox_height_ratio: float | None = None
    source: str = "none"
    lost_frames: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class CommandPreview:
    raw_forward_m_s: float = 0.0
    raw_yaw_rate_deg_s: float = 0.0
    eligible_forward_m_s: float = 0.0
    eligible_yaw_rate_deg_s: float = 0.0
    transmitted_forward_m_s: float = 0.0
    transmitted_yaw_rate_deg_s: float = 0.0
    offboard_observed: bool = False
    telemetry_fresh: bool = False
    target_valid: bool = False
    local_estop_latched: bool = False
    transmission_enabled: bool = False
    gate_reason: str = "passive build"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class VisionSnapshot:
    connected: bool = False
    source: str = ""
    capture_backend: str = "opencv"
    video_decoder: str = "software"
    width: int = 0
    height: int = 0
    capture_fps: float = 0.0
    processing_fps: float = 0.0
    output_fps: float = 0.0
    interpolation_enabled: bool = False
    interpolation_ready: bool = False
    interpolation_backend: str = ""
    interpolation_error: str = ""
    interpolation_ms: float = 0.0
    synthetic_frames: int = 0
    repeated_synthetic_frames: int = 0
    detector_ready: bool = False
    capture_error: str = ""
    reconnect_count: int = 0
    soft_read_failures: int = 0
    dropped_capture_frames: int = 0
    display_mode: str = "ordered"
    detector_error: str = ""
    tracker_error: str = ""
    last_frame_monotonic: float | None = None
    tracks: list[DetectedTrack] = field(default_factory=list)
    target: TargetSnapshot = field(default_factory=TargetSnapshot)

    def age_seconds(self, now: float | None = None) -> float | None:
        if self.last_frame_monotonic is None:
            return None
        return max(0.0, (now if now is not None else monotonic()) - self.last_frame_monotonic)

    def to_dict(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "source": self.source,
            "capture_backend": self.capture_backend,
            "video_decoder": self.video_decoder,
            "width": self.width,
            "height": self.height,
            "capture_fps": self.capture_fps,
            "processing_fps": self.processing_fps,
            "output_fps": self.output_fps,
            "interpolation_enabled": self.interpolation_enabled,
            "interpolation_ready": self.interpolation_ready,
            "interpolation_backend": self.interpolation_backend,
            "interpolation_error": self.interpolation_error,
            "interpolation_ms": self.interpolation_ms,
            "synthetic_frames": self.synthetic_frames,
            "repeated_synthetic_frames": self.repeated_synthetic_frames,
            "detector_ready": self.detector_ready,
            "capture_error": self.capture_error,
            "reconnect_count": self.reconnect_count,
            "soft_read_failures": self.soft_read_failures,
            "dropped_capture_frames": self.dropped_capture_frames,
            "display_mode": self.display_mode,
            "detector_error": self.detector_error,
            "tracker_error": self.tracker_error,
            "age_seconds": self.age_seconds(),
            "tracks": [track.to_dict() for track in self.tracks],
            "target": self.target.to_dict(),
        }
