from __future__ import annotations

from dataclasses import asdict, dataclass, field
from time import monotonic
from typing import Any


@dataclass(slots=True)
class TelemetrySnapshot:
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
        result.pop("last_packet_monotonic", None)
        result.pop("last_laser_monotonic", None)
        result.pop("last_status_monotonic", None)
        result.pop("last_command_ack_monotonic", None)
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
            "detector_error": self.detector_error,
            "tracker_error": self.tracker_error,
            "age_seconds": self.age_seconds(),
            "tracks": [track.to_dict() for track in self.tracks],
            "target": self.target.to_dict(),
        }
