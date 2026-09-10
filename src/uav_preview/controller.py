from __future__ import annotations

from dataclasses import replace
from math import isfinite
from threading import Lock
from time import monotonic

from .config import ControlConfig
from .safety import CONTROL_TRANSMISSION_COMPILED, SafetyGate
from .types import CommandPreview, TargetSnapshot, TelemetrySnapshot


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PID:
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float, integral_limit: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.previous_error: float | None = None
        self.previous_time: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def update(self, error: float, now: float | None = None) -> float:
        sample_time = monotonic() if now is None else now
        if not isfinite(error):
            self.reset()
            return 0.0

        dt = 0.0 if self.previous_time is None else max(0.0, min(0.5, sample_time - self.previous_time))
        derivative = 0.0
        if dt > 1e-4 and self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
            self.integral = _clamp(
                self.integral + error * dt,
                -self.integral_limit,
                self.integral_limit,
            )

        self.previous_error = error
        self.previous_time = sample_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return _clamp(output, -self.output_limit, self.output_limit)


class PreviewController:
    """Computes suggestions but cannot transmit them to the aircraft."""

    def __init__(self, config: ControlConfig, gate: SafetyGate) -> None:
        self._lock = Lock()
        self.config = replace(config)
        self.gate = gate
        self._rebuild_pids()

    def _rebuild_pids(self) -> None:
        cfg = self.config
        self.yaw_pid = PID(cfg.yaw_kp, cfg.yaw_ki, cfg.yaw_kd, cfg.max_yaw_rate_deg_s, cfg.integral_limit)
        self.distance_pid = PID(
            cfg.distance_kp,
            cfg.distance_ki,
            cfg.distance_kd,
            cfg.max_forward_speed_m_s,
            cfg.integral_limit,
        )
    def update_tuning(self, values: dict[str, float]) -> None:
        allowed = {
            "target_bbox_height_ratio",
            "yaw_deadband",
            "distance_deadband",
            "yaw_kp",
            "yaw_ki",
            "yaw_kd",
            "distance_kp",
            "distance_ki",
            "distance_kd",
            "max_yaw_rate_deg_s",
            "max_forward_speed_m_s",
        }
        with self._lock:
            for key, value in values.items():
                if key not in allowed:
                    raise ValueError(f"Unsupported tuning key: {key}")
                if not isfinite(value):
                    raise ValueError(f"{key} must be finite")
                setattr(self.config, key, float(value))
            if not 0.0 < self.config.target_bbox_height_ratio < 1.0:
                raise ValueError("target_bbox_height_ratio must be between 0 and 1")
            if self.config.max_yaw_rate_deg_s <= 0 or self.config.max_forward_speed_m_s <= 0:
                raise ValueError("output limits must be positive")
            self._rebuild_pids()

    def tuning(self) -> dict[str, float]:
        with self._lock:
            return {
                key: float(getattr(self.config, key))
                for key in (
                    "target_bbox_height_ratio",
                    "yaw_deadband",
                    "distance_deadband",
                    "yaw_kp",
                    "yaw_ki",
                    "yaw_kd",
                    "distance_kp",
                    "distance_ki",
                    "distance_kd",
                    "max_yaw_rate_deg_s",
                    "max_forward_speed_m_s",
                )
            }

    def compute(
        self,
        target: TargetSnapshot,
        telemetry: TelemetrySnapshot,
        telemetry_stale_after: float,
        now: float | None = None,
    ) -> CommandPreview:
        sample_time = monotonic() if now is None else now
        with self._lock:
            cfg = replace(self.config)

            target_valid = bool(
                target.locked
                and target.center_error_x is not None
                and target.bbox_height_ratio is not None
            )
            raw_yaw = 0.0
            raw_forward = 0.0
            if target_valid:
                yaw_error = float(target.center_error_x)
                distance_error = cfg.target_bbox_height_ratio - float(target.bbox_height_ratio)
                if abs(yaw_error) < cfg.yaw_deadband:
                    yaw_error = 0.0
                    self.yaw_pid.reset()
                if abs(distance_error) < cfg.distance_deadband:
                    distance_error = 0.0
                    self.distance_pid.reset()
                raw_yaw = self.yaw_pid.update(yaw_error, sample_time)
                raw_forward = self.distance_pid.update(distance_error, sample_time)
            else:
                self.yaw_pid.reset()
                self.distance_pid.reset()

        age = telemetry.age_seconds(sample_time)
        telemetry_fresh = bool(telemetry.connected and age is not None and age <= telemetry_stale_after)
        decision = self.gate.evaluate(telemetry.flight_mode, telemetry_fresh, target_valid)
        eligible_forward = raw_forward if decision.eligible else 0.0
        eligible_yaw = raw_yaw if decision.eligible else 0.0

        # Hard invariant: this passive build never has a transport or command sink.
        transmitted_forward = 0.0
        transmitted_yaw = 0.0

        return CommandPreview(
            raw_forward_m_s=raw_forward,
            raw_yaw_rate_deg_s=raw_yaw,
            eligible_forward_m_s=eligible_forward,
            eligible_yaw_rate_deg_s=eligible_yaw,
            transmitted_forward_m_s=transmitted_forward,
            transmitted_yaw_rate_deg_s=transmitted_yaw,
            offboard_observed=decision.offboard_observed,
            telemetry_fresh=decision.telemetry_fresh,
            target_valid=target_valid,
            local_estop_latched=decision.local_estop_latched,
            transmission_enabled=CONTROL_TRANSMISSION_COMPILED,
            gate_reason=decision.reason,
        )
