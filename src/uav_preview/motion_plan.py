"""Deterministic keyboard plan; no I/O, mode switches, arming or parameters.

Position capture requires a full second of stationary estimates. These limits
are conservative engineering defaults, not a validated aircraft tuning.
"""
from math import hypot, radians
from .types import TelemetrySnapshot


class KeyboardMotionPlan:
    def __init__(self) -> None:
        self.phase = "BRAKING"
        self.target: tuple[float, float, float, float] | None = None
        self.velocity = (0.0, 0.0, 0.0, 0.0)
        self.stable_since: float | None = None
        self.last_time: float | None = None

    def step(self, requested: tuple[float, float, float, float], t: TelemetrySnapshot,
             now: float) -> tuple[str, tuple[float, float, float, float]]:
        dt = 0.0 if self.last_time is None else min(0.15, max(0.0, now - self.last_time))
        self.last_time = now
        moving = any(abs(v) > 1e-8 for v in requested)
        if moving:
            self.target = None
            self.stable_since = None
            self.phase = "MOVING"
        elif self.target is not None:
            self.phase = "POSITION_HOLD"
            return "position", self.target
        else:
            self.phase = "BRAKING"
        limits = (0.5, 0.5, 0.3, 20.0)
        self.velocity = tuple(old + max(-a * dt, min(a * dt, new - old))
                              for old, new, a in zip(self.velocity, requested, limits))
        stopped = max(abs(v) for v in self.velocity) < 1e-6
        measured_stable = hypot(t.vx_m_s, t.vy_m_s) <= 0.10 and abs(t.vz_m_s) <= 0.10
        if not moving and stopped and measured_stable:
            if self.stable_since is None:
                self.stable_since = now
            if now - self.stable_since >= 1.0:
                self.target = (t.local_x_m, t.local_y_m, t.local_z_m, radians(t.yaw_deg))
                self.phase = "POSITION_HOLD"
                return "position", self.target
        else:
            self.stable_since = None
        return "velocity", self.velocity
