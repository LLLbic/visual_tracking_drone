from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Callable, TypeVar


_T = TypeVar("_T")


# This constant applies only to continuous vision-follow setpoints. Explicit,
# user-confirmed ARM/DISARM and Pause actions are implemented on a separate,
# telemetry-gated path.
CONTROL_TRANSMISSION_COMPILED = False


@dataclass(frozen=True, slots=True)
class GateDecision:
    eligible: bool
    reason: str
    offboard_observed: bool
    telemetry_fresh: bool
    local_estop_latched: bool


class SafetyGate:
    """Latched local gate shared by preview and explicit web ARM protection.

    It never changes a PX4 parameter, flight mode, arming state, or RC channel.
    """

    def __init__(self, *, initially_latched: bool = False) -> None:
        self._lock = Lock()
        self._estop_latched = bool(initially_latched)

    @property
    def estop_latched(self) -> bool:
        with self._lock:
            return self._estop_latched

    def latch_estop(self) -> None:
        with self._lock:
            self._estop_latched = True

    def reset_estop(self) -> None:
        with self._lock:
            self._estop_latched = False

    def run_if_unlatched(self, action: Callable[[], _T]) -> _T:
        """Run one unsafe transmit atomically with the latch check.

        Holding the same lock used by ``latch_estop`` closes the race where an
        ARM/TAKEOFF/RTL request passed an earlier check just before the operator
        engaged the latch.  Once ``latch_estop`` returns, no guarded transmit
        can still be in flight or begin afterwards.
        """

        with self._lock:
            if self._estop_latched:
                raise ValueError("网页安全处置仍处于锁存状态，拒绝发送该飞控动作")
            return action()

    def evaluate(self, flight_mode: str, telemetry_fresh: bool, target_valid: bool) -> GateDecision:
        mode = (flight_mode or "UNKNOWN").upper().replace(" ", "_")
        offboard = mode == "OFFBOARD"
        estop = self.estop_latched

        if estop:
            reason = "本地急停已锁存"
            eligible = False
        elif not telemetry_fresh:
            reason = "遥测丢失或已过期"
            eligible = False
        elif not offboard:
            reason = f"当前模式 {mode}，只有人工切入 OFFBOARD 才可授权"
            eligible = False
        elif not target_valid:
            reason = "没有有效锁定目标"
            eligible = False
        else:
            reason = "条件满足，但当前为只读预览版，实际发送始终关闭"
            eligible = True

        return GateDecision(
            eligible=eligible,
            reason=reason,
            offboard_observed=offboard,
            telemetry_fresh=telemetry_fresh,
            local_estop_latched=estop,
        )
