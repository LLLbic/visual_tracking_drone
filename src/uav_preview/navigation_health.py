"""Read-only, fail-closed checks for this optical-flow aircraft."""
from math import isfinite
from time import monotonic
from .types import TelemetrySnapshot


def estimator_ratio_reason(value: float | None, label: str) -> str:
    # Missing/NaN is unavailable evidence, not a measured innovation failure.
    # Both deny navigation; never substitute zero for an unavailable ratio.
    if value is None or not isfinite(value):
        return f"估计器{label}创新比缺失或非有限值，无法验证（不等同检验失败）"
    if value < 0:
        return f"估计器{label}创新比为无效负值"
    if value > 1:
        return f"估计器{label}创新比 {value:.3f} > 1，检验未通过"
    return ""


def confirmed_velocity_ratio_reason(t: TelemetrySnapshot) -> str:
    """Apply the receiver's consecutive-sample policy to velocity ratio."""

    value = t.estimator_velocity_ratio
    reason = estimator_ratio_reason(value, "速度")
    # Missing/non-finite and invalid negative values still fail closed.  Only
    # the noisy >1 rejection threshold receives a short debounce window.
    if value is not None and isfinite(value) and value > 1:
        return reason if t.estimator_velocity_ratio_confirmed_bad else ""
    return reason


def navigation_block_reasons(t: TelemetrySnapshot, now: float | None = None) -> list[str]:
    now = monotonic() if now is None else now
    reasons = []
    if t.navigation_fault:
        reasons.append("定位故障已锁存：" + t.navigation_fault)
    for label, stamp, limit in (
        ("心跳", t.last_heartbeat_monotonic, 1.5),
        ("RC", t.last_rc_channels_monotonic, 1.0),
        ("姿态", t.last_attitude_monotonic, 0.5),
        ("本地位置", t.last_local_position_monotonic, 0.5),
        ("落地状态", t.last_extended_state_monotonic, 1.5),
        ("估计器状态", t.last_estimator_monotonic, 1.5),
        ("光流质量", t.last_flow_monotonic, 1.5),
        ("对地测距", t.last_laser_monotonic, 1.5),
    ):
        if stamp is None or not isfinite(stamp) or not 0 <= now - stamp <= limit:
            reasons.append(label + "缺失或过期，禁止电脑导航控制")
    values = (t.local_x_m, t.local_y_m, t.local_z_m, t.vx_m_s, t.vy_m_s,
              t.vz_m_s, t.roll_deg, t.pitch_deg, t.yaw_deg, t.laser_height_m)
    if any(v is None or not isfinite(v) for v in values):
        reasons.append("位置、速度、姿态或测距不是有效有限数值")
    if not t.flow_quality_authoritative:
        reasons.append("光流quality字段来源未标定，不能用占位值授权键盘/跟踪导航")
    elif t.flow_quality is None or not t.flow_minimum_quality <= t.flow_quality <= 255:
        reasons.append(f"光流质量缺失或低于本地门槛 {t.flow_minimum_quality}；非零不代表融合正常")
    if t.flow_error:
        reasons.append(t.flow_error)
    flags = t.estimator_flags
    if flags is None or flags & 15 != 15 or flags & (128 | 1024 | 2048):
        reasons.append("估计器姿态/水平位置/速度无效或处于异常状态")
    for label, ratio in (("速度", t.estimator_velocity_ratio), ("水平位置", t.estimator_position_ratio)):
        reason = (
            confirmed_velocity_ratio_reason(t)
            if label == "速度"
            else estimator_ratio_reason(ratio, label)
        )
        if reason:
            reasons.append(reason)
    if t.laser_min_m is None or t.laser_max_m is None or not (
        t.laser_height_m is not None and 0 <= t.laser_min_m < t.laser_max_m
        and t.laser_min_m <= t.laser_height_m <= t.laser_max_m
    ):
        reasons.append("对地测距超出有效范围或范围未知")
    for value in (t.rc_channel_6_pwm, t.rc_channel_8_pwm):
        if value is None or not 800 <= value <= 2200:
            reasons.append("实体急停/模式通道数据无效")
            break
    return reasons


def navigation_block_reason(t: TelemetrySnapshot, now: float | None = None) -> str:
    reasons = navigation_block_reasons(t, now)
    return reasons[0] if reasons else ""
