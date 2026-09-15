"""PX4-estimate-backed safety shared by fixed hover and position-only keyboard motion.

Standard telemetry is not a substitute for detailed EKF fusion/reset evidence.
This limited profile explicitly reports those observability gaps. It never
invents fusion flags or changes PX4's arming checks/failsafe parameters.
"""
from math import isfinite, sqrt

from .navigation_health import confirmed_velocity_ratio_reason, estimator_ratio_reason
from .types import TelemetrySnapshot


def fresh(stamp, now, limit):
    return stamp is not None and isfinite(stamp) and 0 <= now-stamp <= limit


def fixed_hover_health(t: TelemetrySnapshot, now: float):
    blockers, warnings = [], []
    if t.navigation_fault:
        blockers.append("定位故障已锁存：" + t.navigation_fault)
    if not t.connected or t.system_id is None or t.component_id != 1:
        blockers.append("飞控遥测来源未确认或连接离线")
    for label, stamp, limit in (
        ("飞控心跳", t.last_heartbeat_monotonic, 1.5),
        ("实体RC", t.last_rc_channels_monotonic, 1.0),
        ("姿态", t.last_attitude_monotonic, .5),
        ("本地位置", t.last_local_position_monotonic, 1.0),
        # This radio currently delivers EXTENDED_SYS_STATE at about 0.6 Hz
        # even after a 2 Hz interval request.  Allow one normal 1.67 s cycle
        # plus bounded jitter without weakening position/attitude/RC checks.
        ("落地状态", t.last_extended_state_monotonic, 2.5),
        ("估计器状态", t.last_estimator_monotonic, 1.5),
    ):
        if not fresh(stamp, now, limit):
            blockers.append(label + "缺失或过期，禁止基础起飞控制")
    if (type(t.local_position_sample_id) is not int or not 0 <= t.local_position_sample_id <= 0xFFFFFFFF
            or not fresh(t.last_distinct_position_monotonic, now, 1.0)):
        blockers.append("缺少新鲜的飞控位置源样本；重传不能刷新定位")
    if any(v is None or not isfinite(v) for v in (
        t.local_x_m, t.local_y_m, t.local_z_m, t.vx_m_s, t.vy_m_s, t.vz_m_s,
        t.roll_deg, t.pitch_deg, t.yaw_deg,
    )):
        blockers.append("位置、速度或姿态不是有效有限数值")
    flags = t.estimator_flags
    if (type(flags) is not int or flags & 15 != 15 or not flags & (32 | 64)
            or flags & (128 | 1024 | 2048)):
        blockers.append("飞控未确认姿态、水平位置/速度或垂直定位有效，或报告估计器异常")
    velocity_reason = confirmed_velocity_ratio_reason(t)
    if velocity_reason:
        blockers.append(velocity_reason)
    elif (
        t.estimator_velocity_ratio is not None
        and isfinite(t.estimator_velocity_ratio)
        and t.estimator_velocity_ratio > 1
    ):
        warnings.append(
            "估计器速度创新比短时超限，等待连续样本确认"
            f"（{t.estimator_velocity_ratio_bad_samples}/2）"
        )
    if t.estimator_position_ratio is not None:
        reason = estimator_ratio_reason(t.estimator_position_ratio, "水平位置")
        if reason:
            blockers.append(reason)
    elif type(flags) is int and flags & 16:
        blockers.append("飞控报告绝对水平定位有效，但水平位置创新证据缺失")
    elif t.estimator_position_ratio_status == "invalid" or (
        t.estimator_position_ratio_status == "nonfinite" and not t.estimator_position_ratio_is_nan
    ):
        blockers.append("水平位置创新报文无效（不是可标记为未提供的NaN）")
    else:
        warnings.append("仅使用飞控相对定位有效标志；水平位置创新比未提供，未独立验证")
    for channel in (t.rc_channel_6_pwm, t.rc_channel_8_pwm):
        if channel is None or not isfinite(channel) or not 800 <= channel <= 2200:
            blockers.append("实体急停/模式通道无效")
            break
    # Missing raw downlink is an observation gap, not proof of onboard failure.
    # An actual observed bad measurement must never be hidden by this profile.
    if t.flow_error:
        blockers.append(t.flow_error)
    if (t.flow_quality_authoritative and t.flow_source and t.flow_quality is not None
            and not t.flow_minimum_quality <= t.flow_quality <= 255):
        blockers.append("已观测到指定来源光流质量低于门槛，禁止基础起飞控制")
    if t.flow_source and not t.flow_quality_authoritative:
        warnings.append("当前链路光流quality字段未标定/可能为占位值，仅作诊断显示，不参与基础起飞或悬停判定")
    if not t.flow_source or not fresh(t.last_flow_monotonic, now, 1.5):
        warnings.append("未收到新鲜的指定来源原始光流；光流质量未验证，不能据此授权跟踪或交接")
    if t.flow_fusion_active is False or t.flow_innovation_rejected is True or t.estimator_dead_reckoning is True:
        blockers.append("已观测到光流未融合、被拒绝或估计器纯推算状态")
    for label, ratio in (("光流X", t.flow_innovation_x_ratio), ("光流Y", t.flow_innovation_y_ratio)):
        if ratio is not None:
            reason = estimator_ratio_reason(ratio, label)
            if reason:
                blockers.append(reason)
    if (not fresh(t.last_flow_fusion_monotonic, now, .5) or t.flow_fusion_active is None
            or t.flow_innovation_rejected is None or t.estimator_dead_reckoning is None):
        warnings.append("详细光流融合证据未接入/过期；基础悬停依赖PX4定位输出，不代表已验证融合")
    if t.estimator_reset_signature is None:
        warnings.append("完整重置计数未接入；仅监视可观测的坐标/航向突变，不能检出所有重置")
    elif not fresh(t.last_reset_evidence_monotonic, now, .5):
        blockers.append("已接入的估计器重置证据过期，禁止沿用旧坐标")
    elif (not isinstance(t.estimator_reset_signature, tuple) or len(t.estimator_reset_signature) != 6
            or any(type(v) is not int or v < 0 for v in t.estimator_reset_signature)):
        blockers.append("已接入的估计器重置计数格式无效")
    if fresh(t.last_laser_monotonic, now, 1.5):
        if (any(v is None or not isfinite(v) for v in (t.laser_height_m, t.laser_min_m, t.laser_max_m))
                or not 0 <= t.laser_min_m < t.laser_max_m
                or not t.laser_min_m <= t.laser_height_m <= t.laser_max_m):
            blockers.append("已收到的对地测距无效或超出量程")
    else:
        warnings.append("对地测距未更新；本次高度基于起飞时本地Z差值，不把旧测距当实时高度")
    return list(dict.fromkeys(blockers)), warnings


class FixedHoverFrameMonitor:
    """Discontinuity alarm, NOT an EKF reset detector or drift-free guarantee."""
    def __init__(self, t: TelemetrySnapshot):
        self.signature = t.estimator_reset_signature
        self.source = (t.system_id, t.component_id)
        self.sample = t.local_position_sample_id
        self.received = t.last_distinct_position_monotonic
        self.xyz = (t.local_x_m, t.local_y_m, t.local_z_m)
        self.velocity = (t.vx_m_s, t.vy_m_s, t.vz_m_s)
        self.yaw = t.yaw_deg
        self.yaw_stamp = t.last_attitude_monotonic

    def check(self, t: TelemetrySnapshot, now: float):
        if (t.system_id, t.component_id) != self.source:
            raise ValueError("飞控来源变化，原位置目标作废")
        if t.estimator_reset_signature != self.signature:
            raise ValueError("重置计数/实例证据变化，原坐标任务作废，不自动恢复")
        if self.signature is not None:
            if (not isinstance(self.signature, tuple) or len(self.signature) != 6
                    or any(type(v) is not int or v < 0 for v in self.signature)):
                raise ValueError("已接入的重置计数格式无效")
            if t.flow_fusion_instance is not None and t.flow_fusion_instance != self.signature[0]:
                raise ValueError("融合证据和主估计器实例不一致")
        if t.last_attitude_monotonic != self.yaw_stamp:
            delta = (t.yaw_deg-self.yaw+180) % 360-180
            # No commanded yaw change in this profile. Conservative alarm on a
            # large per-sample change; cannot distinguish physical turn/reset.
            if abs(delta) > 15:
                raise ValueError("固定航向发生突变，停止沿用原目标")
            self.yaw, self.yaw_stamp = t.yaw_deg, t.last_attitude_monotonic
        if t.local_position_sample_id == self.sample:
            return
        dt = ((t.local_position_sample_id-self.sample) & 0xFFFFFFFF)/1000.
        arrival_dt = t.last_distinct_position_monotonic-self.received
        if not 0 < dt <= 1.0 or not 0 <= arrival_dt <= 1.0 or abs(dt-arrival_dt) > .5:
            raise ValueError("位置源时间倒退、间断或积压，原目标作废")
        xyz, velocity = (t.local_x_m, t.local_y_m, t.local_z_m), (t.vx_m_s, t.vy_m_s, t.vz_m_s)
        residual = sqrt(sum((new-old-(v0+v1)*.5*dt)**2
                            for new, old, v0, v1 in zip(xyz, self.xyz, self.velocity, velocity)))
        if residual > .20:
            raise ValueError("本地坐标疑似跳变超过0.20米，原目标作废（非完整重置计数检测）")
        self.sample, self.received = t.local_position_sample_id, t.last_distinct_position_monotonic
        self.xyz, self.velocity = xyz, velocity
