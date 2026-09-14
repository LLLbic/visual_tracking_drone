"""Single-reference hover/keyboard handoff. Pure logic: no network or actions.

Engineering limits below are NOT flight validation. Missing fusion/reset evidence
fails closed. ESTIMATOR_STATUS/quality alone must never synthesize that evidence.
"""
from dataclasses import dataclass
from math import cos, sin, hypot, isfinite, radians, atan2
from secrets import token_urlsafe

from .navigation_health import navigation_block_reason, estimator_ratio_reason
from .types import TelemetrySnapshot


class HandoffFault(ValueError):
    """Reference no longer trustworthy: caller must stop, never recapture it."""


@dataclass(frozen=True)
class HandoffLimits:
    stable_seconds: float = 3.0
    max_tick_gap: float = 0.25
    input_timeout: float = 0.4
    horizontal_error: float = 0.15
    vertical_error: float = 0.15
    horizontal_stable_speed: float = 0.08
    vertical_stable_speed: float = 0.10
    max_speed: float = 0.25
    max_vertical_speed: float = 0.20
    max_yaw_rate: float = 10.0  # degrees/s
    acceleration: float = 0.40
    vertical_acceleration: float = 0.30
    yaw_acceleration: float = 20.0  # degrees/s^2
    lead_soft: float = 0.20
    lead_hard: float = 0.45
    vertical_lead_soft: float = 0.15
    vertical_lead_hard: float = 0.30
    yaw_lead_soft_deg: float = 15.0
    yaw_lead_hard_deg: float = 30.0


def reset_evidence_reason(t: TelemetrySnapshot, now: float) -> str:
    stamp = t.last_reset_evidence_monotonic
    if stamp is None or not isfinite(stamp) or not 0 <= now-stamp <= 0.5:
        return "估计器重置计数证据缺失或过期，禁止沿用旧坐标"
    signature = t.estimator_reset_signature
    if not isinstance(signature, tuple) or len(signature) != 6 or any(type(n) is not int or n < 0 for n in signature):
        return "缺少有效的估计器实例及位置/速度/高度/航向重置计数"
    return ""


def detailed_health_reason(t: TelemetrySnapshot, now: float) -> str:
    reason = navigation_block_reason(t, now)
    if reason:
        return reason
    reasons = handoff_evidence_reasons(t, now)
    return reasons[0] if reasons else ""


def position_control_health_reason(t: TelemetrySnapshot, now: float) -> str:
    """Shared takeoff/hold evidence, without keyboard authorization windows."""
    reason = navigation_block_reason(t, now)
    if reason:
        return reason
    reasons = position_control_evidence_reasons(t, now)
    return reasons[0] if reasons else ""


def position_control_evidence_reasons(t: TelemetrySnapshot, now: float) -> list[str]:
    """Frame/estimator evidence is necessary even when there is no handoff."""
    reasons = []
    if (not t.flow_source or t.last_flow_monotonic is None
            or not isfinite(t.last_flow_monotonic) or not 0 <= now-t.last_flow_monotonic <= 0.5):
        reasons.append("指定来源光流证据缺失或过期，禁止定点控制")
    stamp = t.last_distinct_position_monotonic
    if (type(t.local_position_sample_id) is not int or not 0 <= t.local_position_sample_id <= 0xFFFFFFFF
            or stamp is None or not isfinite(stamp) or not 0 <= now-stamp <= 0.5):
        reasons.append("本地位置缺少新鲜的飞控源样本时间，重传包不能作为稳定证据")
    reason = reset_evidence_reason(t,now)
    if reason:
        reasons.append(reason)
    stamp = t.last_flow_fusion_monotonic
    if stamp is None or not isfinite(stamp) or not 0 <= now-stamp <= 0.5:
        reasons.append("光流融合证据缺失或过期，禁止定点控制")
    signature = t.estimator_reset_signature
    if t.estimator_flags is None or not t.estimator_flags & (32 | 64):
        reasons.append("估计器未确认垂直位置有效")
    if not reason and (type(t.flow_fusion_instance) is not int or t.flow_fusion_instance != signature[0]):
        reasons.append("光流融合证据与当前估计器实例不一致或实例缺失")
    if t.flow_fusion_active is None or t.flow_innovation_rejected is None:
        reasons.append("缺少光流是否融合/是否被拒绝的明确状态（不能由质量值推断）")
    elif t.flow_fusion_active is not True or t.flow_innovation_rejected is not False:
        reasons.append("光流未有效融合或测量被拒绝")
    for label,ratio in (("光流X",t.flow_innovation_x_ratio),("光流Y",t.flow_innovation_y_ratio)):
        reason=estimator_ratio_reason(ratio,label)
        if reason:
            reasons.append(reason)
    if t.estimator_dead_reckoning is not False:
        reasons.append("未确认估计器已退出纯推算状态，禁止依赖失去约束的位置")
    return reasons


def handoff_evidence_reasons(t: TelemetrySnapshot, now: float) -> list[str]:
    """Shared safety plus additional stability evidence for keyboard handoff."""
    reasons = position_control_evidence_reasons(t, now)
    flow_since = t.flow_good_since_monotonic
    if (not t.flow_source or flow_since is None or not isfinite(flow_since)
            or now-flow_since < 3.0 or t.flow_good_samples < 15
            or t.last_flow_monotonic is None or not 0 <= now-t.last_flow_monotonic <= 0.5
            or t.last_flow_monotonic-flow_since < 3.0):
        reasons.append("指定来源光流尚未连续良好3秒及15个不同样本，拒绝交接")
    return reasons


def _slew(old: float, new: float, step: float) -> float:
    return old + max(-step, min(step, new - old))


def _remove_outward(velocity, radial):
    """At a bound, allow retreat; remove only velocity away from the center."""
    length = hypot(*radial)
    if length < 1e-9:
        return velocity
    nx,ny = radial[0]/length,radial[1]/length
    outward = max(0.0,velocity[0]*nx+velocity[1]*ny)
    return velocity[0]-outward*nx,velocity[1]-outward*ny,velocity[2]


class ContinuousReference:
    """World-NED position + velocity FF; yaw trajectory, fixed frame throughout."""
    def __init__(self, target, ground_z: float, origin_xy, radius: float, limits: HandoffLimits):
        if not all(isfinite(v) for v in (*target, ground_z, *origin_xy, radius)) or radius <= 0:
            raise HandoffFault("无效的轨迹初始目标")
        self.position = tuple(target[:3])
        self.yaw = target[3]
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0
        self.ground_z, self.origin_xy, self.radius = ground_z, origin_xy, radius
        self.limits = limits
        self.last_time = None
        self.phase = "KEYBOARD_READY"

    def step(self, axes, t: TelemetrySnapshot, now: float):
        lim = self.limits
        if not all(isfinite(v) for v in (*axes, t.local_x_m, t.local_y_m, t.local_z_m, t.yaw_deg, now)):
            raise HandoffFault("轨迹输入不是有限数值")
        dt = 0.0 if self.last_time is None else now - self.last_time
        if not 0 <= dt <= lim.max_tick_gap:
            raise HandoffFault("轨迹循环中断，禁止补发积累的移动目标")
        self.last_time = now
        x, y, z = self.position
        yaw_error = atan2(sin(self.yaw-radians(t.yaw_deg)),cos(self.yaw-radians(t.yaw_deg)))
        if (hypot(x-t.local_x_m, y-t.local_y_m) > lim.lead_hard
                or abs(z-t.local_z_m) > lim.vertical_lead_hard
                or abs(yaw_error) > radians(lim.yaw_lead_hard_deg)):
            raise HandoffFault("实际位置未跟上目标，停止轨迹积分并要求接管")
        forward, right, up, turn = axes
        norm = max(1.0, hypot(forward, right))
        heading = radians(t.yaw_deg)
        desired = ((cos(heading)*forward-sin(heading)*right)*lim.max_speed/norm,
                   (sin(heading)*forward+cos(heading)*right)*lim.max_speed/norm,
                   -up*lim.max_vertical_speed)
        # Start decelerating before target lead/geofence budgets are exhausted.
        speed = hypot(*self.velocity[:2])
        reserve = speed*speed/(2*lim.acceleration) + 0.05
        near_edge = hypot(x-self.origin_xy[0], y-self.origin_xy[1]) + reserve >= self.radius
        lead = hypot(x-t.local_x_m, y-t.local_y_m)
        if lead + reserve >= lim.lead_soft:
            desired = _remove_outward(desired,(x-t.local_x_m,y-t.local_y_m))
        if near_edge:
            desired = _remove_outward(desired,(x-self.origin_xy[0],y-self.origin_xy[1]))
        altitude = self.ground_z-z
        vertical_reserve = self.velocity[2]**2/(2*lim.vertical_acceleration)+0.05
        if (desired[2] < 0 and altitude+vertical_reserve >= 3.0) or (desired[2] > 0 and altitude-vertical_reserve <= 1.0):
            desired = (*desired[:2], 0.0)
        if desired[2]*(z-t.local_z_m)>0 and abs(z-t.local_z_m)+vertical_reserve >= lim.vertical_lead_soft:
            desired = (*desired[:2],0.0)
        dx, dy = desired[0]-self.velocity[0], desired[1]-self.velocity[1]
        scale = min(1.0, lim.acceleration*dt/max(hypot(dx, dy), 1e-12))
        velocity = (self.velocity[0]+dx*scale, self.velocity[1]+dy*scale,
                    _slew(self.velocity[2], desired[2], lim.vertical_acceleration*dt))
        candidate = tuple(p+(a+b)*dt/2 for p,a,b in zip(self.position,self.velocity,velocity))
        if (hypot(candidate[0]-self.origin_xy[0], candidate[1]-self.origin_xy[1]) > self.radius
            or not 1.0-1e-6 <= self.ground_z-candidate[2] <= 3.0+1e-6
            or hypot(candidate[0]-t.local_x_m,candidate[1]-t.local_y_m) > lim.lead_hard
            or abs(candidate[2]-t.local_z_m) > lim.vertical_lead_hard):
            raise HandoffFault("轨迹越过位置/高度/目标偏差边界，拒绝发送")
        desired_yaw_rate = turn*radians(lim.max_yaw_rate)
        yaw_reserve = self.yaw_rate**2/(2*radians(lim.yaw_acceleration))+radians(2.)
        if desired_yaw_rate*yaw_error>0 and abs(yaw_error)+yaw_reserve>=radians(lim.yaw_lead_soft_deg):
            desired_yaw_rate=0.0
        yaw_rate = _slew(self.yaw_rate,desired_yaw_rate,radians(lim.yaw_acceleration)*dt)
        self.yaw += (self.yaw_rate+yaw_rate)*dt/2
        self.yaw = atan2(sin(self.yaw),cos(self.yaw))
        self.position, self.velocity, self.yaw_rate = candidate, velocity, yaw_rate
        if any(abs(v)>1e-6 for v in axes):
            self.phase = "MOVING" if any(abs(v)>1e-6 for v in (*desired,turn)) else "LIMIT_BRAKING"
        elif any(abs(v)>1e-6 for v in (*velocity,yaw_rate)):
            self.phase = "BRAKING"
        else:
            self.phase = "POSITION_HOLD"
        return (*self.position, self.yaw, *self.velocity)


class SmoothHandoff:
    def __init__(self, target, ground_z, origin_xy, radius, limits=None, frame_signature=None):
        self.limits = limits or HandoffLimits()
        self.target = tuple(target)
        self.ground_z, self.origin_xy, self.radius = ground_z, origin_xy, radius
        self.run_id = token_urlsafe(18)
        self.client = None
        self.sequence = -1
        self.last_input = None
        self.foreground = False
        self.keys_released = False
        self.axes = (0.0,)*4
        self.token = None
        self.awaiting_neutral = False
        self.good_since = self.last_observe = None
        self.last_sample = None
        self.last_sample_received = None
        self.samples = 0
        self.signature = frame_signature
        self.reference = None
        self.reason = "等待悬停及完整定位证据"
        self.terminal = False
        self.revision = 0
        self.checks = {}

    @property
    def authorized(self):
        return self.token is not None and not self.terminal

    def cancel(self, reason):
        self.revoke(reason)
        self.terminal = True

    def revoke(self, reason):
        self.revision += 1
        self.token = None
        self.axes = (0.0,)*4
        self.awaiting_neutral = True
        self.good_since = None
        self.samples = 0
        self.reason = reason

    def report(self, run_id, client, sequence, axes, foreground, confirmed, token, now, keys_released):
        if self.terminal or run_id != self.run_id:
            raise ValueError("交接会话已失效，必须重新人工启动")
        if not client or type(sequence) is not int or sequence < 0:
            raise ValueError("无效的网页会话或序号")
        if self.authorized and (client != self.client or token != self.token):
            raise ValueError("该网页未获键盘控制权")
        if client != self.client:
            if self.last_input is not None and now-self.last_input <= self.limits.input_timeout:
                raise ValueError("另一个网页正在准备交接")
            self.client, self.sequence = client, -1
            self.good_since, self.samples = None, 0
        if sequence <= self.sequence:
            raise ValueError("拒绝重复或乱序的键盘输入")
        if len(axes) != 4 or any(not isfinite(v) or not -1 <= v <= 1 for v in axes) or (keys_released is True and any(axes)):
            self.revoke("非法轴输入，撤销键盘授权")
            raise ValueError(self.reason)
        self.sequence, self.last_input = sequence, now
        self.foreground = foreground is True and confirmed is True
        self.keys_released = keys_released is True
        if not self.foreground:
            self.revoke("网页失焦或撤销现场确认：减速并保持，需重新授权")
            return
        if self.awaiting_neutral:
            if any(axes) or not self.keys_released:
                self.axes = (0.0,)*4
                raise ValueError("授权后必须先松开全部按键")
            self.awaiting_neutral = False
        self.axes = tuple(axes)
        if not self.authorized and (any(axes) or not self.keys_released):
            self.good_since, self.samples = None, 0

    def check_frame(self, t, now):
        """Called before every position packet, including prestream and hover."""
        reason = reset_evidence_reason(t,now)
        if self.terminal:
            raise HandoffFault(self.reason)
        if reason or (self.signature is not None and self.signature != t.estimator_reset_signature):
            self.cancel(reason or "估计器实例或重置计数变化，旧坐标目标作废")
            raise HandoffFault(self.reason)
        self.signature = t.estimator_reset_signature

    def health_reason(self, t, now):
        reason = detailed_health_reason(t, now)
        if reason:
            return reason
        if t.armed is not True or t.landed_state != "IN_AIR" or t.flight_mode.upper() != "OFFBOARD":
            return "必须ARMED + IN_AIR + OFFBOARD"
        if t.rc_channel_6_pwm >= 1800 or t.rc_channel_8_pwm < 1800:
            return "实体急停/模式开关不允许电脑控制"
        return ""

    def observe(self, t, now):
        self.check_frame(t,now)
        reason = self.health_reason(t, now)
        gap = self.last_observe is not None and not 0 <= now-self.last_observe <= self.limits.max_tick_gap
        self.last_observe = now
        # Test actual observations, not how often the HTTP/poll loop ran.
        # Source time also rejects old buffered samples delivered in a burst.
        distinct = t.local_position_sample_id != self.last_sample
        sample_gap = (t.last_distinct_position_monotonic is None
                      or not 0 <= now-t.last_distinct_position_monotonic <= self.limits.max_tick_gap)
        if distinct and self.last_sample is not None and type(t.local_position_sample_id) is int:
            source_dt = ((t.local_position_sample_id-self.last_sample)&0xFFFFFFFF)/1000.0
            sample_gap = sample_gap or not 0 < source_dt <= self.limits.max_tick_gap
            if self.last_sample_received is not None and t.last_distinct_position_monotonic is not None:
                sample_gap = sample_gap or not 0 < t.last_distinct_position_monotonic-self.last_sample_received <= self.limits.max_tick_gap
        if distinct:
            self.last_sample_received = t.last_distinct_position_monotonic
        if self.reference is not None and (reason or gap or sample_gap):
            self.cancel(reason or ("本地位置实际采样间隔中断" if sample_gap else "控制循环中断"))
            raise HandoffFault(self.reason)
        if self.reference is not None:
            self.target = (*self.reference.position, self.reference.yaw)
        browser_ok = self.foreground and self.last_input is not None and 0 <= now-self.last_input <= self.limits.input_timeout
        if self.authorized and not browser_ok:
            self.revoke("网页输入中断：撤销权限，减速后保持；不自动恢复")
        x,y,z,_ = self.target
        self.checks = {}
        if not reason:
            self.checks = {
                "horizontal_error_m":hypot(t.local_x_m-x,t.local_y_m-y),
                "vertical_error_m":abs(t.local_z_m-z),
                "horizontal_speed_m_s":hypot(t.vx_m_s,t.vy_m_s),
                "vertical_speed_m_s":abs(t.vz_m_s),
                "tilt_deg":max(abs(t.roll_deg),abs(t.pitch_deg)),
            }
        stable = (not reason and hypot(t.local_x_m-x,t.local_y_m-y) <= self.limits.horizontal_error
                  and abs(t.local_z_m-z) <= self.limits.vertical_error
                  and hypot(t.vx_m_s,t.vy_m_s) <= self.limits.horizontal_stable_speed
                  and abs(t.vz_m_s) <= self.limits.vertical_stable_speed
                  and max(abs(t.roll_deg),abs(t.pitch_deg)) <= 5.0
                  and (self.reference is None or not any(abs(v)>1e-6 for v in (*self.reference.velocity,self.reference.yaw_rate))))
        if not stable or not browser_ok or not self.keys_released or any(self.axes) or gap or sample_gap or self.terminal:
            self.good_since, self.samples = None, 0
            details = []
            for field,limit,label in (
                ("horizontal_error_m",self.limits.horizontal_error,"水平位置误差"),
                ("vertical_error_m",self.limits.vertical_error,"高度误差"),
                ("horizontal_speed_m_s",self.limits.horizontal_stable_speed,"水平速度"),
                ("vertical_speed_m_s",self.limits.vertical_stable_speed,"垂直速度"),
                ("tilt_deg",5.,"倾角"),
            ):
                if self.checks.get(field,0)>limit:details.append(label+"超出授权范围")
            if not browser_ok:details.append("网页须保持前台、现场确认且输入报告新鲜")
            if not self.keys_released or any(self.axes):details.append("请松开全部按键")
            if gap:details.append("稳定观测曾中断，重新计时")
            if sample_gap:details.append("真实位置样本间隔超过250毫秒，重新计时")
            self.reason = reason or "；".join(details) or "轨迹尚未停止，等待减速完成"
        else:
            if self.good_since is None:
                self.good_since = now
            if self.last_sample != t.local_position_sample_id:
                self.samples += 1
            self.reason = "" if self.ready(now) else "持续验证稳定中（至少3秒及15个不同位置样本）"
        self.last_sample = t.local_position_sample_id

    def ready(self, now):
        return not self.terminal and self.good_since is not None and now-self.good_since >= self.limits.stable_seconds and self.samples >= 15

    def authorize(self, client, t, now):
        if self.authorized:
            raise ValueError("键盘已授权，拒绝重复交接")
        self.observe(t, now)
        if client != self.client or not self.ready(now):
            raise ValueError(self.reason or "该网页未完成授权准备")
        if self.reference is None:
            # Exactly the existing hover target, NOT current measured position.
            self.reference = ContinuousReference(self.target,self.ground_z,self.origin_xy,self.radius,self.limits)
            self.reference.last_time = now
        self.token = token_urlsafe(24)
        self.revision += 1
        self.awaiting_neutral = True
        self.axes = (0.0,)*4
        self.reason = "键盘待命，原位置目标保持不变；先松键再操作"
        return self.token

    def step(self, t, now):
        self.observe(t,now)
        if self.reference is None:
            return (*self.target,0.0,0.0,0.0)
        return self.reference.step(self.axes if self.authorized and not self.awaiting_neutral else (0.0,)*4,t,now)

    def snapshot(self, now):
        return {"run_id":self.run_id,"authorized":self.authorized,"ready":self.ready(now) and not self.authorized,
                "revision":self.revision,
                "checks":self.checks.copy(),"position_samples":self.samples,
                "reason":self.reason,"phase":self.reference.phase if self.reference else "HOLDING",
                "awaiting_neutral":self.awaiting_neutral,"terminal":self.terminal,
                "stable_seconds":0.0 if self.good_since is None else max(0.0,now-self.good_since),
                "target":self.target,"reference":None if self.reference is None else (*self.reference.position,self.reference.yaw),
                "velocity":None if self.reference is None else self.reference.velocity}
