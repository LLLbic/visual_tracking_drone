"""Raw MAVLink flow-source accounting; never infer EKF fusion from quality."""
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class FlowSample:
    source: str
    sensor_id: int
    sample_us: int
    quality: int
    received: float
    integration_us: int | None
    ground_distance_m: float | None


class FlowMonitor:
    def __init__(self, minimum_quality=100, sensor_id=0, message_type="OPTICAL_FLOW_RAD"):
        self.minimum_quality = minimum_quality
        self.sensor_id = sensor_id
        self.message_type = message_type
        self.samples = {}
        self.good_since = None
        self.good_samples = 0
        self.last_bad = None
        self.selected = None
        self.last_error = ""
        self.rate_hz = None

    def _fault(self, reason, now):
        self.good_since, self.good_samples, self.last_bad = None, 0, now
        self.last_error = reason
        return None, reason

    def accept(self, message, now):
        kind = message.get_type()
        sensor = getattr(message, 'sensor_id', None)
        stamp = getattr(message, 'time_usec', None)
        quality = getattr(message, 'quality', None)
        source = f"{message.get_srcSystem()}/{message.get_srcComponent()}/{kind}/{sensor}"
        if (type(sensor) is not int or not 0 <= sensor <= 255 or type(stamp) is not int
                or not 0 <= stamp <= 0xFFFFFFFFFFFFFFFF or type(quality) is not int or not 0 <= quality <= 255):
            return self._fault("光流报文缺少有效的源时间、传感器编号或原始质量值", now)
        old = self.samples.get(source)
        if old and stamp == old.sample_us:
            return None, ""  # Replay cannot keep a dead flow stream fresh.
        if old and stamp < old.sample_us:
            return self._fault("光流源时间倒退，需核验传感器/飞控重启", now)
        integration = getattr(message, 'integration_time_us', None)
        distance = getattr(message, 'distance', getattr(message, 'ground_distance', None))
        if distance is not None:
            try:
                distance = float(distance)
            except (ValueError, TypeError):
                distance = None
        if distance is not None:
            if not isfinite(distance) or distance < 0:
                distance = None  # Not the separate down-facing rangefinder.
        sample = FlowSample(source, sensor, stamp, quality, now, integration, distance)
        # Cap diagnostics storage; no raw image, full datagram or arbitrary text.
        if source not in self.samples and len(self.samples) >= 8:
            return self._fault("光流来源过多，禁止自动选择", now)
        self.samples[source] = sample
        if kind != self.message_type or sensor != self.sensor_id:
            return None, ""  # Other topics cannot overwrite the configured source.
        if self.selected is not None and self.selected != source:
            return self._fault("光流源身份变化，禁止自动切换来源", now)
        self.selected = source
        gap = None if old is None else now-old.received
        if gap is not None and 0 < gap <= 10:
            rate = min(100.,1/gap)
            self.rate_hz = rate if self.rate_hz is None else .65*self.rate_hz+.35*rate
        if quality < self.minimum_quality:
            self.good_since, self.good_samples, self.last_bad = None, 0, now
            self.last_error = f"光流质量 {quality} 低于本地授权门槛 {self.minimum_quality}"
        else:
            source_gap = None if old is None else (stamp-old.sample_us)/1e6
            if (self.good_since is None or gap is None or not 0 < gap <= .5
                    or source_gap is None or not 0 < source_gap <= .5):
                self.good_since, self.good_samples = now, 0
            self.good_samples += 1
            self.last_error = ""
        return sample, self.last_error

    def diagnostics(self, now):
        return {source: {
            'sensor_id': sample.sensor_id, 'quality_raw': sample.quality,
            'sample_time_us': sample.sample_us, 'age_seconds': max(0.,now-sample.received),
            'integration_time_us': sample.integration_us,
            'ground_distance_m': sample.ground_distance_m,
            'selected': source == self.selected,
        } for source,sample in self.samples.items()}
