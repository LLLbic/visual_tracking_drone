from __future__ import annotations

from dataclasses import replace
from math import hypot
import os
from pathlib import Path
from threading import Condition, Event, Lock, Thread
from time import monotonic, sleep
from typing import Iterator

from .config import AppConfig
from .controller import PreviewController
from .telemetry import PassiveMavlinkReceiver
from .types import CommandPreview, DetectedTrack, TargetSnapshot, VisionSnapshot
from .vision import SingleObjectTracker, UltralyticsTrackDetector, choose_reacquisition_candidate


class VideoTrackingEngine:
    def __init__(
        self,
        config: AppConfig,
        telemetry: PassiveMavlinkReceiver,
        controller: PreviewController,
    ) -> None:
        self.config = config
        self.telemetry = telemetry
        self.controller = controller
        self.detector = UltralyticsTrackDetector(config.vision, config.vision_models)
        self._stop = Event()
        self._condition = Condition()
        self._state_lock = Lock()
        self._thread: Thread | None = None
        self._jpeg: bytes | None = None
        self._frame_serial = 0
        self._vision = VisionSnapshot(source=config.video.source)
        self._preview = CommandPreview()
        self._locked_track: DetectedTrack | None = None
        self._requested_click: tuple[float, float] | None = None
        self._frame_counter = 0
        self._lost_frames = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = Thread(target=self._run, name="video-tracking", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def select_target(self, x_normalized: float, y_normalized: float) -> None:
        if not 0.0 <= x_normalized <= 1.0 or not 0.0 <= y_normalized <= 1.0:
            raise ValueError("target coordinates must be normalized to 0..1")
        with self._state_lock:
            self._requested_click = (x_normalized, y_normalized)

    def clear_target(self) -> None:
        with self._state_lock:
            self._locked_track = None
            self._requested_click = None
            self._lost_frames = 0
            self._vision.target = TargetSnapshot()

    def select_model(self, profile_id: str) -> dict[str, object]:
        self.clear_target()
        return self.detector.request_profile(profile_id)

    def model_status(self) -> dict[str, object]:
        return self.detector.status()

    def snapshot(self) -> tuple[VisionSnapshot, CommandPreview]:
        with self._state_lock:
            vision = replace(
                self._vision,
                tracks=[replace(track) for track in self._vision.tracks],
                target=replace(self._vision.target),
            )
            preview = replace(self._preview)
        return vision, preview

    def mjpeg(self) -> Iterator[bytes]:
        last_serial = -1
        while not self._stop.is_set():
            with self._condition:
                self._condition.wait_for(
                    lambda: self._frame_serial != last_serial or self._stop.is_set(),
                    timeout=1.0,
                )
                jpeg = self._jpeg
                last_serial = self._frame_serial
            if jpeg:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"

    def _publish_jpeg(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg = jpeg
            self._frame_serial += 1
            self._condition.notify_all()

    def _capture_source(self) -> int | str:
        value = self.config.video.source.strip()
        return int(value) if value.isdigit() else value

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            with self._state_lock:
                self._vision.detector_error = f"OpenCV/Numpy 加载失败：{exc}"
            return

        transport = self.config.video.transport.casefold()
        if transport == "tcp":
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                "rtsp_transport;tcp|timeout;6000000|fflags;nobuffer|"
                "flags;low_delay|max_delay;0|reorder_queue_size;0"
            )
            read_timeout_ms = 6000
        else:
            # UDP needs FFmpeg's small receive/reorder buffer. Reusing the TCP
            # zero-buffer options makes this 5 FPS camera time out repeatedly.
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                "rtsp_transport;udp|timeout;12000000"
            )
            read_timeout_ms = 12000
        ultralytics_dir = Path.cwd() / ".runtime" / "ultralytics"
        ultralytics_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("YOLO_CONFIG_DIR", str(ultralytics_dir))
        detector = self.detector
        single_tracker = SingleObjectTracker(self.config.vision)
        with self._state_lock:
            self._vision.detector_ready = detector.ready
            self._vision.detector_error = detector.error

        while not self._stop.is_set():
            capture_parameters = [
                int(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC),
                5000,
                int(cv2.CAP_PROP_READ_TIMEOUT_MSEC),
                read_timeout_ms,
            ]
            capture = cv2.VideoCapture(
                self._capture_source(),
                cv2.CAP_FFMPEG,
                capture_parameters,
            )
            if not capture.isOpened():
                with self._state_lock:
                    self._vision.connected = False
                    self._vision.capture_error = "无法打开RTSP视频流，正在重连"
                    self._vision.reconnect_count += 1
                    has_previous_frame = self._vision.last_frame_monotonic is not None
                if not has_previous_frame:
                    self._show_placeholder(cv2, np, "WAITING FOR RTSP VIDEO", "Check camera power, link and RTSP address")
                sleep(self.config.video.reconnect_seconds)
                continue
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            with self._state_lock:
                self._vision.capture_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)

            started = monotonic()
            processed = 0
            while not self._stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    with self._state_lock:
                        self._vision.connected = False
                        self._vision.capture_error = "RTSP读取超时或数据中断，正在重连"
                        self._vision.reconnect_count += 1
                    break
                self._process_frame(cv2, frame, detector, single_tracker)
                processed += 1
                elapsed = monotonic() - started
                if elapsed >= 1.0:
                    with self._state_lock:
                        self._vision.processing_fps = processed / elapsed
                    processed = 0
                    started = monotonic()
            capture.release()
            single_tracker.clear()
            with self._state_lock:
                self._vision.connected = False
                has_previous_frame = self._vision.last_frame_monotonic is not None
            if not self._stop.is_set():
                # Preserve the latest valid frame during a transient reconnect.
                # Replacing it with a placeholder made a short UDP timeout look
                # like a total video failure in the browser.
                if not has_previous_frame:
                    self._show_placeholder(cv2, np, "RTSP INTERRUPTED - RECONNECTING", self.config.video.source)
                sleep(self.config.video.reconnect_seconds)

    def _process_frame(self, cv2: object, frame: object, detector: UltralyticsTrackDetector, single_tracker: SingleObjectTracker) -> None:
        height, width = frame.shape[:2]
        self._frame_counter += 1
        detector_frame = self._frame_counter % self.config.vision.detector_every_n_frames == 0
        tracks: list[DetectedTrack]
        with self._state_lock:
            tracks = [replace(track) for track in self._vision.tracks]
            click = self._requested_click
            self._requested_click = None
            locked = replace(self._locked_track) if self._locked_track is not None else None

        if detector_frame:
            tracks = detector.detect_and_track(frame)
            with self._state_lock:
                self._vision.detector_ready = detector.ready
                self._vision.detector_error = detector.error

        if click is not None and tracks:
            clicked_x, clicked_y = click[0] * width, click[1] * height
            containing = [
                track for track in tracks if track.x1 <= clicked_x <= track.x2 and track.y1 <= clicked_y <= track.y2
            ]
            pool = containing or tracks
            locked = min(pool, key=lambda track: hypot(track.center[0] - clicked_x, track.center[1] - clicked_y))
            single_tracker.initialize(cv2, frame, locked)

        target_source = "none"
        target_fresh = False
        if locked is not None:
            matched = next((track for track in tracks if track.track_id == locked.track_id), None) if detector_frame else None
            if matched is None and detector_frame:
                matched = choose_reacquisition_candidate(
                    locked,
                    tracks,
                    width,
                    height,
                    self.config.vision.reacquire_max_center_distance,
                    self.config.vision.reacquire_max_height_change,
                )
            if matched is not None:
                locked = matched
                single_tracker.initialize(cv2, frame, locked)
                target_source = "YOLO + ByteTrack"
                target_fresh = True
            else:
                updated = single_tracker.update(
                    frame,
                    locked.track_id,
                    locked.class_name,
                    locked.confidence,
                )
                if updated is not None:
                    locked = updated
                    target_source = self.config.vision.single_object_tracker.upper()
                    target_fresh = True

        if target_fresh:
            self._lost_frames = 0
        elif locked is not None:
            self._lost_frames += 1
            if self._lost_frames > self.config.vision.max_target_lost_frames:
                locked = None
                single_tracker.clear()
                self._lost_frames = 0

        target = self._target_snapshot(
            locked,
            width,
            height,
            target_source,
            target_fresh,
            self._lost_frames,
        )
        telemetry = self.telemetry.snapshot()
        preview = self.controller.compute(
            target,
            telemetry,
            self.config.telemetry.stale_after_seconds,
        )
        self._draw_overlay(cv2, frame, tracks, locked, target, preview)
        encode_ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.video.jpeg_quality)],
        )
        if encode_ok:
            self._publish_jpeg(encoded.tobytes())

        with self._state_lock:
            self._locked_track = replace(locked) if locked is not None else None
            self._vision.connected = True
            self._vision.capture_error = ""
            self._vision.width = width
            self._vision.height = height
            self._vision.last_frame_monotonic = monotonic()
            self._vision.tracks = [replace(track) for track in tracks]
            self._vision.target = target
            self._vision.tracker_error = single_tracker.error
            self._preview = preview

    @staticmethod
    def _target_snapshot(
        locked: DetectedTrack | None,
        width: int,
        height: int,
        source: str,
        fresh: bool,
        lost_frames: int,
    ) -> TargetSnapshot:
        if locked is None or width <= 0 or height <= 0:
            return TargetSnapshot()
        cx, _ = locked.center
        return TargetSnapshot(
            locked=fresh,
            track_id=locked.track_id,
            class_name=locked.class_name,
            confidence=locked.confidence,
            bbox=(locked.x1, locked.y1, locked.x2, locked.y2),
            center_error_x=(cx - width / 2.0) / (width / 2.0),
            bbox_height_ratio=locked.height / height,
            source=source if fresh else "lost",
            lost_frames=lost_frames,
        )

    @staticmethod
    def _draw_overlay(cv2: object, frame: object, tracks: list[DetectedTrack], locked: DetectedTrack | None, target: TargetSnapshot, preview: CommandPreview) -> None:
        height, width = frame.shape[:2]
        cv2.line(frame, (width // 2, 0), (width // 2, height), (80, 80, 80), 1)
        for track in tracks:
            color = (70, 220, 120)
            cv2.rectangle(frame, (int(track.x1), int(track.y1)), (int(track.x2), int(track.y2)), color, 2)
            cv2.putText(
                frame,
                (
                    f"{track.class_name} #{track.track_id} {track.confidence:.2f}"
                    + (f" red={track.red_ratio:.0%}" if track.red_ratio is not None else "")
                ),
                (int(track.x1), max(22, int(track.y1) - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        if locked is not None:
            cv2.rectangle(frame, (int(locked.x1), int(locked.y1)), (int(locked.x2), int(locked.y2)), (0, 210, 255), 3)
            cx, cy = locked.center
            cv2.circle(frame, (int(cx), int(cy)), 6, (0, 210, 255), -1)
        cv2.putText(
            frame,
            f"PREVIEW forward={preview.raw_forward_m_s:+.2f}m/s yaw={preview.raw_yaw_rate_deg_s:+.1f}deg/s",
            (18, height - 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 210, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "TX=OFF  RC/PX4 unchanged",
            (18, height - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (40, 40, 255),
            2,
            cv2.LINE_AA,
        )

    def _show_placeholder(self, cv2: object, np: object, title: str, detail: str) -> None:
        frame = np.zeros((540, 960, 3), dtype=np.uint8)
        frame[:] = (22, 28, 38)
        cv2.putText(frame, title, (52, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 210, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, detail[:100], (52, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (210, 220, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, "TRACKING TX OFF - GROUND ZERO-VELOCITY TEST ONLY", (52, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (60, 80, 255), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if ok:
            self._publish_jpeg(encoded.tobytes())
