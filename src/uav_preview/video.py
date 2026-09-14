from __future__ import annotations

from collections import deque
from dataclasses import replace
from math import hypot
import os
from pathlib import Path
import shutil
import subprocess
from threading import Condition, Event, Lock, Thread
from time import monotonic, sleep
from typing import Iterator

from .config import AppConfig
from .controller import PreviewController
from .frame_interpolation import InterpolatedFrame, NvidiaFrucInterpolator
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
        self._capture_thread: Thread | None = None
        self._ffmpeg_process_lock = Lock()
        self._ffmpeg_process: subprocess.Popen[bytes] | None = None
        self._jpeg: bytes | None = None
        self._frame_serial = 0
        self._raw_condition = Condition()
        self._raw_frame: object | None = None
        self._raw_frame_serial = 0
        self._raw_generation = 0
        self._vision = VisionSnapshot(
            source=config.video.source,
            capture_backend=config.video.capture_backend,
            video_decoder=config.video.ffmpeg_decoder,
            interpolation_enabled=config.frame_interpolation.enabled,
            interpolation_backend=(
                config.frame_interpolation.backend if config.frame_interpolation.enabled else ""
            ),
            display_mode=(
                "latest-real-frame"
                if config.video.low_latency_latest_frame
                else "ordered-interpolated"
            ),
        )
        self._preview = CommandPreview()
        self._locked_track: DetectedTrack | None = None
        self._requested_click: tuple[float, float] | None = None
        self._frame_counter = 0
        self._lost_frames = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._capture_thread = Thread(
            target=self._capture_loop,
            name="video-capture-latest-frame",
            daemon=True,
        )
        self._thread = Thread(target=self._run, name="video-tracking", daemon=True)
        self._capture_thread.start()
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        with self._raw_condition:
            self._raw_condition.notify_all()
        self._terminate_ffmpeg_process()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=3.0)

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
                    lambda: (
                        self._jpeg is not None and self._frame_serial != last_serial
                    )
                    or self._stop.is_set(),
                    timeout=1.0,
                )
                # Never replay a backlog. A slow/new browser always receives
                # the newest completed JPEG and skips every stale display frame.
                jpeg = self._jpeg
                last_serial = self._frame_serial
            if jpeg:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n".encode("ascii")
                    + b"Cache-Control: no-store\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )

    def wait_for_jpeg(
        self, after_serial: int = -1, timeout: float = 1.0
    ) -> tuple[int, bytes | None]:
        """Long-poll the latest JPEG without ever returning queued history."""
        with self._condition:
            self._condition.wait_for(
                lambda: (
                    self._jpeg is not None and self._frame_serial != after_serial
                )
                or self._stop.is_set(),
                timeout=timeout,
            )
            if self._jpeg is None or self._frame_serial == after_serial:
                return self._frame_serial, None
            return self._frame_serial, self._jpeg

    def _publish_jpeg(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg = jpeg
            self._frame_serial += 1
            self._condition.notify_all()

    def _capture_source(self) -> int | str:
        value = self.config.video.source.strip()
        return int(value) if value.isdigit() else value

    def _configure_capture_options(self) -> tuple[int, float]:
        transport = self.config.video.transport.casefold()
        if transport == "tcp":
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                "rtsp_transport;tcp|timeout;6000000|fflags;nobuffer|"
                "flags;low_delay|max_delay;0|reorder_queue_size;0"
            )
            return 6000, 0.0

        timeout_us = self.config.video.udp_read_timeout_ms * 1000
        max_delay_us = self.config.video.udp_max_delay_ms * 1000
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;udp|"
            f"buffer_size;{self.config.video.udp_buffer_size_bytes}|"
            f"reorder_queue_size;{self.config.video.udp_reorder_queue_size}|"
            f"max_delay;{max_delay_us}|timeout;{timeout_us}|"
            "fflags;nobuffer|flags;low_delay"
        )
        return (
            self.config.video.udp_read_timeout_ms,
            self.config.video.udp_failure_grace_seconds,
        )

    def _capture_loop(self) -> None:
        if self.config.video.capture_backend == "ffmpeg":
            self._capture_loop_ffmpeg()
            return
        self._capture_loop_opencv()

    def _ffmpeg_capture_command(self, decoder: str | None = None) -> list[str]:
        executable = shutil.which(self.config.video.ffmpeg_path)
        if executable is None:
            configured = Path(self.config.video.ffmpeg_path).expanduser()
            if not configured.is_file():
                raise FileNotFoundError(
                    f"找不到FFmpeg：{self.config.video.ffmpeg_path}"
                )
            executable = str(configured.resolve())

        selected_decoder = decoder or self.config.video.ffmpeg_decoder
        command = [
            executable,
            "-hide_banner",
            "-nostdin",
            "-loglevel",
            "warning",
        ]
        if self.config.video.source.casefold().startswith("rtsp://"):
            command.extend(
                [
                    "-rtsp_transport",
                    self.config.video.transport,
                    "-fflags",
                    "nobuffer",
                    "-flags",
                    "low_delay",
                    "-avioflags",
                    "direct",
                    "-probesize",
                    "32",
                    "-analyzeduration",
                    "0",
                    # RTSP demuxer's socket timeout also bounds stalled RTP.
                    # rw_timeout alone does not reliably interrupt RTSP reads.
                    "-timeout",
                    str(
                        int(
                            max(
                                self.config.video.udp_read_timeout_ms / 1000.0,
                                self.config.video.udp_failure_grace_seconds,
                            )
                            * 1_000_000
                        )
                    ),
                ]
            )
            if self.config.video.transport == "udp":
                command.extend(
                    [
                        "-buffer_size",
                        str(self.config.video.udp_buffer_size_bytes),
                        "-reorder_queue_size",
                        str(self.config.video.udp_reorder_queue_size),
                        "-max_delay",
                        str(self.config.video.udp_max_delay_ms * 1000),
                    ]
                )
        if selected_decoder != "software":
            command.extend(["-c:v", selected_decoder])
        else:
            # Avoid frame-threaded software decode accumulating a frame queue.
            command.extend(["-threads:v", "1"])
        command.extend(
            [
                "-i",
                self.config.video.source,
                "-map",
                "0:v:0",
                "-an",
                "-sn",
                "-dn",
                "-vf",
                (
                    f"scale={self.config.video.frame_width}:"
                    f"{self.config.video.frame_height}:flags=fast_bilinear"
                ),
                "-pix_fmt",
                "bgr24",
                "-fps_mode",
                "passthrough",
                # The rawvideo encoder otherwise may frame-thread and retain
                # multiple frames even though Python only keeps the latest one.
                "-threads:v",
                "1",
                "-f",
                "rawvideo",
                "-flush_packets",
                "1",
                "pipe:1",
            ]
        )
        return command

    @staticmethod
    def _read_exact(stream: object, size: int) -> bytes | None:
        data = bytearray()
        while len(data) < size:
            chunk = stream.read(size - len(data))  # type: ignore[attr-defined]
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _drain_ffmpeg_stderr(
        process: subprocess.Popen[bytes], lines: deque[str]
    ) -> None:
        if process.stderr is None:
            return
        for raw_line in iter(process.stderr.readline, b""):
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)

    @staticmethod
    def _nvdec_failed(stderr_text: str) -> bool:
        lowered = stderr_text.casefold()
        return any(
            marker in lowered
            for marker in (
                "cannot load libnvcuvid",
                "no device available for decoder",
                "failed setup for format cuda",
                "error while opening decoder",
                "unknown decoder",
            )
        )

    def _terminate_ffmpeg_process(self) -> None:
        with self._ffmpeg_process_lock:
            process = self._ffmpeg_process
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def _capture_loop_ffmpeg(self) -> None:
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            with self._state_lock:
                self._vision.capture_error = f"OpenCV/Numpy加载失败：{exc}"
            return

        requested_decoder = self.config.video.ffmpeg_decoder
        active_decoder = requested_decoder
        frame_size = (
            self.config.video.frame_width
            * self.config.video.frame_height
            * 3
        )
        reconnect_delay = min(0.5, self.config.video.reconnect_seconds)
        generation = 0

        while not self._stop.is_set():
            try:
                command = self._ffmpeg_capture_command(active_decoder)
                process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
            except Exception as exc:
                with self._state_lock:
                    self._vision.connected = False
                    self._vision.capture_error = f"FFmpeg启动失败：{exc}"
                    self._vision.reconnect_count += 1
                sleep(reconnect_delay)
                reconnect_delay = min(
                    self.config.video.reconnect_seconds,
                    max(0.5, reconnect_delay * 2.0),
                )
                continue

            with self._ffmpeg_process_lock:
                self._ffmpeg_process = process
            stderr_lines: deque[str] = deque(maxlen=12)
            stderr_thread = Thread(
                target=self._drain_ffmpeg_stderr,
                args=(process, stderr_lines),
                name="ffmpeg-stderr",
                daemon=True,
            )
            stderr_thread.start()
            generation += 1
            received_frames = 0
            rate_frames = 0
            rate_started = monotonic()
            with self._state_lock:
                self._vision.video_decoder = active_decoder

            if process.stdout is not None:
                while not self._stop.is_set():
                    raw_frame = self._read_exact(process.stdout, frame_size)
                    if raw_frame is None:
                        break
                    frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(
                        self.config.video.frame_height,
                        self.config.video.frame_width,
                        3,
                    ).copy()
                    received_frames += 1
                    rate_frames += 1
                    reconnect_delay = 0.5
                    with self._raw_condition:
                        self._raw_frame = frame
                        self._raw_frame_serial += 1
                        self._raw_generation = generation
                        self._raw_condition.notify_all()
                    elapsed = monotonic() - rate_started
                    if elapsed >= 1.0:
                        with self._state_lock:
                            self._vision.capture_fps = rate_frames / elapsed
                        rate_frames = 0
                        rate_started = monotonic()

            self._terminate_ffmpeg_process()
            stderr_thread.join(timeout=0.5)
            if process.stdout is not None:
                process.stdout.close()
            if process.stderr is not None and not stderr_thread.is_alive():
                process.stderr.close()
            with self._ffmpeg_process_lock:
                if self._ffmpeg_process is process:
                    self._ffmpeg_process = None
            if self._stop.is_set():
                break

            stderr_text = " | ".join(stderr_lines)
            if (
                received_frames == 0
                and active_decoder.endswith("_cuvid")
                and self._nvdec_failed(stderr_text)
            ):
                active_decoder = "software"
                with self._state_lock:
                    self._vision.capture_error = (
                        "NVDEC启动失败，已自动回退CPU解码：" + stderr_text[-240:]
                    )
                    self._vision.soft_read_failures += 1
                sleep(0.1)
                continue

            with self._state_lock:
                self._vision.connected = False
                self._vision.soft_read_failures += 1
                self._vision.reconnect_count += 1
                self._vision.capture_error = (
                    "FFmpeg视频输入中断，正在重连"
                    + (f"：{stderr_text[-300:]}" if stderr_text else "")
                )
                has_previous_frame = self._vision.last_frame_monotonic is not None
            if not has_previous_frame:
                self._show_placeholder(
                    cv2,
                    np,
                    "WAITING FOR LOW-LATENCY FFMPEG VIDEO",
                    self.config.video.source,
                )
            sleep(reconnect_delay)
            reconnect_delay = min(
                self.config.video.reconnect_seconds,
                max(0.5, reconnect_delay * 2.0),
            )

        if active_decoder != requested_decoder:
            with self._state_lock:
                self._vision.video_decoder = active_decoder

    def _capture_loop_opencv(self) -> None:
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            with self._state_lock:
                self._vision.capture_error = f"OpenCV/Numpy 加载失败：{exc}"
            return

        read_timeout_ms, failure_grace_seconds = self._configure_capture_options()
        reconnect_delay = min(0.5, self.config.video.reconnect_seconds)
        generation = 0

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
                    self._show_placeholder(
                        cv2,
                        np,
                        "WAITING FOR RTSP VIDEO",
                        "Check camera power, link and RTSP address",
                    )
                sleep(reconnect_delay)
                reconnect_delay = min(
                    self.config.video.reconnect_seconds,
                    max(0.5, reconnect_delay * 2.0),
                )
                continue

            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            generation += 1
            with self._state_lock:
                self._vision.capture_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
            failure_started: float | None = None
            received_frame = False

            while not self._stop.is_set():
                ok, frame = capture.read()
                if ok and frame is not None:
                    failure_started = None
                    received_frame = True
                    reconnect_delay = 0.5
                    with self._raw_condition:
                        self._raw_frame = frame
                        self._raw_frame_serial += 1
                        self._raw_generation = generation
                        self._raw_condition.notify_all()
                    continue

                now = monotonic()
                if failure_started is None:
                    failure_started = now
                with self._state_lock:
                    self._vision.soft_read_failures += 1
                    self._vision.capture_error = (
                        "UDP数据短暂中断，保留最新画面并等待恢复"
                        if failure_grace_seconds > 0
                        else "RTSP读取超时或数据中断，正在重连"
                    )
                if now - failure_started < failure_grace_seconds:
                    sleep(0.05)
                    continue

                with self._state_lock:
                    self._vision.connected = False
                    self._vision.capture_error = "RTSP持续中断，正在重建连接"
                    self._vision.reconnect_count += 1
                break

            capture.release()
            if self._stop.is_set():
                break
            with self._state_lock:
                has_previous_frame = self._vision.last_frame_monotonic is not None
            if not has_previous_frame and not received_frame:
                self._show_placeholder(
                    cv2,
                    np,
                    "RTSP INTERRUPTED - RECONNECTING",
                    self.config.video.source,
                )
            sleep(reconnect_delay)
            reconnect_delay = min(
                self.config.video.reconnect_seconds,
                max(0.5, reconnect_delay * 2.0),
            )

    def _run(self) -> None:
        try:
            import cv2
        except Exception as exc:
            with self._state_lock:
                self._vision.detector_error = f"OpenCV加载失败：{exc}"
            return

        ultralytics_dir = Path.cwd() / ".runtime" / "ultralytics"
        ultralytics_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("YOLO_CONFIG_DIR", str(ultralytics_dir))
        detector = self.detector
        single_tracker = SingleObjectTracker(self.config.vision)
        with self._state_lock:
            self._vision.detector_ready = detector.ready
            self._vision.detector_error = detector.error

        started = monotonic()
        processed = 0
        rendered = 0
        last_raw_serial = 0
        last_generation = -1
        interpolator: NvidiaFrucInterpolator | None = None

        while not self._stop.is_set():
            with self._raw_condition:
                self._raw_condition.wait_for(
                    lambda: self._raw_frame_serial != last_raw_serial
                    or self._stop.is_set(),
                    timeout=1.0,
                )
                if self._raw_frame_serial == last_raw_serial:
                    continue
                frame = self._raw_frame
                raw_serial = self._raw_frame_serial
                generation = self._raw_generation

            if frame is None:
                continue
            skipped = max(0, raw_serial - last_raw_serial - 1)
            last_raw_serial = raw_serial
            if skipped:
                with self._state_lock:
                    self._vision.dropped_capture_frames += skipped

            if generation != last_generation:
                if interpolator is not None:
                    interpolator.close()
                interpolator = (
                    NvidiaFrucInterpolator(self.config.frame_interpolation)
                    if self.config.frame_interpolation.enabled
                    else None
                )
                single_tracker.clear()
                last_generation = generation

            if self.config.video.low_latency_latest_frame:
                # Publish the real frame first. FRUC interpolation requires the
                # next real frame and is therefore inherently older; it is still
                # evaluated for experiment metrics but never queued ahead of live video.
                interpolation_input = frame.copy() if interpolator is not None else None
                self._process_frame(cv2, frame, detector, single_tracker)
                processed += 1
                rendered += 1
                if interpolator is not None and interpolation_input is not None:
                    try:
                        synthetic = interpolator.push(interpolation_input, monotonic())
                        with self._state_lock:
                            self._vision.interpolation_ready = interpolator.ready
                            self._vision.interpolation_error = interpolator.error
                        if synthetic is not None:
                            self._record_synthetic(synthetic)
                    except Exception as exc:
                        with self._state_lock:
                            self._vision.interpolation_ready = False
                            self._vision.interpolation_error = str(exc)
                        interpolator.close()
                        interpolator = None
            else:
                if interpolator is not None:
                    try:
                        synthetic = interpolator.push(frame, monotonic())
                        with self._state_lock:
                            self._vision.interpolation_ready = interpolator.ready
                            self._vision.interpolation_error = interpolator.error
                        if synthetic is not None:
                            self._process_synthetic_preview(cv2, synthetic)
                            rendered += 1
                    except Exception as exc:
                        with self._state_lock:
                            self._vision.interpolation_ready = False
                            self._vision.interpolation_error = str(exc)
                        interpolator.close()
                        interpolator = None
                self._process_frame(cv2, frame, detector, single_tracker)
                processed += 1
                rendered += 1

            elapsed = monotonic() - started
            if elapsed >= 1.0:
                with self._state_lock:
                    self._vision.processing_fps = processed / elapsed
                    self._vision.output_fps = rendered / elapsed
                processed = 0
                rendered = 0
                started = monotonic()

        if interpolator is not None:
            interpolator.close()
        single_tracker.clear()

    def _record_synthetic(self, synthetic: InterpolatedFrame) -> None:
        with self._state_lock:
            self._vision.interpolation_ready = True
            self._vision.interpolation_ms = synthetic.elapsed_ms
            self._vision.synthetic_frames += 1
            if synthetic.repeated:
                self._vision.repeated_synthetic_frames += 1

    def _process_synthetic_preview(self, cv2: object, synthetic: InterpolatedFrame) -> None:
        frame = synthetic.frame
        with self._state_lock:
            tracks = [replace(track) for track in self._vision.tracks]
            locked = replace(self._locked_track) if self._locked_track is not None else None
            target = replace(self._vision.target)
            preview = replace(self._preview)
        self._draw_overlay(cv2, frame, tracks, locked, target, preview)
        cv2.putText(
            frame,
            "NVIDIA FRUC SYNTHETIC PREVIEW - YOLO/CONTROL USE REAL FRAMES ONLY",
            (18, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 190, 255),
            2,
            cv2.LINE_AA,
        )
        encode_ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.video.jpeg_quality)],
        )
        if encode_ok:
            self._publish_jpeg(encoded.tobytes())
        self._record_synthetic(synthetic)

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
