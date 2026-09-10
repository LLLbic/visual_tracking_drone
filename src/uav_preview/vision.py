from __future__ import annotations

from dataclasses import asdict
from math import hypot
from pathlib import Path
from threading import Lock
from typing import Any

from .config import VisionConfig, VisionModelProfile
from .types import DetectedTrack


class UltralyticsTrackDetector:
    def __init__(
        self,
        config: VisionConfig,
        profiles: dict[str, VisionModelProfile],
    ) -> None:
        self.config = config
        self.profiles = profiles
        self.model: Any = None
        self.ready = False
        self.error = ""
        self._lock = Lock()
        self._active_profile_id: str | None = None
        self._requested_profile_id: str | None = config.active_model
        self._loading_profile_id: str | None = None
        if not config.enabled:
            self.error = "视觉检测已在配置中关闭"
        else:
            self.error = "等待视频首帧后加载视觉模型"

    def _ensure_loaded(self) -> bool:
        if not self.config.enabled:
            return False
        with self._lock:
            requested = self._requested_profile_id
            if requested is None:
                return self.ready
            self._requested_profile_id = None
            self._loading_profile_id = requested
            profile = self.profiles[requested]
            had_model = self.model is not None
            self.error = f"正在加载：{profile.label}"
        try:
            if profile.kind == "yoloe":
                from ultralytics import YOLOE

                loaded_model = YOLOE(profile.model_path)
                loaded_model.set_classes(list(profile.prompts or []))
            else:
                from ultralytics import YOLO

                loaded_model = YOLO(profile.model_path)
            with self._lock:
                self.model = loaded_model
                self._active_profile_id = requested
                self.ready = True
                self.error = ""
        except Exception as exc:
            with self._lock:
                self.ready = had_model
                self.error = f"模型“{profile.label}”加载失败：{exc}"
        finally:
            with self._lock:
                self._loading_profile_id = None
        return self.ready

    def request_profile(self, profile_id: str) -> dict[str, Any]:
        if profile_id not in self.profiles:
            raise ValueError(f"未知视觉模型：{profile_id}")
        with self._lock:
            if profile_id != self._active_profile_id:
                self._requested_profile_id = profile_id
                self.error = f"已请求切换：{self.profiles[profile_id].label}"
        return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            active_id = self._active_profile_id
            requested_id = self._requested_profile_id
            loading_id = self._loading_profile_id
            ready = self.ready
            error = self.error
        active = self.profiles.get(active_id or "")
        return {
            "active_id": active_id,
            "active_label": active.label if active else "尚未加载",
            "active_model_path": active.model_path if active else "",
            "requested_id": requested_id,
            "loading_id": loading_id,
            "ready": ready,
            "error": error,
            "profiles": [asdict(profile) for profile in self.profiles.values()],
        }

    @staticmethod
    def _model_names(model: Any) -> dict[int, str]:
        names = getattr(model, "names", {})
        if isinstance(names, dict):
            return {int(class_id): str(name) for class_id, name in names.items()}
        return {index: str(name) for index, name in enumerate(names or [])}

    def _target_ids(self, model: Any, profile: VisionModelProfile) -> list[int] | None:
        explicit = list(profile.target_class_ids or [])
        if explicit:
            return explicit
        names = self._model_names(model)
        wanted = {name.casefold() for name in (profile.target_class_names or [])}
        if not wanted:
            return None
        ids = [int(class_id) for class_id, name in names.items() if str(name).casefold() in wanted]
        return ids or None

    @staticmethod
    def _red_ratio(frame: Any, coords: Any) -> float:
        import cv2
        import numpy as np

        height, width = frame.shape[:2]
        x1 = max(0, min(width, int(coords[0])))
        y1 = max(0, min(height, int(coords[1])))
        x2 = max(0, min(width, int(coords[2])))
        y2 = max(0, min(height, int(coords[3])))
        if x2 <= x1 or y2 <= y1:
            return 0.0
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        low_red = cv2.inRange(
            hsv,
            np.array((0, 80, 45), dtype=np.uint8),
            np.array((12, 255, 255), dtype=np.uint8),
        )
        high_red = cv2.inRange(
            hsv,
            np.array((168, 80, 45), dtype=np.uint8),
            np.array((179, 255, 255), dtype=np.uint8),
        )
        return float(cv2.countNonZero(cv2.bitwise_or(low_red, high_red))) / float(hsv.shape[0] * hsv.shape[1])

    def detect_and_track(self, frame: Any) -> list[DetectedTrack]:
        if not self._ensure_loaded() or self.model is None:
            return []
        with self._lock:
            model = self.model
            active_profile_id = self._active_profile_id
        if active_profile_id is None:
            return []
        profile = self.profiles[active_profile_id]
        try:
            results = model.track(
                frame,
                persist=True,
                tracker=self.config.multi_object_tracker,
                conf=self.config.confidence,
                iou=self.config.iou,
                classes=self._target_ids(model, profile),
                device=None if self.config.device.casefold() == "auto" else self.config.device,
                verbose=False,
            )
            boxes = results[0].boxes if results else None
            if boxes is None or len(boxes) == 0:
                return []
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confidences = boxes.conf.detach().cpu().numpy()
            class_ids = boxes.cls.detach().cpu().numpy().astype(int)
            if boxes.id is None:
                track_ids = list(range(-1, -len(xyxy) - 1, -1))
            else:
                track_ids = boxes.id.detach().cpu().numpy().astype(int).tolist()
            names = self._model_names(model)
            tracks = []
            for index, coords in enumerate(xyxy):
                class_id = int(class_ids[index])
                red_ratio = self._red_ratio(frame, coords) if profile.red_color_filter else None
                if red_ratio is not None and red_ratio < profile.min_red_ratio:
                    continue
                tracks.append(
                    DetectedTrack(
                        track_id=int(track_ids[index]),
                        class_id=class_id,
                        class_name=str(names.get(class_id, class_id)),
                        confidence=float(confidences[index]),
                        x1=float(coords[0]),
                        y1=float(coords[1]),
                        x2=float(coords[2]),
                        y2=float(coords[3]),
                        red_ratio=red_ratio,
                    )
                )
            with self._lock:
                self.error = ""
            return tracks
        except Exception as exc:
            with self._lock:
                self.error = f"YOLO/ByteTrack 推理失败：{exc}"
            return []


class SingleObjectTracker:
    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.tracker: Any = None
        self.error = ""

    def _create(self, cv2: Any) -> Any:
        backend = self.config.single_object_tracker.casefold()
        if backend == "csrt":
            if hasattr(cv2, "TrackerCSRT_create"):
                return cv2.TrackerCSRT_create()
            if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
                return cv2.legacy.TrackerCSRT_create()
            raise RuntimeError("当前 OpenCV 不包含 CSRT，请安装 opencv-contrib-python")
        if backend == "dasiamrpn":
            required = {
                "model": self.config.dasiamrpn_model,
                "kernel_cls1": self.config.dasiamrpn_kernel_cls1,
                "kernel_r1": self.config.dasiamrpn_kernel_r1,
            }
            missing = [name for name, path in required.items() if not path or not Path(path).is_file()]
            if missing:
                raise RuntimeError(f"DaSiamRPN 缺少模型文件：{', '.join(missing)}")
            params_type = getattr(cv2, "TrackerDaSiamRPN_Params", None)
            creator = getattr(cv2, "TrackerDaSiamRPN_create", None)
            if params_type is None or creator is None:
                raise RuntimeError("当前 OpenCV 不包含 DaSiamRPN")
            params = params_type()
            params.model = required["model"]
            params.kernel_cls1 = required["kernel_cls1"]
            params.kernel_r1 = required["kernel_r1"]
            return creator(params)
        raise RuntimeError(f"未知单目标跟踪器：{self.config.single_object_tracker}")

    def initialize(self, cv2: Any, frame: Any, track: DetectedTrack) -> bool:
        try:
            self.tracker = self._create(cv2)
            bbox = (
                int(round(track.x1)),
                int(round(track.y1)),
                max(1, int(round(track.width))),
                max(1, int(round(track.height))),
            )
            result = self.tracker.init(frame, bbox)
            self.error = ""
            return result is not False
        except Exception as exc:
            self.tracker = None
            self.error = f"单目标跟踪初始化失败：{exc}"
            return False

    def update(self, frame: Any, track_id: int, class_name: str, confidence: float) -> DetectedTrack | None:
        if self.tracker is None:
            return None
        try:
            ok, bbox = self.tracker.update(frame)
            if not ok:
                return None
            x, y, width, height = (float(value) for value in bbox)
            return DetectedTrack(
                track_id=track_id,
                class_id=-1,
                class_name=class_name,
                confidence=confidence,
                x1=x,
                y1=y,
                x2=x + width,
                y2=y + height,
            )
        except Exception as exc:
            self.error = f"单目标跟踪失败：{exc}"
            self.tracker = None
            return None

    def clear(self) -> None:
        self.tracker = None
        self.error = ""


def choose_reacquisition_candidate(
    previous: DetectedTrack,
    candidates: list[DetectedTrack],
    frame_width: int,
    frame_height: int,
    max_center_distance: float,
    max_height_change: float,
) -> DetectedTrack | None:
    if not candidates or frame_width <= 0 or frame_height <= 0 or previous.height <= 0:
        return None
    previous_cx, previous_cy = previous.center
    diagonal = hypot(frame_width, frame_height)
    best: tuple[float, DetectedTrack] | None = None
    for candidate in candidates:
        cx, cy = candidate.center
        center_distance = hypot(cx - previous_cx, cy - previous_cy) / diagonal
        height_change = abs(candidate.height - previous.height) / previous.height
        if center_distance > max_center_distance or height_change > max_height_change:
            continue
        score = center_distance + 0.35 * height_change - 0.05 * candidate.confidence
        if best is None or score < best[0]:
            best = (score, candidate)
    return None if best is None else best[1]
