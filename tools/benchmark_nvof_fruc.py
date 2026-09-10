from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import statistics
import sys
from time import monotonic, perf_counter
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))
ULTRALYTICS_RUNTIME = REPO_ROOT / ".runtime" / "ultralytics"
ULTRALYTICS_RUNTIME.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(ULTRALYTICS_RUNTIME))

from uav_preview.config import AppConfig, VisionModelProfile, load_config
from uav_preview.frame_interpolation import NvidiaFrucInterpolator
from uav_preview.vision import UltralyticsTrackDetector


@dataclass(slots=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    box: tuple[float, float, float, float]


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def best_same_class_iou(reference: Detection, candidates: list[Detection]) -> float | None:
    values = [iou(reference.box, item.box) for item in candidates if item.class_id == reference.class_id]
    return max(values) if values else None


def open_capture(source: str, transport: str) -> Any:
    if transport == "tcp":
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;tcp|timeout;6000000|fflags;nobuffer|flags;low_delay|"
            "max_delay;0|reorder_queue_size;0"
        )
        timeout_ms = 6000
    else:
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|timeout;12000000"
        timeout_ms = 12000
    parsed: int | str = int(source) if source.isdigit() else source
    capture = cv2.VideoCapture(
        parsed,
        cv2.CAP_FFMPEG,
        [
            int(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC),
            5000,
            int(cv2.CAP_PROP_READ_TIMEOUT_MSEC),
            timeout_ms,
        ],
    )
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def capture_real_frames(config: AppConfig, source: str, count: int) -> tuple[list[np.ndarray], list[float]]:
    capture = open_capture(source, config.video.transport)
    if not capture.isOpened():
        raise RuntimeError(f"无法打开视频源：{source}")
    frames: list[np.ndarray] = []
    timestamps: list[float] = []
    try:
        failures = 0
        while len(frames) < count:
            ok, frame = capture.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 3:
                    raise RuntimeError("视频源连续读取失败三次")
                continue
            failures = 0
            frames.append(np.ascontiguousarray(frame))
            timestamps.append(monotonic())
    finally:
        capture.release()
    return frames, timestamps


def make_synthetic_frames(
    config: AppConfig,
    frames: list[np.ndarray],
    timestamps: list[float],
) -> tuple[list[np.ndarray], list[float], list[bool]]:
    generated: list[np.ndarray] = []
    latencies: list[float] = []
    repeated: list[bool] = []
    with NvidiaFrucInterpolator(config.frame_interpolation) as fruc:
        for frame, timestamp in zip(frames, timestamps, strict=True):
            result = fruc.push(frame, timestamp)
            if result is not None:
                generated.append(result.frame)
                latencies.append(result.elapsed_ms)
                repeated.append(result.repeated)
    return generated, latencies, repeated


def load_model(config: AppConfig, profile: VisionModelProfile) -> Any:
    if profile.kind == "yoloe":
        from ultralytics import YOLOE

        model = YOLOE(profile.model_path)
        model.set_classes(list(profile.prompts or []))
    else:
        from ultralytics import YOLO

        model = YOLO(profile.model_path)
    return model


def target_ids(model: Any, profile: VisionModelProfile) -> list[int] | None:
    explicit = list(profile.target_class_ids or [])
    if explicit:
        return explicit
    names = UltralyticsTrackDetector._model_names(model)
    wanted = {name.casefold() for name in (profile.target_class_names or [])}
    if not wanted:
        return None
    selected = [class_id for class_id, name in names.items() if name.casefold() in wanted]
    return selected or None


def detect_sequence(
    config: AppConfig,
    profile: VisionModelProfile,
    model: Any,
    frames: list[np.ndarray],
) -> tuple[list[list[Detection]], list[float]]:
    import torch

    outputs: list[list[Detection]] = []
    latencies: list[float] = []
    names = UltralyticsTrackDetector._model_names(model)
    classes = target_ids(model, profile)
    for frame in frames:
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        started = perf_counter()
        result = model.predict(
            frame,
            conf=config.vision.confidence,
            iou=config.vision.iou,
            classes=classes,
            device=None if config.vision.device.casefold() == "auto" else config.vision.device,
            verbose=False,
        )[0]
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        latencies.append((perf_counter() - started) * 1000.0)
        detections: list[Detection] = []
        boxes = result.boxes
        if boxes is not None and len(boxes):
            coords = boxes.xyxy.detach().cpu().numpy()
            confidences = boxes.conf.detach().cpu().numpy()
            class_ids = boxes.cls.detach().cpu().numpy().astype(int)
            for index, box in enumerate(coords):
                if profile.red_color_filter:
                    red_ratio = UltralyticsTrackDetector._red_ratio(frame, box)
                    if red_ratio < profile.min_red_ratio:
                        continue
                class_id = int(class_ids[index])
                detections.append(
                    Detection(
                        class_id=class_id,
                        class_name=names.get(class_id, str(class_id)),
                        confidence=float(confidences[index]),
                        box=tuple(float(value) for value in box),
                    )
                )
        outputs.append(detections)
    return outputs, latencies


def detection_summary(outputs: list[list[Detection]], latencies: list[float]) -> dict[str, Any]:
    confidences = [item.confidence for frame in outputs for item in frame]
    return {
        "frames": len(outputs),
        "frames_with_detection": sum(bool(frame) for frame in outputs),
        "detection_frame_rate": sum(bool(frame) for frame in outputs) / max(1, len(outputs)),
        "detections": sum(len(frame) for frame in outputs),
        "mean_detections_per_frame": sum(len(frame) for frame in outputs) / max(1, len(outputs)),
        "mean_confidence": statistics.fmean(confidences) if confidences else None,
        "inference_ms_mean": statistics.fmean(latencies) if latencies else None,
        "inference_ms_p95": percentile(latencies, 95),
    }


def compare_synthetic_to_neighbors(
    real: list[list[Detection]], synthetic: list[list[Detection]]
) -> dict[str, Any]:
    class_agreements: list[float] = []
    neighbor_ious: list[float] = []
    retained = 0
    opportunities = 0
    for index, generated in enumerate(synthetic):
        left = real[index]
        right = real[index + 1]
        expected_classes = {item.class_id for item in left + right}
        generated_classes = {item.class_id for item in generated}
        union = expected_classes | generated_classes
        if union:
            class_agreements.append(len(expected_classes & generated_classes) / len(union))
        for neighbor in left + right:
            opportunities += 1
            match = best_same_class_iou(neighbor, generated)
            if match is not None:
                retained += 1
                neighbor_ious.append(match)
    return {
        "class_set_jaccard_mean": statistics.fmean(class_agreements) if class_agreements else None,
        "neighbor_detection_retention": retained / opportunities if opportunities else None,
        "neighbor_box_iou_mean": statistics.fmean(neighbor_ious) if neighbor_ious else None,
        "note": "合成帧没有独立真值；一致性指标只与前后真实帧比较，不能视为精度mAP。",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="NVIDIA FRUC versus real-frame YOLO A/B benchmark")
    parser.add_argument("--config", default=str(REPO_ROOT / "config.toml"))
    parser.add_argument("--source", default=None, help="Override RTSP URL, video path, or camera index")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--profile", default=None, help="vision_models profile id")
    parser.add_argument("--confidence", type=float, default=None, help="Override YOLO confidence threshold")
    parser.add_argument(
        "--output",
        default=str(REPO_ROOT / ".runtime" / "nvof-fruc" / "ab-report.json"),
    )
    args = parser.parse_args()
    if args.frames < 3:
        parser.error("--frames must be at least 3")

    config = load_config(args.config)
    if args.confidence is not None:
        if not 0.0 < args.confidence < 1.0:
            parser.error("--confidence must be between 0 and 1")
        config.vision.confidence = args.confidence
    profile_id = args.profile or config.vision.active_model
    if profile_id not in config.vision_models:
        parser.error(f"unknown profile: {profile_id}")
    profile = config.vision_models[profile_id]
    source = str(args.source if args.source is not None else config.video.source)

    real_frames, timestamps = capture_real_frames(config, source, args.frames)
    synthetic_frames, fruc_latencies, repeated = make_synthetic_frames(
        config, real_frames, timestamps
    )
    model = load_model(config, profile)
    # One warm-up call keeps model initialization out of the measured sequence.
    detect_sequence(config, profile, model, [real_frames[0]])
    real_outputs, real_latencies = detect_sequence(config, profile, model, real_frames)
    synthetic_outputs, synthetic_latencies = detect_sequence(
        config, profile, model, synthetic_frames
    )

    intervals = [
        (timestamps[index] - timestamps[index - 1]) * 1000.0
        for index in range(1, len(timestamps))
    ]
    report = {
        "experiment": "NVIDIA Optical Flow SDK FRUC 2x / YOLO impact",
        "safety": "Synthetic frames are benchmark/preview-only and never enter control or MAVLink.",
        "source": source,
        "profile": profile_id,
        "model_path": profile.model_path,
        "device": config.vision.device,
        "confidence_threshold": config.vision.confidence,
        "resolution": [int(real_frames[0].shape[1]), int(real_frames[0].shape[0])],
        "real_frames": len(real_frames),
        "synthetic_frames": len(synthetic_frames),
        "capture_interval_ms_mean": statistics.fmean(intervals) if intervals else None,
        "estimated_input_fps": 1000.0 / statistics.fmean(intervals) if intervals else None,
        "estimated_preview_fps": (
            2000.0 / statistics.fmean(intervals) if intervals else None
        ),
        "fruc": {
            "process_ms_mean": statistics.fmean(fruc_latencies) if fruc_latencies else None,
            "process_ms_p95": percentile(fruc_latencies, 95),
            "repeated_frames": sum(repeated),
            "repeat_rate": sum(repeated) / max(1, len(repeated)),
        },
        "yolo_real": detection_summary(real_outputs, real_latencies),
        "yolo_synthetic": detection_summary(synthetic_outputs, synthetic_latencies),
        "synthetic_consistency": compare_synthetic_to_neighbors(real_outputs, synthetic_outputs),
        "latency_note": (
            "FRUC uses the next real frame plus the cached previous frame. It improves visual cadence "
            "but cannot remove RTSP/camera delay and adds one midpoint processing step."
        ),
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\nReport: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
