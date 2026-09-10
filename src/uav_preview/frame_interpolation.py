from __future__ import annotations

from ctypes import (
    CDLL,
    POINTER,
    byref,
    c_char,
    c_double,
    c_int,
    c_size_t,
    c_uint8,
    c_void_p,
    c_wchar_p,
    create_string_buffer,
)
from dataclasses import dataclass
import os
from pathlib import Path
from threading import Lock
from typing import Any

from .config import FrameInterpolationConfig


@dataclass(slots=True)
class InterpolatedFrame:
    frame: Any
    timestamp: float
    repeated: bool
    elapsed_ms: float


class NvidiaFrucInterpolator:
    """Thin Python owner for the local NVIDIA FRUC native bridge.

    FRUC produces one temporal midpoint after two consecutive real frames.
    The returned frame is explicitly marked synthetic by this API and must not
    be used by the aircraft control path.
    """

    def __init__(self, config: FrameInterpolationConfig) -> None:
        self.config = config
        self.ready = False
        self.error = ""
        self._library: Any | None = None
        self._handle: int | None = None
        self._dll_directory: Any | None = None
        self._shape: tuple[int, int] | None = None
        self._previous_timestamp: float | None = None
        self._lock = Lock()

    @staticmethod
    def _error_text(buffer: Any) -> str:
        return buffer.value.decode("utf-8", errors="replace")

    def _load_library(self) -> None:
        if self._library is not None:
            return
        native_path = Path(self.config.native_library).expanduser().resolve()
        if not native_path.is_file():
            raise RuntimeError(
                f"本机FRUC桥接库不存在：{native_path}；请先运行 tools/build_nvof_fruc_bridge.ps1"
            )
        self._library = CDLL(str(native_path))
        self._library.nvof_fruc_create.argtypes = [
            c_wchar_p,
            c_int,
            c_int,
            c_int,
            POINTER(c_char),
            c_size_t,
        ]
        self._library.nvof_fruc_create.restype = c_void_p
        self._library.nvof_fruc_push_bgr.argtypes = [
            c_void_p,
            POINTER(c_uint8),
            c_size_t,
            c_double,
            POINTER(c_uint8),
            c_size_t,
            POINTER(c_int),
            POINTER(c_double),
            POINTER(c_char),
            c_size_t,
        ]
        self._library.nvof_fruc_push_bgr.restype = c_int
        self._library.nvof_fruc_destroy.argtypes = [c_void_p]
        self._library.nvof_fruc_destroy.restype = None

    def _create(self, width: int, height: int) -> None:
        self.close()
        fruc_dll = Path(self.config.sdk_root).expanduser().resolve() / (
            "NvOFFRUC/NvOFFRUCSample/bin/win64/NvOFFRUC.dll"
        )
        if not fruc_dll.is_file():
            raise RuntimeError(f"NVIDIA FRUC运行库不存在：{fruc_dll}")
        if hasattr(os, "add_dll_directory"):
            self._dll_directory = os.add_dll_directory(str(fruc_dll.parent))
        self._load_library()
        error = create_string_buffer(1024)
        handle = self._library.nvof_fruc_create(
            str(fruc_dll),
            width,
            height,
            self.config.device_id,
            error,
            len(error),
        )
        if not handle:
            raise RuntimeError(self._error_text(error) or "NVIDIA FRUC初始化失败")
        self._handle = int(handle)
        self._shape = (height, width)
        self._previous_timestamp = None
        self.ready = True
        self.error = ""

    def push(self, frame: Any, timestamp: float) -> InterpolatedFrame | None:
        import numpy as np

        if frame is None or getattr(frame, "ndim", 0) != 3 or frame.shape[2] != 3:
            raise ValueError("FRUC输入必须是H×W×3的BGR图像")
        source = np.ascontiguousarray(frame, dtype=np.uint8)
        height, width = source.shape[:2]
        with self._lock:
            try:
                if self._handle is None or self._shape != (height, width):
                    self._create(width, height)
                output = np.empty_like(source)
                repeated = c_int(0)
                elapsed_ms = c_double(0.0)
                error = create_string_buffer(1024)
                result = self._library.nvof_fruc_push_bgr(
                    c_void_p(self._handle),
                    source.ctypes.data_as(POINTER(c_uint8)),
                    source.strides[0],
                    float(timestamp),
                    output.ctypes.data_as(POINTER(c_uint8)),
                    output.strides[0],
                    byref(repeated),
                    byref(elapsed_ms),
                    error,
                    len(error),
                )
                if result < 0:
                    raise RuntimeError(self._error_text(error) or "NVIDIA FRUC处理失败")
                previous = self._previous_timestamp
                self._previous_timestamp = float(timestamp)
                if result == 0 or previous is None:
                    return None
                self.ready = True
                self.error = ""
                return InterpolatedFrame(
                    frame=output,
                    timestamp=(previous + float(timestamp)) * 0.5,
                    repeated=bool(repeated.value),
                    elapsed_ms=float(elapsed_ms.value),
                )
            except Exception as exc:
                self.ready = False
                self.error = str(exc)
                raise

    def close(self) -> None:
        if self._handle is not None and self._library is not None:
            self._library.nvof_fruc_destroy(c_void_p(self._handle))
        self._handle = None
        self._shape = None
        self._previous_timestamp = None
        self.ready = False
        if self._dll_directory is not None:
            self._dll_directory.close()
            self._dll_directory = None

    def __enter__(self) -> "NvidiaFrucInterpolator":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
