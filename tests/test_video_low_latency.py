from __future__ import annotations

from pathlib import Path
import sys
from threading import Condition, Event
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.video import VideoTrackingEngine
from uav_preview.config import VideoConfig


class LowLatencyMjpegTests(unittest.TestCase):
    def test_ffmpeg_has_rtsp_timeout_and_single_thread_raw_output(self) -> None:
        engine = object.__new__(VideoTrackingEngine)
        engine.config = SimpleNamespace(video=VideoConfig(source="rtsp://test.invalid/camera"))
        with patch("uav_preview.video.shutil.which", return_value="ffmpeg"):
            command = engine._ffmpeg_capture_command("h264_cuvid")
        input_index = command.index("-i")
        self.assertIn("-timeout", command[:input_index])
        self.assertNotIn("-rw_timeout", command)
        self.assertEqual(command[command.index("-c:v") + 1], "h264_cuvid")
        output_options = command[input_index + 2:]
        self.assertEqual(output_options[output_options.index("-threads:v") + 1], "1")
        self.assertEqual(output_options[output_options.index("-fps_mode") + 1], "passthrough")
        self.assertNotIn("-r", command)

    def test_new_client_receives_only_current_jpeg_with_no_cache_headers(self) -> None:
        engine = object.__new__(VideoTrackingEngine)
        engine._stop = Event()
        engine._condition = Condition()
        engine._jpeg = b"newest-frame"
        engine._frame_serial = 42

        chunk = next(engine.mjpeg())

        self.assertIn(b"Content-Length: 12\r\n", chunk)
        self.assertIn(b"Cache-Control: no-store\r\n", chunk)
        self.assertTrue(chunk.endswith(b"newest-frame\r\n"))

    def test_latest_frame_long_poll_returns_current_frame_without_history(self) -> None:
        engine = object.__new__(VideoTrackingEngine)
        engine._stop = Event()
        engine._condition = Condition()
        engine._jpeg = b"current"
        engine._frame_serial = 17

        serial, jpeg = engine.wait_for_jpeg(after_serial=4, timeout=0.0)

        self.assertEqual(serial, 17)
        self.assertEqual(jpeg, b"current")

    def test_latest_frame_long_poll_returns_empty_when_serial_is_unchanged(self) -> None:
        engine = object.__new__(VideoTrackingEngine)
        engine._stop = Event()
        engine._condition = Condition()
        engine._jpeg = b"current"
        engine._frame_serial = 17

        serial, jpeg = engine.wait_for_jpeg(after_serial=17, timeout=0.0)

        self.assertEqual(serial, 17)
        self.assertIsNone(jpeg)


if __name__ == "__main__":
    unittest.main()
