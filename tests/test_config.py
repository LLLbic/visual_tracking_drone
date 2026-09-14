from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import load_config


class ConfigTests(unittest.TestCase):
    def test_local_takeoff_handoff_is_explicit_boolean_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "handoff.toml"
            path.write_text("", encoding="utf-8")
            self.assertFalse(load_config(path).local_offboard_takeoff.keyboard_handoff_enabled)
            path.write_text("[local_offboard_takeoff]\nkeyboard_handoff_enabled = true\n", encoding="utf-8")
            self.assertTrue(load_config(path).local_offboard_takeoff.keyboard_handoff_enabled)
            for value in ("1", "'false'", "'true'"):
                path.write_text(f"[local_offboard_takeoff]\nkeyboard_handoff_enabled = {value}\n", encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "must be a boolean"):
                    load_config(path)

    def test_position_stream_defaults_to_five_hz_without_parameter_writes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "position-stream.toml"
            path.write_text("", encoding="utf-8")
            config = load_config(path)
            self.assertEqual(config.position_stream.local_position_hz, 5.0)
            self.assertEqual(config.position_stream.global_position_hz, 5.0)
            self.assertEqual(config.position_stream.uplink_host, "127.0.0.1")

    def test_position_stream_rejects_non_loopback_uplink(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-position-stream.toml"
            path.write_text(
                "[position_stream]\nuplink_host = '192.168.1.201'\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "loopback-only"):
                load_config(path)

    def test_transmission_cannot_be_enabled_from_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe.toml"
            path.write_text("[control]\ntransmit_enabled = true\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "passive-only"):
                load_config(path)

    def test_ground_offboard_frequency_is_fixed_at_five_hz(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-rate.toml"
            path.write_text(
                "[ground_offboard_test]\navailable = true\nfrequency_hz = 10.0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "exactly 5.0 Hz"):
                load_config(path)

    def test_keyboard_ground_takeoff_cannot_be_reenabled_from_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-keyboard-takeoff.toml"
            path.write_text(
                "[keyboard_control]\nallow_ground_takeoff = true\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "must remain false"):
                load_config(path)

    def test_fruc_live_path_rejects_yolo_on_synthetic_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-fruc.toml"
            path.write_text(
                "[frame_interpolation]\nenabled = true\nyolo_on_synthetic = true\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "synthetic frames is forbidden"):
                load_config(path)

    def test_fruc_multiplier_is_currently_fixed_at_two(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad-fruc-rate.toml"
            path.write_text("[frame_interpolation]\nmultiplier = 4\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "multiplier=2"):
                load_config(path)

    def test_udp_low_latency_defaults_are_bounded(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "udp.toml"
            path.write_text("[video]\ntransport = 'udp'\n", encoding="utf-8")
            config = load_config(path)
            self.assertTrue(config.video.low_latency_latest_frame)
            self.assertEqual(config.video.udp_reorder_queue_size, 32)
            self.assertEqual(config.video.udp_max_delay_ms, 100)
            self.assertEqual(config.video.udp_read_timeout_ms, 1000)

    def test_udp_buffer_cannot_be_unbounded(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "udp-buffer.toml"
            path.write_text(
                "[video]\ntransport = 'udp'\nudp_buffer_size_bytes = 33554432\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "udp_buffer_size_bytes"):
                load_config(path)

    def test_ffmpeg_cuvid_capture_configuration(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ffmpeg.toml"
            path.write_text(
                "[video]\n"
                "capture_backend = 'ffmpeg'\n"
                "ffmpeg_decoder = 'hevc_cuvid'\n"
                "frame_width = 640\n"
                "frame_height = 360\n",
                encoding="utf-8",
            )
            config = load_config(path)
            self.assertEqual(config.video.capture_backend, "ffmpeg")
            self.assertEqual(config.video.ffmpeg_decoder, "hevc_cuvid")

    def test_unknown_capture_backend_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "backend.toml"
            path.write_text("[video]\ncapture_backend = 'unknown'\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "capture_backend"):
                load_config(path)

    def test_h264_cuvid_is_available_for_camera_codec_migration(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "h264.toml"
            path.write_text(
                "[video]\ncapture_backend = 'ffmpeg'\nffmpeg_decoder = 'h264_cuvid'\n",
                encoding="utf-8",
            )
            self.assertEqual(load_config(path).video.ffmpeg_decoder, "h264_cuvid")

    def test_takeoff_height_envelope_cannot_be_widened(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-takeoff.toml"
            path.write_text(
                "[takeoff]\nmin_height_m = 0.5\nmax_height_m = 5.0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "min_height_m"):
                load_config(path)

    def test_takeoff_default_is_within_one_to_three_metres(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "takeoff.toml"
            path.write_text(
                "[takeoff]\navailable = true\ndefault_height_m = 2.2\n",
                encoding="utf-8",
            )
            config = load_config(path)
            self.assertEqual(config.takeoff.min_height_m, 1.0)
            self.assertEqual(config.takeoff.max_height_m, 3.0)
            self.assertEqual(config.takeoff.default_height_m, 2.2)

    def test_local_takeoff_height_envelope_and_stream_rate_are_bounded(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe-local-takeoff.toml"
            path.write_text(
                "[local_offboard_takeoff]\nmax_height_m = 4.0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "max_height_m"):
                load_config(path)

            path.write_text(
                "[local_offboard_takeoff]\nfrequency_hz = 30.0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "frequency_hz"):
                load_config(path)

    def test_local_takeoff_defaults_to_current_pose_relative_height(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "local-takeoff.toml"
            path.write_text(
                "[local_offboard_takeoff]\navailable = true\ndefault_height_m = 2.0\n",
                encoding="utf-8",
            )
            config = load_config(path)
            self.assertTrue(config.local_offboard_takeoff.available)
            self.assertEqual(config.local_offboard_takeoff.default_height_m, 2.0)
            self.assertEqual(config.local_offboard_takeoff.frequency_hz, 10.0)


if __name__ == "__main__":
    unittest.main()
