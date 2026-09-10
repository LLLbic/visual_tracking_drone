from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import load_config


class ConfigTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
