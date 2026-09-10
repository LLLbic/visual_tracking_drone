from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_preview.config import VisionConfig, VisionModelProfile, load_config
from uav_preview.vision import UltralyticsTrackDetector


class VisionModelTests(unittest.TestCase):
    def test_config_loads_red_barrel_yoloe_profile(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "models.toml"
            path.write_text(
                "[vision]\nactive_model = 'red_barrel'\n"
                "[vision_models.red_barrel]\n"
                "label = 'Red barrel'\nmodel_path = 'yoloe-26n-seg.pt'\n"
                "kind = 'yoloe'\nprompts = ['red metal barrel']\n"
                "red_color_filter = true\nmin_red_ratio = 0.08\n",
                encoding="utf-8",
            )
            config = load_config(path)
            profile = config.vision_models["red_barrel"]
            self.assertEqual(profile.kind, "yoloe")
            self.assertTrue(profile.red_color_filter)
            self.assertEqual(profile.prompts, ["red metal barrel"])

    def test_config_rejects_unknown_active_profile(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.toml"
            path.write_text("[vision]\nactive_model = 'missing'\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "is not defined"):
                load_config(path)

    def test_red_filter_accepts_red_and_rejects_blue(self) -> None:
        red = np.zeros((40, 40, 3), dtype=np.uint8)
        red[:] = (0, 0, 255)
        blue = np.zeros((40, 40, 3), dtype=np.uint8)
        blue[:] = (255, 0, 0)
        box = np.array([0, 0, 40, 40], dtype=np.float32)
        self.assertGreater(UltralyticsTrackDetector._red_ratio(red, box), 0.99)
        self.assertLess(UltralyticsTrackDetector._red_ratio(blue, box), 0.01)

    def test_frontend_selection_is_limited_to_registered_profiles(self) -> None:
        profile = VisionModelProfile(
            id="barrel",
            label="Barrel",
            model_path="barrel.pt",
        )
        detector = UltralyticsTrackDetector(
            VisionConfig(enabled=False, active_model="barrel"),
            {"barrel": profile},
        )
        with self.assertRaisesRegex(ValueError, "未知视觉模型"):
            detector.request_profile("arbitrary-path")


if __name__ == "__main__":
    unittest.main()
