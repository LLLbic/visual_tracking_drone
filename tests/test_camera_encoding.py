from __future__ import annotations

import copy
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from camera_encoding import low_latency_substream


class CameraEncodingTests(unittest.TestCase):
    def setUp(self):
        self.original = [{
            "MainFormat": {"Video": {"Compression": "H.265", "FPS": 5, "Resolution": "720P"}},
            "ExtraFormat": {"AudioEnable": True, "VideoEnable": True, "Video": {
                "Compression": "H.265", "FPS": 5, "GOP": 2, "Resolution": "QVGA",
                "BitRate": 53, "BitRateControl": "VBR", "Quality": 1}},
        }]

    def test_substream_preserves_original_and_other_fields(self):
        before = copy.deepcopy(self.original)
        result = low_latency_substream(self.original)
        self.assertEqual(self.original, before)
        self.assertEqual(result[0]["MainFormat"], before[0]["MainFormat"])
        extra = result[0]["ExtraFormat"]
        self.assertTrue(extra["AudioEnable"])
        self.assertEqual(extra["Video"]["Resolution"], "QVGA")
        self.assertEqual(extra["Video"]["Compression"], "H.264")
        self.assertEqual(extra["Video"]["FPS"], 15)
        self.assertEqual(extra["Video"]["GOP"], 1)  # Device uses seconds, not frames.
        self.assertEqual(extra["Video"]["BitRate"], 512)

    def test_shared_codec_only_changes_main_codec(self):
        result = low_latency_substream(self.original, sync_main_codec=True)
        self.assertEqual(result[0]["MainFormat"]["Video"],
                         {"Compression": "H.264", "FPS": 5, "Resolution": "720P"})


if __name__ == "__main__":
    unittest.main()
