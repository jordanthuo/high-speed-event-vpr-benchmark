"""Regression tests: Selection of a wrapped angle within a specific revolution.
Missing coverage must not be borrowed from a neighboring turn.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path

import cv2

from build_angle_reference_csv import Sample
from make_angle_synchronised_video import AngleVideoInput, crossing_for_revolution, render_angle_synchronised_video


class AngleSynchronisedVideoTests(unittest.TestCase):
    def setUp(self):
        self.samples = [
            Sample(index * 0.1, -10.0 + index * 10.0, 1.0, 0.0)
            for index in range(75)
        ]

    def test_finds_same_wrapped_angle_in_distinct_revolutions(self):
        first = crossing_for_revolution(self.samples, 120.0, 0)
        second = crossing_for_revolution(self.samples, 120.0, 1)
        self.assertEqual(first, (1.3, 120.0))
        self.assertEqual(second, (4.9, 480.0))

    def test_missing_revolution_returns_none(self):
        self.assertIsNone(crossing_for_revolution(self.samples, 120.0, 3))

    def test_rejects_invalid_target(self):
        with self.assertRaises(ValueError):
            crossing_for_revolution(self.samples, 360.0, 0)

    def test_partial_angle_range_renders_only_available_common_arc(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "test.raw"
            words = []
            for time_us in (5_001_000, 5_101_000, 5_201_000):
                words.extend((0x8000 | (time_us >> 12), 0x6000 | (time_us & 0xFFF), 100, 0x2800 | 100))
            raw.write_bytes(b"% format EVT3\n% geometry 640x480\n% end\n"
                            + b"".join(word.to_bytes(2, "little") for word in words))
            bins = root / "bins.csv"
            with bins.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=["camera_start_us", "camera_end_us",
                    "platform_angle_deg", "platform_wrapped_angle_deg", "timing_uncertainty_ms"])
                writer.writeheader()
                for index in range(4):
                    angle = 3 + index * 100
                    writer.writerow(dict(camera_start_us=index * 50_000 - 25_000,
                        camera_end_us=index * 50_000 + 25_000, platform_angle_deg=angle,
                        platform_wrapped_angle_deg=angle, timing_uncertainty_ms=1))
            sources = [AngleVideoInput(label, 0, raw, bins, 0.2) for label in ("run1", "run2", "run3")]
            output = root / "partial.mp4"
            with contextlib.redirect_stdout(io.StringIO()):
                render_angle_synchronised_video(sources, output, angle_step_deg=100,
                    fps=10, start_angle_deg=72, end_angle_deg=297)
            with output.with_suffix(".csv").open(newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 9)
            self.assertEqual(sorted({float(row['target_wrapped_angle_deg']) for row in rows}), [72, 172, 272])
            self.assertTrue(all(row['matched'] == 'true' for row in rows))
            capture = cv2.VideoCapture(str(output))
            try:
                self.assertTrue(capture.isOpened())
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 3)
                self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 10)
            finally:
                capture.release()

    def test_rejects_invalid_range_before_opening_raw(self):
        sources = [AngleVideoInput(label, 0, Path("missing.raw"), Path("missing.csv"), 1)
                   for label in ("run1", "run2")]
        for start, end in ((297, 72), (-1, 300), (0, 361), (float('nan'), 360)):
            with self.subTest(start=start, end=end):
                with self.assertRaisesRegex(ValueError, "Angle range"):
                    render_angle_synchronised_video(sources, Path("unused.mp4"),
                        start_angle_deg=start, end_angle_deg=end)


if __name__ == "__main__":
    unittest.main()
