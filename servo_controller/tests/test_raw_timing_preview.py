"""Exercise RAW decoding, quiet pre-roll timing, and the actual encoded MP4."""

import argparse
import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path

import cv2

from raw_timing_preview import render_preview, setup_run


class RawTimingPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.raw = self.root / "recording.raw"
        words = []
        for time_us in (5_001_000, 5_006_000, 5_101_000):
            words.extend((0x8000 | (time_us >> 12), 0x6000 | (time_us & 0xFFF),
                          0x0000 | 100, 0x2800 | 100))
        self.raw.write_bytes(
            b"% format EVT3;height=480;width=640\n% geometry 640x480\n% end\n"
            + b"".join(word.to_bytes(2, "little") for word in words)
        )

    def args(self, **changes):
        values = dict(raw=self.raw, output=self.root / "preview.mp4",
                      studio_duration_s=0.2, accumulation_ms=2.0,
                      source_fps=100.0, playback_speed=1.0, start_s=0.08, end_s=0.13)
        values.update(changes)
        return argparse.Namespace(**values)

    def render(self, args):
        with contextlib.redirect_stdout(io.StringIO()):
            return render_preview(args)

    def test_studio_time_retains_quiet_preroll_and_matches_video(self):
        original = self.raw.read_bytes()
        args = self.args()
        self.assertEqual(self.render(args), 5)
        with (self.root / "preview_frames.csv").open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        # First event is at Studio time 0.100 s, not recording time zero.
        self.assertEqual(rows[2]["raw_start_us"], "5001000")
        self.assertEqual(rows[2]["manual_evk4_motion_s"], "0.100000")
        self.assertEqual(rows[2]["window_end_s"], "0.102000")
        self.assertEqual(rows[2]["video_time_s"], "0.020000")
        capture = cv2.VideoCapture(str(args.output))
        try:
            self.assertTrue(capture.isOpened())
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 100)
            self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 5)
            self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), 568)
            self.assertTrue(capture.read()[0])
        finally:
            capture.release()
        self.assertEqual(self.raw.read_bytes(), original)
        with self.assertRaises(FileExistsError):
            self.render(args)

    def test_missing_duration_does_not_label_first_event_time_as_manual(self):
        self.render(self.args(studio_duration_s=None, start_s=0, end_s=0.02))
        with (self.root / "preview_frames.csv").open(newline="") as handle:
            row = next(csv.DictReader(handle))
        self.assertEqual(row["raw_start_us"], "5001000")
        self.assertEqual(row["time_reference"], "first_event")
        self.assertEqual(row["manual_evk4_motion_s"], "")

    def test_slow_playback_preserves_source_windows_and_manual_anchor(self):
        args = self.args(playback_speed=0.25)
        self.assertEqual(self.render(args), 5)
        with (self.root / "preview_frames.csv").open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[2]["raw_start_us"], "5001000")
        self.assertEqual(rows[2]["window_end_s"], "0.102000")
        self.assertEqual(rows[2]["manual_evk4_motion_s"], "0.100000")
        self.assertEqual(rows[2]["video_time_s"], "0.080000")
        capture = cv2.VideoCapture(str(args.output))
        try:
            self.assertTrue(capture.isOpened())
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 25)
            self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 5)
        finally:
            capture.release()

    def test_wrong_duration_is_rejected_before_creating_output(self):
        with self.assertRaisesRegex(ValueError, "shorter than the event span"):
            self.render(self.args(studio_duration_s=0.01))
        self.assertFalse((self.root / "preview.mp4").exists())

    def write_setup(self, rows):
        path = self.root / "experiment_setup.csv"
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=("run_label", "raw_file", "studio_duration_s"))
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_setup_selects_matching_run_and_keeps_recording_timeline(self):
        setup = self.write_setup([
            dict(run_label="run2", raw_file="missing.raw", studio_duration_s="99"),
            dict(run_label="run1", raw_file=self.raw.name, studio_duration_s="0.2"),
        ])
        original = setup.read_bytes()
        self.assertEqual(self.render(self.args(raw=None, setup=setup, run="run1",
                                              studio_duration_s=None)), 5)
        with (self.root / "preview_frames.csv").open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[2]["raw_start_us"], "5001000")
        self.assertEqual(rows[2]["manual_evk4_motion_s"], "0.100000")
        self.assertEqual(setup.read_bytes(), original)

    def test_setup_rejects_invalid_duration_instead_of_using_first_event_time(self):
        for duration in ("", "0", "-2", "nan", "inf", "00:01:30"):
            with self.subTest(duration=duration):
                setup = self.write_setup([dict(run_label="run1", raw_file=self.raw.name,
                                              studio_duration_s=duration)])
                with self.assertRaisesRegex(ValueError, "positive duration"):
                    setup_run(setup, "run1")

    def test_setup_rejects_ambiguous_or_missing_run(self):
        row = dict(run_label="run1", raw_file=self.raw.name, studio_duration_s="0.2")
        setup = self.write_setup([row, row])
        with self.assertRaisesRegex(ValueError, "found 2"):
            setup_run(setup, "run1")
        with self.assertRaisesRegex(ValueError, "found 0"):
            setup_run(setup, "run3")


if __name__ == "__main__":
    unittest.main()
