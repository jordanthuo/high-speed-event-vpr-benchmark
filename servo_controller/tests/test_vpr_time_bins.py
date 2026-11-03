"""Regression tests: Run identity, monotonic samples, anchors and nonoverlapping
bin boundaries. Half-open coverage and units are part of the downstream data
contract.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_vpr_time_bins import (
    build_motion_start_anchor,
    build_time_bins,
    load_camera_anchor,
    load_motion_rows,
    write_time_bins,
)


class VprTimeBinTests(unittest.TestCase):
    def test_encoder_samples_define_complete_nonoverlapping_camera_bins(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            metadata_path = directory / "camera_metadata.json"
            motion_path = directory / "recorded.csv"
            output_path = directory / "bins.csv"
            metadata_path.write_text(json.dumps({
                "run_id": "run-1",
                "session_start_lattepanda_monotonic_ns": 10_000_000_000,
                "lattepanda_t0_monotonic_ns": 14_000_000_000,
                "session_end_lattepanda_monotonic_ns": 20_000_000_000,
            }), encoding="utf-8")
            with motion_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow((
                    "run_id", "lattepanda_time_s", "motor_pc_monotonic_ns",
                    "motor_speed_command_rpm", "recorded_motor_feedback_rpm",
                    "encoder_position_pulses", "platform_angle_deg",
                    "platform_wrapped_angle_deg",
                ))
                writer.writerow(("run-1", "0.0", "100", "50", "49", "1000", "10", "10"))
                writer.writerow(("run-1", "0.02", "120", "50", "50", "1100", "13", "13"))
                writer.writerow(("run-1", "0.05", "150", "50", "50", "1200", "17.5", "17.5"))

            metadata, anchor = load_camera_anchor(metadata_path, camera_start_us=1_000_000)
            motion_rows = load_motion_rows(motion_path, metadata["run_id"])
            bins = build_time_bins(motion_rows, anchor, "10")
            write_time_bins(output_path, bins)

            self.assertEqual(anchor.camera_t0_us, 5_000_000)
            self.assertEqual(anchor.camera_end_us, 11_000_000)
            self.assertEqual(len(bins), 3)
            self.assertEqual(bins[0]["camera_start_us"], 1_000_000)
            self.assertEqual(bins[0]["camera_end_us"], 5_010_000)
            self.assertEqual(bins[1]["camera_start_us"], 5_010_000)
            self.assertEqual(bins[1]["camera_end_us"], 5_035_000)
            self.assertEqual(bins[2]["camera_start_us"], 5_035_000)
            self.assertEqual(bins[2]["camera_end_us"], 11_000_000)
            self.assertEqual(bins[1]["platform_angle_deg"], "13")
            self.assertEqual(bins[1]["camera_interval"], "[5010000,5035000)")

            with output_path.open("r", newline="", encoding="utf-8") as stream:
                written = list(csv.DictReader(stream))
            self.assertEqual(len(written), 3)
            self.assertEqual(written[0]["sync_method"], "recording_start_anchor")
            self.assertEqual(written[-1]["extends_to_recording_edge"], "true")

    def test_mismatched_run_id_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            motion_path = Path(temporary) / "recorded.csv"
            with motion_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow((
                    "run_id", "lattepanda_time_s", "motor_pc_monotonic_ns",
                    "motor_speed_command_rpm", "recorded_motor_feedback_rpm",
                    "encoder_position_pulses", "platform_angle_deg",
                    "platform_wrapped_angle_deg",
                ))
                writer.writerow(("wrong-run", "0", "100", "0", "0", "1", "0", "0"))
            with self.assertRaisesRegex(ValueError, "do not match camera run ID"):
                load_motion_rows(motion_path, "expected-run")

    def test_manual_motion_anchor_maps_detected_encoder_start_to_evk4_time(self) -> None:
        rows = [
            {
                "run_id": "run-1", "lattepanda_time_s": "0.04471535",
                "motor_pc_monotonic_ns": "1", "motor_speed_command_rpm": "50",
                "recorded_motor_feedback_rpm": "0.0", "encoder_position_pulses": "1",
                "platform_angle_deg": "10", "platform_wrapped_angle_deg": "10",
            },
            {
                "run_id": "run-1", "lattepanda_time_s": "0.0607043",
                "motor_pc_monotonic_ns": "2", "motor_speed_command_rpm": "50",
                "recorded_motor_feedback_rpm": "0.2", "encoder_position_pulses": "2",
                "platform_angle_deg": "10.1", "platform_wrapped_angle_deg": "10.1",
            },
            {
                "run_id": "run-1", "lattepanda_time_s": "0.0766344",
                "motor_pc_monotonic_ns": "3", "motor_speed_command_rpm": "50",
                "recorded_motor_feedback_rpm": "0.8", "encoder_position_pulses": "3",
                "platform_angle_deg": "10.2", "platform_wrapped_angle_deg": "10.2",
            },
            {
                "run_id": "run-1", "lattepanda_time_s": "0.09765295",
                "motor_pc_monotonic_ns": "4", "motor_speed_command_rpm": "50",
                "recorded_motor_feedback_rpm": "1.7", "encoder_position_pulses": "4",
                "platform_angle_deg": "10.3", "platform_wrapped_angle_deg": "10.3",
            },
            {
                "run_id": "run-1", "lattepanda_time_s": "0.11328615",
                "motor_pc_monotonic_ns": "5", "motor_speed_command_rpm": "50",
                "recorded_motor_feedback_rpm": "3.7", "encoder_position_pulses": "5",
                "platform_angle_deg": "10.4", "platform_wrapped_angle_deg": "10.4",
            },
        ]

        anchor = build_motion_start_anchor(rows, 4_500_000)
        bins = build_time_bins(rows, anchor)

        self.assertEqual(anchor.sync_method, "manual_motion_start_anchor")
        self.assertEqual(anchor.sync_anchor_lattepanda_time_s, "0.076634400")
        self.assertEqual(bins[2]["encoder_sample_camera_us"], 4_500_000)
        self.assertEqual(bins[0]["encoder_sample_camera_us"], 4_468_081)


if __name__ == "__main__":
    unittest.main()
