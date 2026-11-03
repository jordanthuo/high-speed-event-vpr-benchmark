"""Regression tests: History conversion, tracking statistics, CSV exports and
sweep values. Keep blank optional data distinct from measured zeros.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import csv
import tempfile
import unittest
from pathlib import Path

from servo_control.models import DriveSnapshot
from servo_control.telemetry import EventHistory, TelemetryBuffer, speed_sweep_values


def snapshot(command: float, feedback: float) -> DriveSnapshot:
    return DriveSnapshot(
        p3_002=6,
        p3_006=0x0060,
        p4_007=0x00A1,
        p4_009=0x0003,
        drive_status=0x0003,
        alarm_code=0,
        requested_speed_rpm=command,
        integrated_command_rpm=command,
        feedback_speed_rpm=feedback,
        encoder_position_pulses=1 << 23,
        monitor_1_selection=9,
        monitor_2_selection=7,
        monitor_3_selection=3,
        simulated=True,
    )


class TelemetryTests(unittest.TestCase):
    def test_tracking_statistics_use_encoder_feedback(self) -> None:
        buffer = TelemetryBuffer(platform_ratio=2.0)
        buffer.append(snapshot(10.0, 8.0))
        buffer.append(snapshot(10.0, 6.0))
        self.assertAlmostEqual(buffer.rms_tracking_error() or 0.0, (10.0) ** 0.5)
        self.assertEqual(buffer.peak_tracking_error(), 4.0)
        self.assertEqual(buffer.samples[-1].platform_speed_rpm, 3.0)
        self.assertEqual(buffer.samples[-1].encoder_position_pulses, 1 << 23)
        self.assertEqual(buffer.samples[-1].motor_position_deg, 180.0)
        self.assertEqual(buffer.samples[-1].platform_position_deg, 90.0)

    def test_exports_telemetry_and_event_csv(self) -> None:
        buffer = TelemetryBuffer()
        buffer.append(snapshot(-20.0, -18.0))
        history = EventHistory()
        history.add("info", "test", "Sweep started", "simulation")
        with tempfile.TemporaryDirectory() as directory:
            telemetry_path = Path(directory) / "telemetry.csv"
            event_path = Path(directory) / "events.csv"
            buffer.export_csv(telemetry_path)
            history.export_csv(event_path)
            with telemetry_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
            with event_path.open(newline="", encoding="utf-8") as stream:
                event_rows = list(csv.reader(stream))
        self.assertEqual(rows[0][5], "encoder_feedback_rpm")
        self.assertEqual(rows[1][5], "-18.000")
        self.assertEqual(rows[0][1:3], ["sampled_monotonic_s", "acquisition_duration_ms"])
        self.assertEqual(
            rows[0][-6:],
            [
                "motor_position_deg", "motor_sin_theta", "platform_position_deg",
                "platform_sin_theta", "scene_displacement_ain0_v", "scene_vertical_displacement_mm",
            ],
        )
        self.assertAlmostEqual(float(rows[1][-5]), 0.0, places=6)
        self.assertAlmostEqual(float(rows[1][-3]), 1.0, places=6)
        self.assertEqual(event_rows[1][2:4], ["TEST", "Sweep started"])


class SequenceTests(unittest.TestCase):
    def test_speed_sweep_includes_end_and_returns(self) -> None:
        self.assertEqual(speed_sweep_values(-100, 100, 100), [-100, 0, 100, 0, -100])

    def test_invalid_sequence_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            speed_sweep_values(0, 100, 0)


if __name__ == "__main__":
    unittest.main()
