"""Regression tests: Recorded-run CSV contracts and frozen clock mapping. Preserve
column meanings and synchronized/unsynchronized behavior when modifying
exporters.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from servo_control.clock_sync import ClockSyncSample, TestRunClock
from servo_control.motion_profile import load_motion_profile
from servo_control.telemetry import TelemetrySample
from servo_control.ui import ServoControllerApp


class ProfileExportTests(unittest.TestCase):
    @staticmethod
    def _sync_sample() -> ClockSyncSample:
        return ClockSyncSample(
            sequence=7,
            local_send_monotonic_ns=9_999_000_000,
            remote_receive_monotonic_ns=10_004_100_000,
            remote_send_monotonic_ns=10_004_200_000,
            local_receive_monotonic_ns=9_999_300_000,
            local_send_utc_ns=0,
            remote_receive_utc_ns=0,
            remote_send_utc_ns=0,
            local_receive_utc_ns=0,
        )

    def test_speed_export_includes_raw_encoder_position_and_platform_angle(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            profile_path = directory_path / "speed.csv"
            export_path = directory_path / "speed_recorded.csv"
            profile_path.write_text("time_s,motor_rpm\n0,0\n1,60\n", encoding="utf-8")
            profile = load_motion_profile(profile_path, ratio=2.0)

            app = SimpleNamespace(
                written_profile=profile,
                compiled_profile=None,
                profile_recorded_samples=[TelemetrySample(
                    elapsed_s=0.5,
                    sampled_monotonic_s=10.0,
                    acquisition_duration_s=0.002,
                    requested_rpm=60.0,
                    integrated_command_rpm=55.0,
                    encoder_feedback_rpm=54.0,
                    tracking_error_rpm=1.0,
                    platform_speed_rpm=27.0,
                    encoder_position_pulses=6_757_490,
                    motor_position_deg=145.0,
                    platform_position_deg=-9953.688372,
                    drawstring_voltage_v=3.0,
                    drawstring_extension_mm=1.0,
                )],
                profile_run_origin_motor_deg=100.0,
                session_origin_pulses=None,
                profile_preview=SimpleNamespace(
                    _speed_series=lambda _profile: ([0.0, 1.0], [0.0, 60.0], [0.0, 60.0])
                ),
                clock_sync=SimpleNamespace(samples=lambda: [], running=False),
                profile_run_clock=TestRunClock.create(
                    profile_path,
                    10_000_000_000,
                    self._sync_sample(),
                    run_id="test-run",
                ),
                profile_local_events=[{
                    "source": "motor_pc",
                    "event": "motor_release_requested",
                    "motor_pc_monotonic_ns": 10_000_000_000,
                    "lattepanda_monotonic_ns": 10_005_000_000,
                    "lattepanda_time_s": 0.0,
                    "details": "",
                }],
                profile_remote_events=[{
                    "source": "lattepanda",
                    "event": "serial_0x00_write_completed",
                    "lattepanda_monotonic_ns": 10_005_200_000,
                    "sequence": 0,
                    "scheduled_lattepanda_monotonic_ns": 10_005_000_000,
                    "details": "port=COM1; value=0x00",
                }],
                profile_clock_start_ns=None,
                profile_clock_end_ns=None,
                root=None,
                _log=lambda *args: None,
            )

            with patch("servo_control.ui.filedialog.asksaveasfilename", return_value=str(export_path)):
                ServoControllerApp._export_profile_run(app)

            with export_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            with export_path.with_name("speed_recorded_events.csv").open(
                newline="", encoding="utf-8"
            ) as stream:
                event_rows = list(csv.DictReader(stream))
            metadata = json.loads(
                export_path.with_name("speed_recorded_metadata.json").read_text(encoding="utf-8")
            )

        self.assertEqual(rows[0]["encoder_position_pulses"], "6757490")
        self.assertEqual(rows[0]["platform_angle_deg"], "-9953.688372")
        self.assertEqual(rows[0]["platform_wrapped_angle_deg"], "126.311628")
        self.assertEqual(rows[0]["run_id"], "test-run")
        self.assertEqual(rows[0]["lattepanda_time_s"], "0.000000000")
        self.assertEqual(rows[0]["motor_pc_monotonic_ns"], "10000000000")
        self.assertEqual(rows[0]["lattepanda_monotonic_ns"], "10005000000")
        self.assertEqual(rows[0]["clock_offset_ns"], "5000000.000")
        self.assertEqual(rows[0]["clock_sync_status"], "synchronized")
        self.assertEqual(rows[0]["motor_speed_command_rpm"], "60.000")
        self.assertNotIn("estimated_motor_speed_rpm", rows[0])
        self.assertNotIn("estimated_platform_speed_rpm", rows[0])
        self.assertEqual(
            [row["event"] for row in event_rows],
            ["motor_release_requested", "serial_0x00_write_completed"],
        )
        self.assertEqual(event_rows[1]["lattepanda_time_s"], "0.000200000")
        self.assertEqual(event_rows[1]["sequence"], "0")
        self.assertEqual(event_rows[1]["schedule_error_us"], "200.000")
        self.assertEqual(metadata["run_id"], "test-run")
        self.assertEqual(metadata["lattepanda_t0_monotonic_ns"], 10005000000)

    def test_run_clock_exports_negative_zero_and_positive_lattepanda_times(self) -> None:
        clock = TestRunClock.create(
            "profile.csv", 10_000_000_000, self._sync_sample(), run_id="timing"
        )
        self.assertEqual(clock.latte_panda_time_s(9_750_000_000), -0.25)
        self.assertEqual(clock.latte_panda_time_s(10_000_000_000), 0.0)
        self.assertEqual(clock.latte_panda_time_s(10_125_000_000), 0.125)

    def test_run_clock_marks_missing_sync_as_unsynchronized(self) -> None:
        clock = TestRunClock.create("profile.csv", 10_000_000_000, None, run_id="offline")
        self.assertFalse(clock.synchronized)
        self.assertIsNone(clock.lattepanda_t0_monotonic_ns)
        self.assertIsNone(clock.latte_panda_time_s(10_500_000_000))

    def test_scheduled_lattepanda_t0_is_converted_to_motor_pc_clock(self) -> None:
        clock = TestRunClock.from_scheduled_t0(
            "profile.csv", 25_000_000_000, self._sync_sample(), "scheduled"
        )
        self.assertEqual(clock.lattepanda_t0_monotonic_ns, 25_000_000_000)
        self.assertEqual(clock.motor_pc_t0_monotonic_ns, 24_995_000_000)
        self.assertEqual(clock.latte_panda_time_s(24_995_000_000), 0.0)


if __name__ == "__main__":
    unittest.main()
