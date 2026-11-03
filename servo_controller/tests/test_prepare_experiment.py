"""Regression tests: Temporary experiment trees exercise discovery, organization
and preparation. Never substitute production acquisition folders for these
fixtures.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import prepare_experiment as preparation
from prepare_experiment import discover_runs, infer_nominal_speed, organize_runs


class PrepareExperimentTests(unittest.TestCase):
    def test_discovers_uuid_matched_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_id = "abc123"
            recorded = root / "50-rpm_test_recorded.csv"
            with recorded.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["run_id"])
                writer.writeheader()
                writer.writerow({"run_id": run_id})
            (root / f"{run_id}_camera.raw").write_bytes(b"raw")
            (root / f"{run_id}_camera_metadata.json").write_text(
                json.dumps({"run_id": run_id}), encoding="utf-8"
            )
            (root / f"{run_id}_camera_0x00_writes.csv").write_text("sequence\n", encoding="utf-8")
            runs = discover_runs(root)
            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0].run_id, run_id)

    def test_nominal_speed_comes_from_profile_prefix(self):
        self.assertEqual(infer_nominal_speed(Path("50-rpm_test_recorded.csv")), "50")
        self.assertEqual(infer_nominal_speed(Path("custom_recorded.csv")), "")

    def test_organizes_raw_and_complete_run_bundle_idempotently(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_id = "abc123"
            recorded = root / "50-rpm_test_recorded.csv"
            recorded.write_text(f"run_id\n{run_id}\n", encoding="utf-8")
            clock = root / "50-rpm_test_recorded_clock_sync.csv"
            clock.write_text("sequence\n", encoding="utf-8")
            raw = root / f"{run_id}_camera.raw"
            raw.write_bytes(b"raw")
            metadata = root / f"{run_id}_camera_metadata.json"
            metadata.write_text(json.dumps({"run_id": run_id}), encoding="utf-8")
            writes = root / f"{run_id}_camera_0x00_writes.csv"
            writes.write_text("sequence\n", encoding="utf-8")
            trigger = root / f"{run_id}_camera_trigger_events.csv"
            trigger.write_text("trigger_index\n", encoding="utf-8")
            discovered = discover_runs(root)
            setup = {run_id: {"run_label": "run1"}}

            first = organize_runs(root, discovered, setup)
            second = organize_runs(root, first, setup)

            self.assertEqual(first, second)
            self.assertTrue((root / "raw" / raw.name).is_file())
            for source in (recorded, clock, metadata, writes, trigger):
                self.assertTrue((root / "runs" / "run1" / source.name).is_file())

    def test_trigger_mode_builds_affine_bins_and_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_id = "trigger-run"
            recorded = root / "50-rpm_test_recorded.csv"
            fields = (
                "run_id", "lattepanda_time_s", "motor_pc_monotonic_ns",
                "motor_speed_command_rpm", "recorded_motor_feedback_rpm",
                "encoder_position_pulses", "platform_angle_deg",
                "platform_wrapped_angle_deg",
            )
            with recorded.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                for index, time_s in enumerate(("0.0", "0.02", "0.04")):
                    writer.writerow({
                        "run_id": run_id, "lattepanda_time_s": time_s,
                        "motor_pc_monotonic_ns": str(index),
                        "motor_speed_command_rpm": "50",
                        "recorded_motor_feedback_rpm": "50",
                        "encoder_position_pulses": str(index),
                        "platform_angle_deg": str(index),
                        "platform_wrapped_angle_deg": str(index),
                    })
            (root / f"{run_id}_camera.raw").write_bytes(b"raw")
            metadata = root / f"{run_id}_camera_metadata.json"
            t0_ns = 100_000_000_000
            metadata.write_text(json.dumps({
                "run_id": run_id,
                "lattepanda_t0_monotonic_ns": t0_ns,
            }), encoding="utf-8")
            writes = root / f"{run_id}_camera_0x00_writes.csv"
            with writes.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=(
                    "sequence", "write_requested_lattepanda_monotonic_ns",
                    "write_completed_lattepanda_monotonic_ns", "status",
                ))
                writer.writeheader()
                for sequence in range(6):
                    requested = t0_ns + sequence * 1_000_000_000
                    writer.writerow({
                        "sequence": sequence,
                        "write_requested_lattepanda_monotonic_ns": requested,
                        "write_completed_lattepanda_monotonic_ns": requested + 200_000,
                        "status": "completed",
                    })
            triggers = root / f"{run_id}_camera_trigger_events.csv"
            with triggers.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=(
                    "trigger_index", "raw_timestamp_us", "polarity", "channel_id",
                ))
                writer.writeheader()
                for sequence in range(6):
                    writer.writerow({
                        "trigger_index": sequence,
                        "raw_timestamp_us": round(1_500_000 + 4_000_000 + 1_000_010 * (sequence + 0.0001)),
                        "polarity": 0, "channel_id": 0,
                    })

            discovered = preparation.discover_runs(root)
            setup_path = preparation.write_setup(root, discovered)
            with setup_path.open("r", newline="", encoding="utf-8") as handle:
                setup_rows = list(csv.DictReader(handle))
            setup_rows[0]["sync_mode"] = "trigger"
            setup_rows[0]["studio_duration_s"] = "12.0"
            with setup_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=preparation.SETUP_FIELDS)
                writer.writeheader()
                writer.writerows(setup_rows)

            with patch.object(preparation, "raw_studio_origin_us", return_value=1_500_000):
                _setup, catalog_path, messages = preparation.prepare_experiment(root)

            with catalog_path.open("r", newline="", encoding="utf-8") as handle:
                catalog = list(csv.DictReader(handle))
            diagnostics = root / "outputs" / "trigger_sync" / "run1_trigger_sync.csv"
            diagnostics_exists = diagnostics.is_file()
            bins = root / "outputs" / "time_bins" / "run1_time_bins.csv"
            with bins.open("r", newline="", encoding="utf-8") as handle:
                first_bin = next(csv.DictReader(handle))

        self.assertEqual(catalog[0]["sync_method"], "trigger_affine")
        self.assertEqual(catalog[0]["trigger_match_count"], "6")
        self.assertTrue(diagnostics_exists)
        self.assertEqual(first_bin["sync_method"], "trigger_affine")
        self.assertIn("ready (trigger)", messages[0])


if __name__ == "__main__":
    unittest.main()
