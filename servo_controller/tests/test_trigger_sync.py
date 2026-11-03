"""Regression tests: Completed-write/edge filtering and affine matching, including
missing edges. Small synthetic residuals do not establish real physical timing
accuracy.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from trigger_sync import (
    TriggerEvent,
    UartWrite,
    build_trigger_anchor,
    fit_trigger_sync,
    load_trigger_events,
    load_uart_writes,
)


class TriggerSyncTests(unittest.TestCase):
    def synthetic_data(self):
        t0_ns = 100_000_000_000
        origin_us = 1_500_000
        intercept_us = 4_000_000.0
        scale = 1_000_020.0
        writes = []
        events = []
        for sequence in range(8):
            requested = t0_ns + sequence * 1_000_000_000 + 5_000_000
            completed = requested + 200_000
            write = UartWrite(
                sequence, requested, completed,
                ((requested + completed) / 2.0 - t0_ns) / 1_000_000_000.0,
            )
            writes.append(write)
            if sequence == 3:
                continue
            camera_us = intercept_us + scale * write.lattepanda_time_s
            events.append(TriggerEvent(sequence, round(origin_us + camera_us), 0, 0))
        return writes, events, origin_us, intercept_us, scale

    def test_affine_fit_handles_a_missing_edge(self):
        writes, events, origin_us, intercept_us, scale = self.synthetic_data()
        fit = fit_trigger_sync(writes, events, origin_us)
        anchor = build_trigger_anchor(fit, 12.0)

        self.assertEqual(len(fit.matches), 7)
        self.assertAlmostEqual(fit.intercept_camera_us_at_t0, intercept_us, delta=1.0)
        self.assertAlmostEqual(fit.scale_camera_us_per_lattepanda_s, scale, delta=1.0)
        self.assertAlmostEqual(fit.drift_ppm, 20.0, delta=1.0)
        self.assertEqual(anchor.sync_method, "trigger_affine")
        self.assertEqual(anchor.camera_start_us, 0)
        self.assertEqual(anchor.camera_end_us, 12_000_000)

    def test_loaders_filter_completed_writes_and_falling_main_edges(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metadata = root / "metadata.json"
            metadata.write_text(json.dumps({
                "lattepanda_t0_monotonic_ns": 10_000_000_000,
            }), encoding="utf-8")
            writes_path = root / "writes.csv"
            with writes_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=(
                    "sequence", "write_requested_lattepanda_monotonic_ns",
                    "write_completed_lattepanda_monotonic_ns", "status",
                ))
                writer.writeheader()
                for sequence in range(4):
                    writer.writerow({
                        "sequence": sequence,
                        "write_requested_lattepanda_monotonic_ns": 10_000_000_000 + sequence * 1_000_000_000,
                        "write_completed_lattepanda_monotonic_ns": 10_000_100_000 + sequence * 1_000_000_000,
                        "status": "failed" if sequence == 3 else "completed",
                    })
            triggers_path = root / "triggers.csv"
            with triggers_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=(
                    "trigger_index", "raw_timestamp_us", "polarity", "channel_id",
                ))
                writer.writeheader()
                writer.writerows([
                    {"trigger_index": 0, "raw_timestamp_us": 1, "polarity": 0, "channel_id": 0},
                    {"trigger_index": 1, "raw_timestamp_us": 2, "polarity": 1, "channel_id": 0},
                    {"trigger_index": 2, "raw_timestamp_us": 3, "polarity": 0, "channel_id": 1},
                    {"trigger_index": 3, "raw_timestamp_us": 4, "polarity": 0, "channel_id": 0},
                    {"trigger_index": 4, "raw_timestamp_us": 5, "polarity": 0, "channel_id": 0},
                ])

            writes = load_uart_writes(writes_path, metadata)
            triggers = load_trigger_events(triggers_path)

        self.assertEqual(len(writes), 3)
        self.assertEqual([event.raw_timestamp_us for event in triggers], [1, 4, 5])


if __name__ == "__main__":
    unittest.main()
