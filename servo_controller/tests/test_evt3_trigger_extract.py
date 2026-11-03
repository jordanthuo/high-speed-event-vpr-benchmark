"""Regression tests: Synthetic EVT3 word streams exercise timestamp reconstruction
and external edges. Keep chunk/wrap cases when optimizing the streaming
decoder.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import csv
import tempfile
import unittest
from pathlib import Path

from evt3_trigger_extract import extract_evt3_trigger_events


def word(event_type, payload):
    return ((event_type << 12) | payload).to_bytes(2, "little")


class Evt3TriggerExtractTests(unittest.TestCase):
    def test_extracts_edges_with_native_timestamps_and_rollover(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "test.raw"
            raw.write_bytes(
                b"% format EVT3;height=720;width=1280\n% end\n"
                + word(0x8, 0xFFF)
                + word(0x6, 4000)
                + word(0xA, (3 << 8) | 0)
                + word(0x8, 0)
                + word(0x6, 25)
                + word(0xA, (2 << 8) | 1)
            )
            output = root / "events.csv"
            count = extract_evt3_trigger_events(raw, output, chunk_bytes=7)
            with output.open("r", newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(count, 2)
        self.assertEqual(rows[0]["raw_timestamp_us"], str((0xFFF << 12) + 4000))
        self.assertEqual(rows[0]["polarity"], "0")
        self.assertEqual(rows[0]["channel_id"], "3")
        self.assertEqual(rows[1]["raw_timestamp_us"], str((1 << 24) + 25))
        self.assertEqual(rows[1]["polarity"], "1")
        self.assertEqual(rows[1]["channel_id"], "2")

    def test_rejects_non_evt3_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "test.raw"
            raw.write_bytes(b"% format EVT2\n% end\n")
            with self.assertRaisesRegex(ValueError, "expected EVT3"):
                extract_evt3_trigger_events(raw, root / "events.csv")


if __name__ == "__main__":
    unittest.main()
