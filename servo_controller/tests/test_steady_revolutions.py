import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from steady_revolutions import analyse, main, ranges


class SteadyRevolutionTests(unittest.TestCase):
    def rows(self):
        return [dict(platform_angle_deg=str(angle), motor_speed_command_rpm="100",
                     recorded_motor_feedback_rpm=str(50 if angle < 360 else 100))
                for angle in range(0, 1351, 10)]

    def test_excludes_acceleration_and_incomplete_last_turn(self):
        target, results = analyse(self.rows())
        self.assertEqual(target, 100)
        self.assertEqual([result["revolution"] for result in results], [0, 1, 2])
        # Turn 1's entry boundary still brackets acceleration; turn 2 is clean.
        self.assertEqual([result["revolution"] for result in results if result["steady"]], [2])

    def test_speed_dip_excludes_an_otherwise_steady_turn(self):
        rows = self.rows()
        rows[90]["recorded_motor_feedback_rpm"] = "80"
        _, results = analyse(rows)
        self.assertFalse(results[2]["steady"])
        self.assertEqual(results[2]["worst_percent"], 20)

    def test_constant_feedback_does_not_hide_changed_command(self):
        rows = self.rows()
        rows[90]["motor_speed_command_rpm"] = "0"
        _, results = analyse(rows)
        self.assertFalse(results[2]["steady"])

    def test_shared_ranges_stay_separate(self):
        common = {1, 2, 3, 5, 6} & {2, 3, 4, 5, 6}
        self.assertEqual(ranges(common), [[2, 3], [5, 6]])

    def test_reversal_is_rejected(self):
        rows = self.rows()
        rows[-1]["platform_angle_deg"] = "0"
        with self.assertRaisesRegex(ValueError, "Reversing"):
            analyse(rows)

    def test_catalog_paths_and_printed_comparison_command(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with (root / "bins.csv").open("w", newline="") as handle:
                rows = self.rows()
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            catalog = root / "vpr_runs.csv"
            with catalog.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=["run_label", "time_bins_csv"])
                writer.writeheader()
                writer.writerows(dict(run_label=f"run{i}", time_bins_csv="bins.csv") for i in (1, 2, 3))
            output = io.StringIO()
            with patch("sys.argv", ["steady_revolutions.py", "--catalog", str(catalog)]):
                with contextlib.redirect_stdout(output):
                    main()
            self.assertIn("Common steady ranges (inclusive, zero-based): 2-2", output.getvalue())
            self.assertIn("--instances run1:2 run2:2 run3:2", output.getvalue())
            self.assertFalse((root / "angle_verification").exists())


if __name__ == "__main__":
    unittest.main()
