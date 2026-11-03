"""Paired-window orchestration checks; synthetic summaries are test fixtures only."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from run_vpr_window_comparison import duration_summary, validate_pair, main
from rig_vpr.sectors import sector_index, aggregate_sectors


def protocol(ms):
    return dict(acquisitions=[dict(key="example")], frozen_input_sha256={"example": "abc"},
        pair_mode="fixed-reference", reference="Low-Speed/run1", discovery_warnings=[],
        settings=dict(accumulation_ms=ms, angle_step_deg=1., tolerance_deg=5.,
            start_wrapped_deg=4., width=160, height=90, normalization="none",
            polarity="sum", pixels=0, seed=0, steady_tolerance_percent=5.))


class WindowComparisonTests(unittest.TestCase):
    def test_centred_sectors_wrap_and_preserve_all_query_scores(self):
        counts = [0] * 12
        for requested in range(4, 364):
            counts[sector_index(requested, 30, -15)] += 1
        self.assertEqual(counts, [30] * 12)
        angles = (-15, 0, 14.999, 15, 344.999, 345, 359.999, 360)
        self.assertEqual([sector_index(v, 30, -15) for v in angles], [0, 0, 0, 1, 11, 0, 0, 0])
        entries = [dict(key=k, condition=c) for k, c in (("ref", "Low-Speed"), ("qry", "High-Speed"))]
        queries, windows = {}, {}
        for i in range(360):
            queries["ref", "qry", i] = dict(query_wrapped_angle_deg=str(i),
                correct_at_1=i % 3 == 0, correct_at_5=i % 2 == 0, status="matched")
            windows["qry", i] = dict(wrapped_angle_deg=str(i), requested_wrapped_angle_deg=str(i))
        raw = dict(audit=dict(source_protocol=dict(acquisitions=entries)), rows=queries)
        pairs, refs, speeds, assigned = aggregate_sectors(raw, raw, windows, 30, -15)
        self.assertEqual(len(assigned), 360)
        self.assertEqual(len(pairs), 24)
        self.assertEqual(sorted({r["sector_start_deg"] for r in pairs}), list(range(-15, 345, 30)))
        for mode in ("none", "l1"):
            selected = [r for r in pairs if r["normalization"] == mode]
            self.assertTrue(all(r["query_count"] == 30 for r in selected))
            self.assertEqual(sum(r["correct_at_1"] for r in selected), 120)
            self.assertEqual(sum(r["correct_at_5"] for r in selected), 180)

    def test_45_degree_sectors_keep_all_360_sample_centres(self):
        counts = [0] * 8
        for requested in range(4, 364):
            counts[sector_index(requested, 45)] += 1
        self.assertEqual(counts, [45] * 8)
        self.assertEqual([sector_index(v, 45) for v in (0, 44.999, 45, 359.999, 360)],
                         [0, 0, 1, 7, 0])

    def test_pair_rejects_alignment_warning_and_different_acquisitions(self):
        with patch("rig_vpr.data.check_inputs", return_value=({}, [])) as check:
            with patch("rig_vpr.data.load_protocol", side_effect=[(protocol(1), Path(".")), (protocol(2), Path("."))]):
                validate_pair(Path("example"))
                self.assertEqual(check.call_count, 2)
            bad = protocol(1); bad["discovery_warnings"] = ["setup/prepared onset mismatch"]
            with patch("rig_vpr.data.load_protocol", return_value=(bad, Path("."))):
                with self.assertRaisesRegex(ValueError, "alignment warnings"):
                    validate_pair(Path("example"))
            bad = protocol(2); bad["acquisitions"] = [dict(key="different")]
            with patch("rig_vpr.data.load_protocol", side_effect=[(protocol(1), Path(".")), (bad, Path("."))]):
                with self.assertRaisesRegex(ValueError, "identical acquisitions"):
                    validate_pair(Path("example"))

    def test_percentage_point_comparison(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for ms, value in [(1, .7), (2, .5)]:
                for mode in ("raw", "l1"):
                    folder = root / f"{ms}ms/references_{mode}"
                    folder.mkdir(parents=True)
                    row = dict(reference_condition="Low-Speed", query_condition="High-Speed",
                        mean_recall_at_1=value, mean_recall_at_5=value)
                    with (folder / "summary_by_reference_speed.csv").open("w", newline="") as f:
                        writer = csv.DictWriter(f, fieldnames=list(row))
                        writer.writeheader(); writer.writerow(row)
            duration_summary(root)
            with (root / "window_comparison_by_speed.csv").open(newline="") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(float(rows[0]["recall_at_5_1ms_minus_2ms_percentage_points"]), 20.)

    def test_prepare_existing_study_fails_before_dispatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch("sys.argv", ["runner", "prepare", "--output-root", tmp]), patch("run_vpr_window_comparison.execute") as execute:
                with self.assertRaises(FileExistsError):
                    main()
                execute.assert_not_called()

    def test_changed_protocol_fails_before_dispatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / "1ms").mkdir()
            (root / "1ms/protocol.json").write_text("{}")
            (root / "study_plan.json").write_text(json.dumps(dict(protocol_sha256={"1": "wrong"})))
            with patch("sys.argv", ["runner", "run", "--output-root", tmp]), patch("run_vpr_window_comparison.execute") as execute:
                with self.assertRaisesRegex(ValueError, "protocol changed"):
                    main()
                execute.assert_not_called()
