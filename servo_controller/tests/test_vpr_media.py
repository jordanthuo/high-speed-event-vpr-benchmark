"""Regression tests: CLI helper contracts for catalog paths, labels and
run/revolution selection. Relative catalog paths must resolve from the catalog
location, not the shell directory.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import unittest
from unittest.mock import patch
from tempfile import TemporaryDirectory

from pathlib import Path

from vpr_media import angle_slug, catalogue_path_value, parse_instances, selection_suffix, build_parser, command_run_video, RunConfig
from make_angle_video import AngleBin


class VprMediaTests(unittest.TestCase):
    def test_run_video_time_selection_and_invalid_bounds(self):
        root = Path('example')
        run = RunConfig('run1', root/'recorded.csv', root/'bins.csv', root/'camera.raw', 20.0)
        parser = build_parser()
        base = ['run-video', '--run', 'run1']
        with patch('vpr_media.render_video') as render:
            for options, expected in [
                (['--start-s', '5', '--end-s', '10'], (5.0, 10.0)),
                (['--start-s', '5'], (5.0, 20.0)),
                (['--end-s', '10'], (0.0, 10.0)),
            ]:
                command_run_video(parser.parse_args(base + options), {'run1': run}, root)
                self.assertEqual(render.call_args.args[-2:], expected)
                self.assertIn('_time', render.call_args.args[2].name)
            for options in [
                ['--start-s', '-1'],
                ['--start-s', '10', '--end-s', '5'],
                ['--end-s', '0'],
                ['--end-s', '21'],
                ['--start-s', 'nan'],
                ['--end-s', 'inf'],
                ['--start-s', '5', '--start-rev', '2'],
                ['--end-s', '10', '--end-rev', '2'],
            ]:
                render.reset_mock()
                with self.assertRaises(ValueError):
                    command_run_video(parser.parse_args(base + options), {'run1': run}, root)
                render.assert_not_called()

    def test_run_video_passes_selected_bounds_and_preserves_full_video_name(self):
        with TemporaryDirectory() as folder:
            root = Path(folder)
            run = RunConfig('run1', root/'recorded.csv', root/'bins.csv', root/'camera.raw', 20.0)
            bins = [AngleBin(i, i + 1, i * 360, 0) for i in range(10)]
            with patch('vpr_media.load_bins', return_value=bins), patch('vpr_media.render_video') as render:
                args = build_parser().parse_args(['run-video', '--run', 'run1', '--start-rev', '3', '--end-rev', '6'])
                command_run_video(args, {'run1': run}, root)
                self.assertEqual(render.call_args.args[-2:], (3.0, 7.0))
                self.assertEqual(render.call_args.args[2].name, 'run1_angle_overlay_rev3-6.mp4')
                args = build_parser().parse_args(['run-video', '--run', 'run1'])
                command_run_video(args, {'run1': run}, root)
                self.assertEqual(render.call_args.args[-2:], (None, None))
                self.assertEqual(render.call_args.args[2].name, 'run1_angle_overlay.mp4')
                args = build_parser().parse_args(['run-video', '--run', 'run1', '--end-rev', '8'])
                short = RunConfig('run1', run.recorded_csv, run.time_bins_csv, run.raw_file, 5.0)
                with self.assertRaisesRegex(ValueError, 'RAW recording duration'):
                    command_run_video(args, {'run1': short}, root)

    def test_angle_slug_is_path_safe(self):
        self.assertEqual(angle_slug(120.5), "120p5deg")
        self.assertEqual(angle_slug(-1.25), "m1p25deg")

    def test_instance_selection_parser(self):
        self.assertEqual(parse_instances(["run1:0", "run2:3"]), {("run1", 0), ("run2", 3)})
        with self.assertRaises(ValueError):
            parse_instances(["run1"])

    def test_selection_suffix_preserves_selected_instances(self):
        self.assertEqual(selection_suffix(None, {("run1", 3), ("run1", 0)}), "run1-rev0_run1-rev3")
        many = {(f"run{run}", revolution) for run in range(1, 5) for revolution in (1, 5, 8)}
        shortened = selection_suffix(None, many)
        self.assertRegex(shortened, r"^selected-12-[0-9a-f]{10}$")
        self.assertLessEqual(len(shortened), 32)
        self.assertEqual(shortened, selection_suffix(None, many))
        self.assertEqual(selection_suffix({"run2", "run1"}, None), "runs-run1-run2")
        self.assertEqual(selection_suffix(None, None), "all")

    def test_catalogue_paths_are_relative_to_catalogue(self):
        catalog = Path.cwd() / "vpr_runs.csv"
        source = Path.cwd() / "Speed Profiles" / "run.csv"
        self.assertEqual(catalogue_path_value(catalog, source), "Speed Profiles/run.csv")


if __name__ == "__main__":
    unittest.main()
