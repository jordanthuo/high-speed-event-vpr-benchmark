import math
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np

from evaluate_rig import CONDITIONS, angle_level, angular_edges, circular_delta, command_events, crossing, event_interval, event_statistics, import_event_results, ramp_reference, report, sustained, weighted_stats, write_csv


class RigEvaluationTests(unittest.TestCase):
    def test_report_accepts_direct_raw_results_without_import_only_alignment_field(self):
        summary_keys = ('nominal_platform_rpm', 'mean_platform_rpm', 'between_run_mean_speed_sd_rpm',
            'mean_within_run_sd_rpm', 'mean_rise_10_90_s', 'mean_fall_90_10_s',
            'mean_zero_command_to_stopped_s', 'start_orientation_range_deg', 'start_orientation_sd_deg',
            'mean_total_travel_deg', 'total_travel_sd_deg')
        result_keys = ('estimated_full_ramp_s', 'estimated_10_90_ramp_s', 'rise_10_90_s',
            'fall_90_10_s', 'ramp_model_mae_platform_rpm', 'ramp_model_rmse_platform_rpm')
        summaries = [dict(condition=c, **dict.fromkeys(summary_keys, 1.0)) for c in CONDITIONS]
        results = [dict(condition=c, **dict.fromkeys(result_keys, 1.0)) for c in CONDITIONS]
        qualities = [dict(applied_biases='{}', median_sample_interval_ms=16,
                          maximum_steady_sample_interval_ms=17, maximum_sample_interval_ms=100)]
        events = [dict(condition=c, run=f'run{i}', status='processed', cd_events=3600,
            events_per_second=1000, events_per_degree=10, mean_events_per_2ms=2,
            mean_active_pixel_fraction_2ms=.1, on_fraction=.5, empty_2ms_fraction=0,
            manual_evk4_motion_s_used=1.1) for c in CONDITIONS for i in (1,2,3)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report(root, summaries, qualities, events, results, 4, 364)
            text = (root/'rig_evaluation_report.md').read_text(encoding='utf-8')
            self.assertIn('common 360 degree rotation', text)
            self.assertNotIn('### Imported alignment values', text)
            events[0]['local_manual_evk4_motion_s'] = 1.2
            report(root, summaries, qualities, events, results, 4, 364)
            self.assertIn('### Imported alignment values', (root/'rig_evaluation_report.md').read_text(encoding='utf-8'))

    def test_import_reconciles_counts_and_retains_linux_alignment_without_changing_setup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trace = dict(condition='Low-Speed', run='run1', selected_rev=1,
                         setup=dict(raw_file='raw/test.raw', manual_evk4_motion_s='1.2'))
            row = dict(condition='Low-Speed', run='run1', raw_file='/linux/test.raw', status='processed',
                       revolution=1, sector_start_deg=72, sector_end_deg=297, duration_s=1,
                       cd_events=2250, events_per_second=2250, events_per_degree=10,
                       complete_2ms_windows=500, mean_events_per_2ms=4.5,
                       mean_active_pixel_fraction_2ms=.1, on_fraction=.5, empty_2ms_fraction=0)
            write_csv(root/'event_statistics.csv', [row])
            write_csv(root/'event_sector_counts.csv', [dict(condition='Low-Speed', run='run1',
                      angle_start_deg=72+i*5, angle_end_deg=77+i*5, cd_events=50, events_per_degree=10)
                      for i in range(45)])
            write_csv(root/'data_quality.csv', [dict(condition='Low-Speed', run='run1',
                      manual_evk4_motion_s=1.1, studio_duration_s=10)])
            # Legacy partial-sector files must not enter the new full-turn analysis.
            with self.assertRaisesRegex(ValueError, 'angular selection differs'):
                import_event_results(root, [trace])
            results, sectors = import_event_results(root, [trace], 72, 297)
            self.assertEqual(results[0]['events_not_assigned_to_angular_bins'], 0)
            self.assertEqual(results[0]['manual_evk4_motion_s_used'], 1.1)
            self.assertEqual(trace['setup']['manual_evk4_motion_s'], '1.2')
            self.assertEqual(len(sectors), 45)
            row['events_per_second'] = 999
            write_csv(root/'event_statistics.csv', [row])
            with self.assertRaisesRegex(ValueError, 'does not reconcile'):
                import_event_results(root, [trace], 72, 297)

    def test_full_turn_starts_in_selected_revolution_and_crosses_wrapped_zero(self):
        t = np.arange(11, dtype=float)
        rows = [dict(recorded_motor_feedback_rpm='30', lattepanda_time_s=str(time)) for time in t]
        trace = dict(rows=rows, selected_rev=1, angles=3.4 + 90 * t,
                     commands=np.full(11, 15.0), setup=dict(manual_evk4_motion_s='1', studio_duration_s='20'))
        first, last, level, _ = event_interval(trace)
        self.assertEqual(level, 364)
        self.assertAlmostEqual(last-first, 4)
        self.assertAlmostEqual(first, 1 + (364-3.4)/90)
        self.assertEqual((level+360)%360, 4)
        self.assertEqual(len(angular_edges(4,364))-1, 72)
        self.assertEqual(set(CONDITIONS), {'Low-Speed', 'Medium-Speed', 'High-Speed'})

    def test_eight_seconds_at_500_defines_ramp_rate_not_time_to_each_target(self):
        times = np.array([0, .1, 2.5, 4.9, 10.1, 12.5, 14.9, 15.1])
        actual = ramp_reference(times, .1, 10.1, 300, 500 / 8)
        np.testing.assert_allclose(actual, [0, 0, 150, 300, 300, 150, 0, 0], atol=1e-10)

    def test_weighted_mean_and_sd_do_not_depend_on_sampling_density(self):
        for t in ([0, 1], [0, .01, .02, .03, 1]):
            stats = weighted_stats(np.array(t), np.array(t) * 2, .2, .8)
            self.assertAlmostEqual(stats['mean'], 1)
            self.assertAlmostEqual(stats['sd'], 1.2 / math.sqrt(12))
            self.assertAlmostEqual(stats['peak_to_peak'], 1.2)

    def test_rise_and_fall_crossings_interpolate_and_respect_stop_time(self):
        t = np.arange(5, dtype=float)
        y = np.array([0, 10, 10, 0, 0])
        self.assertAlmostEqual(crossing(t, y, 9), .9)
        self.assertAlmostEqual(crossing(t, y, 9, start=2, rising=False), 2.1)

    def test_stopped_confirmation_restarts_after_a_failed_sample(self):
        t = np.array([10, 10.1, 10.2, 10.3, 10.4, 10.5, 10.6])
        mask = [True, True, False, True, True, True, True]
        self.assertAlmostEqual(sustained(t, mask, .25, start=10), 10.3)

    def test_command_stop_uses_first_zero_after_motion_not_final_verification(self):
        events = [dict(event='motor_command_write_completed', motor_pc_monotonic_ns=str(t * 10**9),
                       details=f'command={value:+.3f} rpm')
                  for t, value in [(1, 0), (2, 300), (12, 0), (17, 0)]]
        self.assertEqual(command_events(events, 10**9), (1, 11, 300))

    def test_circular_reference_and_negative_continuous_encoder_angles(self):
        np.testing.assert_allclose(circular_delta([359.9, .1], 359.9), [0, .2], atol=1e-10)
        level = angle_level(-19436.42925, 11, 72)
        self.assertEqual(level % 360, 72)
        self.assertTrue(-19436.42925 + 11 * 360 <= level < -19436.42925 + 12 * 360)

    @unittest.skipUnless(importlib.util.find_spec('eventcv'), 'RAW decoder is available on Linux; event counting is tested with NPZ fixtures below')
    def test_raw_counts_use_continuous_half_open_sector_and_disjoint_windows(self):
        # First CD event anchors raw time. Three events fall inside 72–297 deg;
        # the event at the exact sector endpoint must be excluded.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / 'test.raw'
            words = []
            entries = [(5_001_000, 10, 1), (5_051_000, 100, 1),
                       (5_052_000, 100, 0), (5_053_000, 101, 1),
                       (5_207_250, 102, 1), (5_301_000, 103, 1)]
            for time, x, polarity in entries:
                words.extend((0x8000 | (time >> 12), 0x6000 | (time & 0xFFF),
                              100, 0x2000 | (polarity << 11) | x))
            raw.write_bytes(b'% format EVT3\n% geometry 640x480\n% end\n'
                            + b''.join(word.to_bytes(2, 'little') for word in words))
            rows = [dict(recorded_motor_feedback_rpm='1', lattepanda_time_s=str(t))
                    for t in (0, .1, .2, .3)]
            trace = dict(condition='test', run='run1', experiment=root, selected_rev=0,
                         setup=dict(raw_file='test.raw', manual_evk4_motion_s='0', studio_duration_s='.3'),
                         rows=rows, angles=np.array([0, 144, 288, 432]))
            result, sectors = event_statistics(trace, 72, 297)
            self.assertEqual(result['cd_events'], 3)
            self.assertEqual(sum(row['cd_events'] for row in sectors), 3)
            self.assertEqual(result['complete_2ms_windows'], 78)
            self.assertAlmostEqual(result['mean_events_per_2ms'], 3 / 78)
            self.assertAlmostEqual(result['on_fraction'], 2 / 3)
            self.assertAlmostEqual(result['events_per_degree'], 3 / 225)
            self.assertAlmostEqual(result['duration_s'], .15625)
            full, full_sectors = event_statistics(trace)
            self.assertEqual(full['angular_extent_deg'], 360)
            self.assertEqual(full['wrapped_start_angle_deg'], full['wrapped_end_angle_deg'])
            self.assertAlmostEqual(full['duration_s'], .25)
            self.assertEqual(full['cd_events'], 4)
            self.assertEqual(sum(s['cd_events'] for s in full_sectors), 4)
            self.assertEqual(len(full_sectors), 72)
            self.assertAlmostEqual(full['events_per_degree'], 4/360)


if __name__ == '__main__':
    unittest.main()
