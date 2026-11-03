import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

from evaluate_rig import event_statistics, trajectory_repeatability, weighted_absolute_mean, CONDITIONS, read_csv, write_csv, spatial_figures, event_vpr_comparison


class RigEventPipelineTests(unittest.TestCase):
    def trace(self, root, duration=1.):
        times = np.array([0., duration/2, duration, duration+.5])
        return dict(condition='Low-Speed',run='run1',experiment=root,selected_rev=0,
            rows=[dict(recorded_motor_feedback_rpm='30',lattepanda_time_s=str(t)) for t in times],
            angles=4+360*times/duration, accepted_camera_times=times,
            setup=dict(raw_file='events.npz',manual_evk4_motion_s='0',studio_duration_s='2'))

    def test_native_maps_and_centred_bins_reconcile_with_disjoint_windows(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            times = np.array([0,1999,2000,19999,20000,50000,999999,1000000])
            events = np.column_stack((np.zeros(len(times),dtype=int),np.zeros(len(times),dtype=int),
                1000000+times,np.arange(len(times))%2))
            np.savez(root/'events.npz',events=events,sensor_size=np.array([2,2]),raw_origin_us=1000000)
            result, detail = event_statistics(self.trace(root),artifact_root=root/'output')
            self.assertEqual(result['cd_events'],7)
            self.assertEqual(sum(row['cd_events'] for row in detail),7)
            self.assertEqual(result['complete_2ms_windows'],500)
            self.assertEqual(result['complete_window_events'],7)
            self.assertEqual(result['incomplete_tail_events'],0)
            self.assertEqual(result['longest_empty_2ms_sequence'],473)
            self.assertEqual(result['union_active_pixel_fraction'],.25)
            with np.load(root/'output'/result['event_arrays_file']) as saved:
                self.assertEqual(int((saved['on_counts']+saved['off_counts']).sum()),7)
                self.assertEqual(int(saved['on_counts'].sum()),3)
                self.assertEqual(int(saved['window_active_pixels'].sum()),6)
                self.assertEqual(saved['centred_30deg_counts'].tolist(),[6,1]+[0]*10)
            sectors = read_csv(root/'output/event_arrays/Low-Speed_run1_sectors_30deg.csv')
            self.assertEqual(len(sectors),12)
            self.assertAlmostEqual(sum(float(r['duration_s']) for r in sectors),1)
            self.assertTrue(all(float(r['measured_angular_extent_deg'])==30 for r in sectors))

    def test_partial_tail_is_counted_in_total_but_not_complete_windows(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            events = np.array([[0,0,1000000,0],[1,1,2000500,1],[1,1,2001000,1]])
            np.savez(root/'events.npz',events=events,sensor_size=np.array([2,2]),raw_origin_us=1000000)
            result, detail = event_statistics(self.trace(root,1.001))
            self.assertEqual(result['cd_events'],2)
            self.assertEqual(result['complete_window_events'],1)
            self.assertEqual(result['incomplete_tail_events'],1)
            self.assertEqual(result['complete_2ms_windows'],500)
            self.assertAlmostEqual(result['on_fraction'],.5)

    def test_pairwise_trajectory_statistics_keep_original_start_offsets(self):
        traces = []
        for condition in CONDITIONS:
            for i, initial in enumerate((359,1,2)):
                times = np.array([0.,1.,2.])
                traces.append(dict(condition=condition,run=f'run{i+1}',t=times,
                    displacement=(10+2*i)*times,steady_start=1.,steady_end=2.,initial_angle=initial))
        rows = trajectory_repeatability(traces)
        self.assertEqual(len(rows),9)
        row = rows[0]
        self.assertAlmostEqual(row['mean_signed_displacement_difference_deg'],3)
        self.assertAlmostEqual(row['displacement_mae_deg'],3)
        self.assertAlmostEqual(row['displacement_rmse_deg'],np.sqrt(28/3))
        self.assertAlmostEqual(row['initial_orientation_difference_deg'],2)

    def test_absolute_mean_integrates_sign_change(self):
        self.assertAlmostEqual(weighted_absolute_mean(np.array([0.,2.]),np.array([-1.,1.]),0,2),.5)

    def test_event_vpr_join_rejects_a_different_protocol(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source=root/'results'; sectors=root/'sectors_30deg_centred'
            source.mkdir(); sectors.mkdir()
            (source/'protocol_snapshot.json').write_text(json.dumps(dict(example=1)))
            (sectors/'sector_provenance.json').write_text(json.dumps(dict(source_protocol=dict(example=2))))
            (sectors/'sector_analysis_complete.json').write_text(json.dumps(dict(sector_deg=30,sector_start_deg=-15)))
            with self.assertRaisesRegex(ValueError,'differs'):
                event_vpr_comparison(source,[])

    def test_spatial_temporal_and_sector_figures_render_without_real_data(self):
        from matplotlib.figure import Figure
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); (root/'event_arrays').mkdir()
            rows, sectors = [], []
            for condition in CONDITIONS:
                for i in (1,2,3):
                    relative=f'event_arrays/{condition}_run{i}.npz'
                    np.savez(root/relative,on_counts=np.ones((2,3))*i,off_counts=np.ones((2,3)),
                        duration_s=.004,window_start_us=np.array([0,2000]),camera_start_us=0,
                        window_event_counts=np.array([2,3]))
                    rows.append(dict(condition=condition,run=f'run{i}',status='processed',event_arrays_file=relative))
                    sectors.extend(dict(condition=condition,run=f'run{i}',angle_start_deg=s,
                        events_per_degree=2,events_per_second=3) for s in range(-15,345,30))
            write_csv(root/'event_sector_counts_30deg_centred.csv',sectors)
            formats=[]
            def draw(figure,path,*args,**kwargs):
                figure.canvas.draw()
                formats.append(Path(path).suffix)
            with patch.object(Figure,'savefig',draw):
                spatial_figures(root,rows)
            self.assertEqual(formats.count('.pdf'),3)
            self.assertEqual(len(formats),9)
