"""Numerical tests. Generated event fixtures are synthetic, never experimental results."""
import contextlib
import csv
import io
import json
import tempfile
import types
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

from rig_vpr.algorithm import EventSource, circular_error, count_frame, descriptors, evaluate_scores, match, metrics, pixel_indices, sad_matrix
from rig_vpr.data import CONDITIONS, check_inputs, discover, frame_plan, load_protocol, load_trace, validate_protocol
from generate_vpr_time_bins import build_motion_start_anchor, build_time_bins, load_motion_rows
from sad_vpr import main, run
from rig_vpr.rescore import rescore
from rig_vpr.data import sha256
from rig_vpr.references import compare_references
from rig_vpr.angular import angular_study
from rig_vpr.sectors import analyse_sectors, sector_index
from rig_vpr.stationary import activity_spectrum, analyse_stream, stationary_analysis
from rig_vpr.revolutions import revolution_study, parse_revolutions


def config(**updates):
    cfg = dict(accumulation_ms=2., angle_step_deg=90., tolerance_deg=2., start_wrapped_deg=4.,
        width=4, height=2, pixels=0, seed=0, steady_tolerance_percent=5., polarity="sum", normalization="none")
    cfg.update(updates)
    return cfg


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def make_fixture(root):
    """Nine toy acquisitions, each with four distinctive count frames."""
    for condition, (speed, revolution) in CONDITIONS.items():
        setup = []
        for r in (1,2,3):
            uid = condition.lower().replace("-", "")+str(r)
            recorded = Path("runs")/f"run{r}"/"recorded.csv"
            camera = Path("runs")/f"run{r}"/"camera.json"
            raw = Path("raw")/(uid+".npz")
            times = np.arange(0, 17.01, .01)
            angles = 3.5 + speed*6*times
            motion = [dict(run_id=uid, lattepanda_time_s=f"{t:.9f}", motor_pc_monotonic_ns=str(round(t*1e9)+1),
                motor_speed_command_rpm=str(speed*2), recorded_motor_feedback_rpm=str(speed*2),
                recorded_platform_feedback_rpm=str(speed), encoder_position_pulses=str(i),
                platform_angle_deg=str(a), platform_wrapped_angle_deg=str(a%360)) for i,(t,a) in enumerate(zip(times,angles))]
            folder = root/condition
            write_csv(folder/recorded, motion)
            (folder/camera).write_text(json.dumps(dict(run_id=uid, camera_healthy_at_finalize=True, applied_biases={"bias_diff":0})), encoding="utf-8")
            entry = dict(key=f"{condition}/run{r}", nominal_platform_rpm=speed, revolution=revolution, studio_duration_s=20.)
            trace = dict(times=times+1, angles=angles, speed=np.full(len(times),speed), command=np.full(len(times),speed))
            plan = frame_plan(trace, entry, config())
            events=[]
            for i, center in enumerate(plan["center_us"]):
                events.extend([[i,0,1_234_567+int(center),0]]*(i+1))
            (folder/raw).parent.mkdir(parents=True, exist_ok=True)
            np.savez(folder/raw, events=np.array(events,dtype=np.int64), sensor_size=np.array([4,2]), raw_origin_us=np.int64(1_234_567))
            setup.append(dict(include="true", run_label=f"run{r}", sync_mode="manual", run_id=uid,
                recorded_csv=recorded.as_posix(), raw_file=raw.as_posix(), camera_metadata=camera.as_posix(),
                manual_evk4_motion_s="1", studio_duration_s="20"))
        write_csv(root/condition/"experiment_setup.csv", setup)


class SadVprTests(unittest.TestCase):
    def test_stationary_spectrum_resolves_periodicity_and_rejects_dc(self):
        t=np.arange(1000)*.0005
        f,p,stats=activity_spectrum(100+40*np.sin(2*np.pi*100*t),500)
        self.assertEqual(stats['strongest_peak_hz'],100.)
        self.assertEqual(stats['frequency_resolution_hz'],2.)
        self.assertGreater(stats['power_fraction_near_100hz'],.99)
        _,p,stats=activity_spectrum(np.full(1000,100),500)
        self.assertIsNone(stats['strongest_peak_hz'])
        self.assertTrue(np.all(p==0))

    def test_stationary_stream_counts_half_open_bins_and_pixel_maps(self):
        events=np.array([[0,0,1000,1],[1,0,1499,0],[1,0,1500,1],[0,0,2000,0]])
        stream=types.SimpleNamespace(origin_us=1000,sensor_size=(2,1),events_between=lambda a,b:events[(events[:,2]>=1000+a)&(events[:,2]<1000+b)])
        counts,maps=analyse_stream(stream,0,1000,500)
        np.testing.assert_array_equal(counts,[[1,0],[1,1]])
        self.assertEqual(int(maps.sum()),3)
        np.testing.assert_array_equal(maps,[[[0,1]],[[1,1]]])

    def test_stationary_all_nine_and_prepare_only_preserve_source(self):
        with tempfile.TemporaryDirectory() as tmp,contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp);make_fixture(root)
            for raw in root.glob('*/raw/*.npz'):
                with np.load(raw,allow_pickle=False) as saved:
                    data={k:saved[k].copy() for k in saved.files}
                origin=int(data['raw_origin_us'])
                extra=[]
                for time in range(300000,800000,10000):
                    extra.extend([[0,0,origin+time,1]]*5)
                    extra.extend([[1,1,origin+time+5000,0]]*5)
                data['events']=np.vstack([data['events'],np.array(extra,dtype=np.int64)])
                data['events']=data['events'][np.argsort(data['events'][:,2],kind='stable')]
                np.savez(raw,**data)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            source=run(path,no_plots=True)
            frozen={p:sha256(p) for p in source.rglob('*') if p.is_file()}
            with patch('rig_vpr.stationary.EventSource',side_effect=AssertionError('Prepare must not decode RAW')):
                output=stationary_analysis(source,prepare_only=True,no_plots=True)
            self.assertFalse((output/'stationary_complete.json').exists())
            stationary_analysis(source,no_plots=True)
            with (output/'stationary_summary.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),9)
            self.assertTrue(all(int(r['total_events'])==500 and float(r['on_fraction'])==.5 for r in rows))
            # A pulse train has multiple harmonics; its strongest bin need not be the fundamental.
            self.assertTrue(all(float(r['on_strongest_peak_hz']) in (100,200,300,400,500) for r in rows))
            self.assertTrue(all(float(r['on_power_fraction_near_100hz'])>.1 for r in rows))
            self.assertTrue(all(r['encoder_fully_covers_interval']=='False' for r in rows))
            self.assertTrue((output/'stationary_complete.json').is_file())
            self.assertTrue(all(sha256(p)==value for p,value in frozen.items()))
            with self.assertRaises(FileExistsError):
                stationary_analysis(source,no_plots=True)
            with self.assertRaisesRegex(ValueError,'insufficient guarded'):
                stationary_analysis(source,duration_s=2,no_plots=True)

    def test_forward_screen_allows_acceleration_without_relaxing_default(self):
        times=np.arange(0,10.01,.01)
        trace=dict(times=times,angles=3.5+30*times+3*times**2,speed=5+times,command=np.full(len(times),15.))
        entry=dict(key='test/run1',nominal_platform_rpm=15.,revolution=0,studio_duration_s=12.)
        with self.assertRaisesRegex(ValueError,'steady screening'):
            frame_plan(trace,entry,config())
        plan=frame_plan(trace,entry,config(motion_screen='forward'))
        self.assertGreater(plan['worst_speed_error_percent'],5)
        self.assertLess(plan['minimum_feedback_rpm'],plan['maximum_feedback_rpm'])
        self.assertEqual(parse_revolutions('0,2-4',trace),[0,2,3,4])
        with self.assertRaisesRegex(ValueError,'nonnegative'):
            parse_revolutions('-1',trace)

    def test_revolution_study_fixes_reference_and_reuses_original_turns(self):
        with tempfile.TemporaryDirectory() as tmp,contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp);make_fixture(root)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            protocol,_=load_protocol(path)
            # Add toy events to an earlier complete turn before freezing RAW extraction.
            for entry in protocol['acquisitions']:
                raw=root/entry['raw_file']
                with np.load(raw,allow_pickle=False) as saved:
                    data={k:saved[k].copy() for k in saved.files}
                trace=load_trace(root,entry)
                earlier=frame_plan(trace,dict(entry,revolution=entry['revolution']-1),config())
                extra=[]
                for i,center in enumerate(earlier['center_us']):
                    extra.extend([[i,0,int(data['raw_origin_us'])+int(center),0]]*(i+1))
                data['events']=np.vstack([data['events'],np.array(extra,dtype=np.int64)])
                data['events']=data['events'][np.argsort(data['events'][:,2],kind='stable')]
                np.savez(raw,**data)
            source=run(path,no_plots=True)
            frozen={p:sha256(p) for folder in (source,path.parent/'cache') for p in folder.rglob('*') if p.is_file()}
            args=dict(low='0,1,999',medium='2,3',high='10,11',no_plots=True)
            with patch('rig_vpr.revolutions.extract',side_effect=AssertionError('Prepare must not extract')):
                output=revolution_study(source,prepare_only=True,**args)
            self.assertFalse((output/'revolution_study_complete.json').exists())
            revolution_study(source,**args)
            with (output/'summary_per_turn.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),32)
            self.assertTrue(all(r['reference']=='Low-Speed/run1' and r['reference_revolution']=='1' for r in rows))
            self.assertTrue(all(r['query']!='Low-Speed/run1' and float(r['recall_at_5'])==1 for r in rows))
            self.assertEqual(len(list((output/'cache').glob('*.npz'))),8)
            with (output/'planned_turns.csv').open() as stream:
                planned=list(csv.DictReader(stream))
            self.assertEqual(sum(r['status']=='skipped' for r in planned),2)
            with (output/'delta_vs_original_turn.csv').open() as stream:
                delta=list(csv.DictReader(stream))
            self.assertTrue(all(float(r['change_pp'])==0 for r in delta))
            self.assertTrue(all(sha256(p)==value for p,value in frozen.items()))
            self.assertTrue((output/'revolution_study_complete.json').exists())

    def test_sector_boundaries_are_half_open_and_wrap(self):
        self.assertEqual([sector_index(v,30) for v in (0,29.999999,30,359.999,360,-.1)],[0,0,1,11,0,11])
        with self.assertRaisesRegex(ValueError,'finite'):
            sector_index(float('nan'),30)
        with self.assertRaisesRegex(ValueError,'divide 360'):
            analyse_sectors('unused','unused',sector_deg=31)

    def test_angular_windows_follow_crossings_not_nominal_time_scaling(self):
        joint=94/87
        times=np.r_[np.linspace(0,joint,101),np.linspace(joint,5,101)[1:]]
        angles=np.where(times<=joint,87*times,94+93*(times-joint))
        trace=dict(times=times,angles=angles,speed=np.where(times<=joint,14.5,15.5),command=np.full(len(times),15.))
        cfg=config(accumulation_mode='angle',accumulation_deg=.6)
        entry=dict(key='test/run1',nominal_platform_rpm=15.,revolution=0,studio_duration_s=8.)
        plan=frame_plan(trace,entry,cfg)
        np.testing.assert_allclose(plan['window_angle_span_deg'],.6,atol=.0001)
        self.assertAlmostEqual(plan['start_us'][1]/1e6,joint-.3/87,places=6)
        self.assertAlmostEqual(plan['end_us'][1]/1e6,joint+.3/93,places=6)
        self.assertGreater(abs((plan['start_us'][1]+plan['end_us'][1])/2-plan['center_us'][1]),50)
        temporal=frame_plan(trace,entry,config())
        np.testing.assert_array_equal(plan['center_us'],temporal['center_us'])
        with self.assertRaisesRegex(ValueError,'No forward encoder crossing'):
            frame_plan(trace,entry,config(accumulation_mode='angle',accumulation_deg=20.))
        with self.assertRaisesRegex(ValueError,'at least 1 microsecond'):
            frame_plan(trace,entry,config(accumulation_mode='angle',accumulation_deg=1e-8))

    def test_angular_study_full_pipeline_and_prepare_only(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp);make_fixture(root)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            source=run(path,no_plots=True)
            frozen={p:sha256(p) for p in source.rglob('*') if p.is_file()}
            prepared=angular_study(source,prepare_only=True,no_plots=True)
            self.assertTrue((prepared/'protocol.json').is_file())
            self.assertFalse((prepared/'results').exists())
            self.assertFalse((prepared/'window_study_complete.json').exists())
            output=angular_study(source,no_plots=True)
            with (output/'window_comparison_by_speed.csv').open() as stream:
                groups=list(csv.DictReader(stream))
            with (output/'window_comparison_per_pair.csv').open() as stream:
                pairs=list(csv.DictReader(stream))
            with (output/'planned_frame_windows.csv').open() as stream:
                planned=list(csv.DictReader(stream))
            self.assertEqual(len(groups),18)
            self.assertEqual(len(pairs),144)
            self.assertTrue(all(float(r['change_pp_at_5'])==0 for r in groups))
            self.assertTrue(all(abs(float(r['angular_span_deg'])-.6)<.002 for r in planned))
            self.assertEqual({r['normalization'] for r in groups},{'none','l1'})
            self.assertTrue((output/'window_study_complete.json').is_file())
            self.assertTrue(all(sha256(p)==value for p,value in frozen.items()))
            # Repeating a completed study uses its existing extraction/comparisons.
            with patch('sad_vpr.run',side_effect=AssertionError('Must reuse completed angular extraction')):
                self.assertEqual(angular_study(source,no_plots=True),output)
            with self.assertRaisesRegex(ValueError,'separate protocol directory'):
                angular_study(source,output_root=source.parent,prepare_only=True,no_plots=True)
            with self.assertRaisesRegex(ValueError,'disagrees'):
                angular_study(source,accumulation_deg=.8,output_root=output,prepare_only=True,no_plots=True)

    def test_count_frame_preserves_counts_and_polarity(self):
        events=np.array([[0,0,1,0],[0,0,2,1],[3,1,3,1]])
        np.testing.assert_array_equal(count_frame(events,(4,2),(2,1)), [[[2,1]]])
        np.testing.assert_array_equal(count_frame(events,(4,2),(2,1),"split"), [[[1,0]],[[1,1]]])
        with self.assertRaisesRegex(ValueError,"bounds"):
            count_frame(np.array([[4,0,1,0]]),(4,2),(2,1))

    def test_sad_orientation_and_unsigned_subtraction(self):
        ref=np.array([[0,10],[255,0]],dtype=np.uint8)
        qry=np.array([[250,0],[1,9],[0,1]],dtype=np.uint8)
        matrix,backend=sad_matrix(ref,qry,backend="numpy")
        np.testing.assert_array_equal(matrix,[[260,2,9],[5,263,256]])
        self.assertEqual(matrix.shape,(2,3))
        self.assertEqual(backend,"numpy-blocked-l1")

    def test_circular_error_wraps(self):
        np.testing.assert_allclose(circular_error([1,359,180],[359,1,0]),[2,-2,-180])

    def test_recall_at_five_and_cutoff_ties_are_separate_from_angle_tolerance(self):
        angles=np.array([90.,120.,150.,180.,359.,1.,250.])
        scores=np.column_stack([
            [0,1,2,3,4,5,6],  # Correct reference is rank 5, across wrapped zero.
            [0,1,2,3,4,4,6],  # Correct candidates tie across ranks 5 and 6.
            [0,1,2,3,4,4,6],  # Same boundary tie, but correct rank 1 still counts.
            [1,1,1,1,1,1,1],  # All seven tied: top five cannot be established.
            [0,0,2,3,4,5,6],  # A tied best group wholly within the top five.
            [0,1,2,3,4,5,6],  # Empty query stays a failure.
        ]).astype(float)
        query_angles=np.array([0.,0.,90.,0.,120.,0.])
        result=evaluate_scores(scores,angles,query_angles,np.ones(7,dtype=bool),np.array([1,1,1,1,1,0],dtype=bool),2.)
        self.assertEqual(result['correct'].tolist(),[False,False,True,False,False,False])
        self.assertEqual(result['correct_at_5'].tolist(),[True,False,True,False,True,False])
        self.assertEqual(result['top5_boundary_tie'].tolist(),[False,True,True,True,False,False])
        summary=metrics(result,2.)
        self.assertAlmostEqual(summary['recall_at_1'],1/6)
        self.assertAlmostEqual(summary['recall_at_5'],3/6)
        self.assertEqual(result['top_indices'][:,0].tolist(),[0,1,2,3,4])
        self.assertFalse(result['top_credited'][4,1])

    def test_top_five_ranking_excludes_empty_references_and_handles_fewer_than_five(self):
        result=evaluate_scores(np.array([[0,0],[2,2],[3,3.]]),np.array([0.,90.,180.]),np.array([0.,90.]),
            np.array([False,True,True]),np.array([True,True]),2.)
        self.assertEqual(result['top5_effective_k'],2)
        self.assertEqual(result['top_indices'][:,0].tolist(),[1,2])
        self.assertEqual(result['correct_at_5'].tolist(),[False,True])

    def test_top_five_order_never_uses_encoder_truth(self):
        scores=np.arange(7,dtype=float)[:,None]
        a=evaluate_scores(scores,np.arange(7)*30.,np.array([120.]),np.ones(7,dtype=bool),np.ones(1,dtype=bool),2.)
        b=evaluate_scores(scores,np.arange(7)*30.,np.array([180.]),np.ones(7,dtype=bool),np.ones(1,dtype=bool),2.)
        np.testing.assert_array_equal(a['top_indices'],b['top_indices'])
        self.assertTrue(a['correct_at_5'].item())
        self.assertFalse(b['correct_at_5'].item())

    def test_match_never_uses_ground_truth_to_choose_reference(self):
        ref=dict(frames=np.array([[[[1,0]]],[[[0,10]]]],dtype=np.float32),angle_deg=np.array([0.,180.]))
        qry=dict(frames=np.array([[[[0,10]]]],dtype=np.float32),angle_deg=np.array([0.]))
        result=match(ref,qry,config(),np.arange(2))
        self.assertEqual(result["best"].item(),1)
        self.assertFalse(result["correct"].item())
        qry["angle_deg"][:]=180
        changed=match(ref,qry,config(),np.arange(2))
        np.testing.assert_array_equal(changed["best"],result["best"])
        self.assertTrue(changed["correct"].item())

    def test_empty_and_tied_queries_are_failed_not_removed(self):
        ref=dict(frames=np.array([[[[1,0]]],[[[1,0]]],[[[0,2]]],[[[0,0]]]],dtype=np.float32),angle_deg=np.array([0.,90.,180.,270.]))
        qry=dict(frames=np.array([[[[0,0]]],[[[1,0]]],[[[0,2]]]],dtype=np.float32),angle_deg=np.array([0.,0.,180.]))
        result=match(ref,qry,config(),np.arange(2)); summary=metrics(result,2)
        self.assertEqual(result["status"].tolist(),["empty_query_descriptor","ambiguous_tie","matched"])
        self.assertEqual(summary["query_count"],3)
        self.assertEqual(summary["correct_queries"],1)
        self.assertAlmostEqual(summary["recall_at_1"],1/3)
        self.assertTrue(np.isinf(result["scores"][3]).all())

    def test_seeded_sparse_bins_shared_and_l1(self):
        cfg=config(pixels=3,seed=12)
        np.testing.assert_array_equal(pixel_indices(cfg),pixel_indices(cfg))
        self.assertEqual(len(set(pixel_indices(cfg))),3)
        values,valid=descriptors(np.array([[[[1,3]]],[[[0,0]]]],dtype=np.float32),np.arange(2),"l1")
        np.testing.assert_allclose(values,[[.25,.75],[0,0]])
        self.assertEqual(valid.tolist(),[True,False])

    def test_frame_plan_full_turn_centres_and_steady_aperture(self):
        times=np.arange(0,12,.01); angles=3.5+90*times
        trace=dict(times=times,angles=angles,speed=np.full(len(times),15.),command=np.full(len(times),15.))
        entry=dict(key="test",nominal_platform_rpm=15.,revolution=1,studio_duration_s=20.)
        plan=frame_plan(trace,entry,config())
        np.testing.assert_allclose(plan["angle_deg"],[4,94,184,274],atol=.0001)
        np.testing.assert_allclose(plan["window_angle_span_deg"],.18,atol=1e-6)
        self.assertTrue(np.all(plan["end_us"]-plan["start_us"]==2000))
        trace["speed"][np.searchsorted(times,plan["start_us"][0]/1e6)-1]=10
        with self.assertRaisesRegex(ValueError,"steady screening"):
            frame_plan(trace,entry,config())

    def test_npz_reader_enforces_half_open_timestamp_bounds(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"fixture.npz"
            np.savez(path,events=np.array([[0,0,100,0],[0,0,101,1],[0,0,102,0]]),sensor_size=[2,1],raw_origin_us=100)
            source=EventSource(path,20)
            self.assertEqual(source.events_between(0,2)[:,2].tolist(),[100,101])
            source.close()

    def test_raw_reader_preserves_quiet_lead_in_and_absolute_timestamp_domain(self):
        requests=[]
        class Reader:
            sensor_size=(4,2)
            def time_span(self,unit):
                return (12_000_000, 30_000_000)
            def slice(self, **kwargs):
                requests.append(kwargs)
                return types.SimpleNamespace(numpy=lambda: np.array([[0,0,kwargs["t0_us"],0],[0,0,kwargs["t1_us"],0]]))
        fake=types.SimpleNamespace(open=lambda *a,**k: Reader())
        with patch.dict("sys.modules", {"eventcv":fake}):
            source=EventSource(Path("fixture.raw"),20.)
            self.assertEqual(source.origin_us,10_000_000)
            events=source.events_between(1_000_000,1_002_000)
            self.assertEqual(requests,[dict(t0_us=11_000_000,t1_us=11_002_000)])
            self.assertEqual(events[:,2].tolist(),[11_000_000])
            source.close()

    def test_scipy_and_numpy_sad_agree_when_available(self):
        try:
            from scipy.spatial.distance import cdist
        except ImportError:
            self.skipTest("Optional SciPy unavailable")
        rng=np.random.default_rng(11)
        reference=rng.integers(0,100,size=(37,23)).astype(np.float32)
        query=rng.integers(0,100,size=(13,23)).astype(np.float32)
        a,_=sad_matrix(reference,query,"numpy")
        b,_=sad_matrix(reference,query,"scipy")
        np.testing.assert_array_equal(a,b)

    def test_duplicate_acquisition_ids_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); make_fixture(root)
            protocol=discover(root,root/"out/protocol.json",config())
            protocol["acquisitions"][1]["run_id"]=protocol["acquisitions"][0]["run_id"]
            with self.assertRaisesRegex(ValueError,"run_id"):
                validate_protocol(protocol)

    def test_prepared_onset_wins_and_exact_encoder_timestamps_are_used(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); make_fixture(root)
            folder=root/"Low-Speed"
            recorded=folder/"runs/run1/recorded.csv"
            with recorded.open() as stream:
                motion=list(csv.DictReader(stream))
            motion[0]["lattepanda_time_s"]="0.001"
            write_csv(recorded,motion)
            anchor=build_motion_start_anchor(motion,1_000_000)
            bins=build_time_bins(motion,anchor)
            write_csv(folder/"outputs/time_bins/run1.csv",bins)
            with (folder/"experiment_setup.csv").open() as stream:
                setup=list(csv.DictReader(stream))
            setup[0]["manual_evk4_motion_s"]="1.4"
            write_csv(folder/"experiment_setup.csv",setup)
            write_csv(folder/"outputs/vpr_runs.csv",[dict(run_label="run1",recorded_csv="../runs/run1/recorded.csv",
                raw_file="../raw/lowspeed1.npz",time_bins_csv="time_bins/run1.csv",studio_duration_s="20",manual_evk4_motion_s="1",sync_mode="manual")])
            protocol=discover(root,root/"VPR/protocol.json",config())
            entry=protocol["acquisitions"][0]
            self.assertEqual(entry["manual_evk4_motion_s"],1.)
            self.assertTrue(any("prepared value retained" in s for s in protocol["discovery_warnings"]))
            trace=load_trace(root,entry)
            expected=np.array([b["encoder_sample_camera_us"]/1e6 for b in bins])
            np.testing.assert_array_equal(trace["times"],expected)
            midpoint=np.array([(b["camera_start_us"]+b["camera_end_us"])/2e6 for b in bins])
            self.assertGreater(float(np.max(abs(midpoint-expected))),1e-5)

    def test_optional_all_pairs_excludes_each_same_acquisition(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp); make_fixture(root)
            path=root/"AllPairs/protocol.json"
            main(["init","--experiments-root",str(root),"--output",str(path),"--width","4","--height","2","--angle-step-deg","90","--all-pairs"])
            output=run(path,no_plots=True)
            with (output/"summary_per_acquisition.csv").open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),72)
            self.assertTrue(all(r["reference"]!=r["query"] for r in rows))

    def test_rescore_uses_only_saved_matrices_and_preserves_original_results(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp); make_fixture(root)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            source=run(path,no_plots=True)
            old_hashes={p.relative_to(source).as_posix():sha256(p) for p in source.rglob('*') if p.is_file()}
            for raw in root.glob('*/raw/*.npz'):
                raw.unlink()
            with patch('rig_vpr.algorithm.EventSource',side_effect=AssertionError('Rescoring must not decode RAW')):
                destination=rescore(source,no_plots=True)
            with (destination/'summary_per_acquisition.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),8)
            self.assertTrue(all(float(r['recall_at_5'])==1. for r in rows))
            self.assertEqual(old_hashes,{p.relative_to(source).as_posix():sha256(p) for p in source.rglob('*') if p.is_file()})
            with self.assertRaises(FileExistsError):
                rescore(source,no_plots=True)
            with self.assertRaisesRegex(ValueError,'separate'):
                rescore(source,output_root=source,no_plots=True)
            # A bad legacy summary cannot silently be replaced with inconsistent scores.
            old_summary=source/'summary_per_acquisition.csv'
            with old_summary.open() as stream:
                tampered=list(csv.DictReader(stream))
            tampered[0]['correct_queries']='0'
            write_csv(old_summary,tampered)
            failed_output=root/'BadRescore'
            with self.assertRaisesRegex(ValueError,'regression'):
                rescore(source,failed_output,no_plots=True)
            self.assertFalse(failed_output.exists())

    def test_reference_comparison_uses_cached_frames_and_all_72_directed_pairs(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp); make_fixture(root)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            source=run(path,no_plots=True)
            original={p.relative_to(source).as_posix():sha256(p) for p in source.rglob('*') if p.is_file()}
            for raw in root.glob('*/raw/*.npz'):
                raw.unlink()
            with patch('rig_vpr.algorithm.EventSource',side_effect=AssertionError('Comparison must not decode RAW')):
                output=compare_references(source,tolerance=5.,no_plots=True)
            with (output/'summary_per_pair.csv').open() as stream:
                pairs=list(csv.DictReader(stream))
            with (output/'summary_by_reference.csv').open() as stream:
                summaries=list(csv.DictReader(stream))
            with (output/'summary_by_reference_speed.csv').open() as stream:
                groups=list(csv.DictReader(stream))
            self.assertEqual(len(pairs),72)
            self.assertEqual(len(summaries),27)
            self.assertEqual(len(groups),9)
            self.assertTrue(all(r['reference']!=r['query'] and float(r['tolerance_deg'])==5 for r in pairs))
            self.assertTrue(all(float(r['recall_at_5'])==1 for r in pairs))
            audit=json.loads((output/'comparison_provenance.json').read_text())
            self.assertEqual(audit['reused_unique_matrices'],8)
            self.assertEqual(audit['computed_unique_matrices'],28)
            self.assertEqual([int(r['ordered_pairs']) for r in groups],[6,9,9,9,6,9,9,9,6])
            self.assertEqual(original,{p.relative_to(source).as_posix():sha256(p) for p in source.rglob('*') if p.is_file()})
            with self.assertRaises(FileExistsError):
                compare_references(source,tolerance=5.,no_plots=True)
            # Corrupting numeric frames cannot silently enter a new comparison.
            cache=next((path.parent/'cache').glob('*.npz'))
            with np.load(cache,allow_pickle=False) as stored:
                contents={k:stored[k].copy() for k in stored.files}
            contents['frames'][0,0,0,0]+=1
            np.savez_compressed(cache,**contents)
            with self.assertRaisesRegex(ValueError,'reconcile'):
                compare_references(source,tolerance=5.,output_root=root/'BadComparison',no_plots=True)

    def test_reference_comparison_recomputes_masked_saved_matrices(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp); make_fixture(root)
            raw=root/'Low-Speed/raw/lowspeed1.npz'
            with np.load(raw,allow_pickle=False) as stored:
                contents={k:stored[k].copy() for k in stored.files}
            contents['events']=contents['events'][contents['events'][:,0]!=0]
            np.savez(raw,**contents)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            source=run(path,no_plots=True)
            output=compare_references(source,tolerance=5.,no_plots=True)
            audit=json.loads((output/'comparison_provenance.json').read_text())
            self.assertEqual(audit['reused_unique_matrices'],0)
            self.assertEqual(audit['computed_unique_matrices'],36)

    def test_l1_normalization_removes_per_frame_count_scaling_and_preserves_empties(self):
        frames=np.array([[[[1,3,0]]],[[[10,30,0]]],[[[0,0,0]]]],dtype=np.float32)
        original=frames.copy()
        values,valid=descriptors(frames,np.arange(3),'l1')
        np.testing.assert_allclose(values,[[.25,.75,0],[.25,.75,0],[0,0,0]])
        np.testing.assert_array_equal(valid,[True,True,False])
        np.testing.assert_array_equal(frames,original)
        scores,_=sad_matrix(values[:1],values[1:2])
        self.assertEqual(scores.item(),0.)

    def test_normalized_comparison_recomputes_distances_and_pairs_with_raw_baseline(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp); make_fixture(root)
            path=root/'VPR/protocol.json'
            main(['init','--experiments-root',str(root),'--output',str(path),'--width','4','--height','2','--angle-step-deg','90'])
            source=run(path,no_plots=True)
            baseline=compare_references(source,no_plots=True)
            frozen={p:sha256(p) for folder in (source,baseline,path.parent/'cache') for p in folder.rglob('*') if p.is_file()}
            for raw in root.glob('*/raw/*.npz'):
                raw.unlink()
            with patch('rig_vpr.algorithm.EventSource',side_effect=AssertionError('Must reuse cached counts')):
                output=compare_references(source,normalization='l1',baseline_comparison=baseline,no_plots=True)
            self.assertEqual(output.name,'reference_comparison_l1_5deg')
            audit=json.loads((output/'comparison_provenance.json').read_text())
            self.assertEqual(audit['reused_unique_matrices'],0)
            self.assertEqual(audit['computed_unique_matrices'],36)
            self.assertEqual(audit['source_protocol']['settings']['normalization'],'none')
            self.assertEqual(audit['evaluation_settings']['normalization'],'l1')
            self.assertTrue(audit['unchanged_cached_count_frames'])
            self.assertFalse(audit['unchanged_cached_representation'])
            with (output/'normalization_comparison_per_pair.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),72)
            self.assertTrue(all(float(r['change_pp_at_5'])==0 for r in rows))
            matrix=next((output/'distance_matrices').glob('*.npz'))
            with np.load(matrix,allow_pickle=False) as stored:
                np.testing.assert_array_equal(stored['scores'],2*(1-np.eye(4)))
            self.assertTrue(all(sha256(p)==value for p,value in frozen.items()))
            sectors=analyse_sectors(baseline,output,sector_deg=90,output_root=root/'Sectors',no_plots=True)
            with (sectors/'sectors_per_pair.csv').open() as stream:
                sector_pairs=list(csv.DictReader(stream))
            with (sectors/'sectors_by_reference_speed.csv').open() as stream:
                sector_groups=list(csv.DictReader(stream))
            self.assertEqual(len(sector_pairs),72*4*2)
            self.assertEqual(len(sector_groups),9*4*2)
            self.assertTrue(all(int(r['query_count'])==1 and float(r['recall_at_5'])==1 for r in sector_pairs))
            self.assertTrue(all(sha256(p)==value for p,value in frozen.items()))
            with self.assertRaisesRegex(ValueError,'same source protocol/tolerance'):
                compare_references(source,tolerance=2,normalization='l1',baseline_comparison=baseline,no_plots=True)
            self.assertFalse((path.parent/'reference_comparison_l1_2deg').exists())
            with (output/'query_metrics.csv').open() as stream:
                tampered=list(csv.DictReader(stream))
            tampered[0]['correct_at_5']='False'
            write_csv(output/'query_metrics.csv',tampered)
            with self.assertRaisesRegex(ValueError,'success flags'):
                analyse_sectors(baseline,output,sector_deg=90,output_root=root/'BadSectors',no_plots=True)
            self.assertFalse((root/'BadSectors').exists())

    def test_end_to_end_protocol_cache_and_eight_independent_pairs(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp); make_fixture(root)
            protocol_path=root/"VPR/protocol.json"
            main(["init","--experiments-root",str(root),"--output",str(protocol_path),"--width","4","--height","2","--angle-step-deg","90"])
            protocol,_=load_protocol(protocol_path)
            self.assertEqual(len(protocol["acquisitions"]),9)
            self.assertTrue(protocol["frozen_input_sha256"])
            output=run(protocol_path,no_plots=True)
            with (output/"summary_per_acquisition.csv").open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),8)
            self.assertTrue(all(r["query"]!=r["reference"] for r in rows))
            self.assertTrue(all(float(r["recall_at_1"])==1 for r in rows))
            with (output/"summary_by_speed.csv").open() as stream:
                summary=list(csv.DictReader(stream))
            self.assertEqual([int(r["independent_query_acquisitions"]) for r in summary],[2,3,3])
            self.assertEqual(len(list((protocol_path.parent/"cache").glob("*.npz"))),9)
            with self.assertRaises(FileExistsError):
                run(protocol_path,no_plots=True)
            run(protocol_path,no_plots=True,overwrite=True)
            self.assertTrue((output/"run_complete.json").is_file())
            self.assertEqual(len(list(protocol_path.parent.glob("results.previous-*"))),1)
            path=root/protocol["acquisitions"][0]["camera_metadata"]
            path.write_text(path.read_text()+"\n")
            with self.assertRaisesRegex(ValueError,"input changed"):
                check_inputs(protocol,root)


if __name__ == "__main__":
    unittest.main()
