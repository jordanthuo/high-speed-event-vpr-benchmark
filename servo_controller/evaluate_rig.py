"""Reproducible motor/position diagnostics and optional continuous RAW statistics.

Inputs are read only. Defaults analyse Low, Medium and High, three runs each.
Run on the recording computer with --events --no-plots to add RAW statistics.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

CONDITIONS = {"Low-Speed": (1, 1, 1), "Medium-Speed": (1, 6, 3), "High-Speed": (6, 17, 11)}


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: "" if value is None else value for key, value in row.items()})


def crossing(t, y, level, start=-math.inf, rising=True):
    for i in range(len(t) - 1):
        if t[i + 1] < start:
            continue
        if y[i + 1] == y[i]:
            continue
        inside = y[i] <= level <= y[i + 1] if rising else y[i] >= level >= y[i + 1]
        if inside:
            result = t[i] + (level - y[i]) / (y[i + 1] - y[i]) * (t[i + 1] - t[i])
            if result >= start:
                return float(result)
    return None


def sustained(t, mask, duration, start, end=math.inf):
    first = None
    for i, valid in enumerate(mask):
        if t[i] < start or t[i] > end or not valid:
            first = None
        elif first is None:
            first = i
        if first is not None and t[i] - t[first] >= duration:
            return float(t[first])
    return None


def weighted_stats(t, y, start, end):
    """Integrate a linearly interpolated signal, including exact boundary times."""
    if not t[0] <= start < end <= t[-1]:
        raise ValueError("Statistics interval lies outside telemetry")
    inner = (t > start) & (t < end)
    times = np.r_[start, t[inner], end]
    values = np.interp(times, t, y)
    dt = np.diff(times)
    mean = np.sum(dt * (values[:-1] + values[1:]) / 2) / (end - start)
    centered = values - mean
    variance = np.sum(dt * (centered[:-1] ** 2 + centered[:-1] * centered[1:]
                             + centered[1:] ** 2) / 3) / (end - start)
    return dict(mean=float(mean), sd=float(np.sqrt(max(0, variance))),
                minimum=float(values.min()), maximum=float(values.max()),
                peak_to_peak=float(np.ptp(values)))


def weighted_absolute_mean(t, y, start, end):
    inner = (t > start) & (t < end)
    times = np.r_[start, t[inner], end]
    values = np.interp(times, t, y)
    left, right = abs(values[:-1]), abs(values[1:])
    area = np.diff(times) * (left + right) / 2
    changes = values[:-1] * values[1:] < 0
    area[changes] = np.diff(times)[changes] * (
        left[changes] ** 2 + right[changes] ** 2) / (2 * (left[changes] + right[changes]))
    return float(area.sum() / (end - start))


def angle_level(origin, revolution, wrapped):
    return wrapped + 360 * math.ceil((origin + 360 * revolution - wrapped) / 360)


def circular_delta(a, b):
    return (np.asarray(a) - b + 180) % 360 - 180


def ramp_reference(t, start, stop, target, rate):
    if stop - start < target / rate:
        raise ValueError("Ramp reference requires the target to be reached before stopping")
    return np.clip((np.asarray(t) - start) * rate, 0, target) - np.clip((np.asarray(t) - stop) * rate, 0, target)


def command_events(events, t0_ns):
    records = []
    for event in events:
        if event["event"] != "motor_command_write_completed":
            continue
        match = re.search(r"command=([+-]?[\d.]+)", event["details"])
        if match:
            records.append(((int(event["motor_pc_monotonic_ns"]) - t0_ns) / 1e9, float(match[1])))
    start, target = next((t, value) for t, value in records if value > 0)
    stop = next(t for t, value in records if t > start and value == 0)
    return start, stop, target


def analyse_run(experiment, setup, args):
    recorded = experiment / setup["recorded_csv"]
    rows = read_csv(recorded)
    metadata_path = recorded.with_name(recorded.stem + "_metadata.json")
    metadata = json.loads(metadata_path.read_text())
    camera_path = experiment / setup["camera_metadata"]
    camera = json.loads(camera_path.read_text())
    events_path = recorded.with_name(recorded.stem + "_events.csv")
    event_rows = read_csv(events_path)
    t0_ns = int(metadata["motor_pc_t0_monotonic_ns"])
    t = np.array([(int(row["motor_pc_monotonic_ns"]) - t0_ns) / 1e9 for row in rows])
    if not np.all(np.diff(t) > 0):
        raise ValueError(f"Non-increasing telemetry timestamps: {recorded}")
    speed = np.array([float(row["recorded_motor_feedback_rpm"]) for row in rows])
    platform_speed = np.array([float(row["recorded_platform_feedback_rpm"]) for row in rows])
    angles = np.array([float(row["platform_angle_deg"]) for row in rows])
    commands = np.array([float(row["motor_speed_command_rpm"]) for row in rows])
    if not np.all(np.isfinite(np.r_[t, speed, platform_speed, angles, commands])):
        raise ValueError(f"Non-finite telemetry: {recorded}")
    if not np.allclose(platform_speed * 2, speed, atol=.002):
        raise ValueError("Recorded speed does not agree with the 2:1 transmission")
    command_start, command_stop, target = command_events(event_rows, t0_ns)
    low, high, selected_rev = CONDITIONS[experiment.name]
    start = crossing(t, angles, angles[0] + low * 360)
    end = crossing(t, angles, angles[0] + (high + 1) * 360)
    if start is None or end is None:
        raise ValueError(f"Common steady range is incomplete: {recorded}")
    stats = weighted_stats(t, platform_speed, start, end)
    steady_samples = platform_speed[(t >= start) & (t <= end)]
    if max(abs(stats["minimum"] - target / 2), abs(stats["maximum"] - target / 2)) > target / 2 * .05:
        raise ValueError(f"Previously selected interval no longer satisfies 5% screening: {recorded}")
    in_band = abs(speed - target) <= target * .05
    settled = sustained(t, in_band, .25, command_start, command_stop)
    stopped = sustained(t, abs(speed) < .5, .25, command_stop)
    rise10 = crossing(t, speed, target * .1, command_start)
    rise90 = crossing(t, speed, target * .9, command_start)
    fall90 = crossing(t, speed, target * .9, command_stop, False)
    fall10 = crossing(t, speed, target * .1, command_stop, False)
    initial_mask = t < command_start
    final_mask = t >= t[-1] - .25
    if not np.any(initial_mask) or np.any(abs(speed[initial_mask]) >= .5):
        raise ValueError(f"Initial stationary position unavailable: {recorded}")
    initial = float(angles[initial_mask].mean())
    final = float(angles[final_mask].mean()) if np.all(abs(speed[final_mask]) < .5) else None
    ramp_seconds = getattr(args, "ramp_seconds", 8.0)
    ramp_limit = getattr(args, "ramp_reference_rpm", 500.0)
    rate = ramp_limit / ramp_seconds
    predicted_full_ramp = target / rate
    compare_end = min(t[-1], max(command_stop + predicted_full_ramp, stopped or command_stop))
    compare_t = np.unique(np.r_[command_start, compare_end,
        t[(t > command_start) & (t < compare_end)],
        command_start + predicted_full_ramp, command_stop, command_stop + predicted_full_ramp])
    compare_t = compare_t[(compare_t >= command_start) & (compare_t <= compare_end)]
    expected = ramp_reference(compare_t, command_start, command_stop, target, rate) / 2
    residual = np.interp(compare_t, t, platform_speed) - expected
    residual_stats = weighted_stats(compare_t, residual, command_start, compare_end)
    # Exact integral of the absolute piecewise-linear error, including sign changes.
    left, right = abs(residual[:-1]), abs(residual[1:])
    sign_change = residual[:-1] * residual[1:] < 0
    abs_area = np.diff(compare_t) * (left + right) / 2
    abs_area[sign_change] = np.diff(compare_t)[sign_change] * (
        left[sign_change] ** 2 + right[sign_change] ** 2) / (2 * (left[sign_change] + right[sign_change]))
    ramp_mae = float(abs_area.sum() / (compare_end - command_start))
    dt = np.diff(t)
    steady_dt = dt[(t[:-1] >= start) & (t[1:] <= end)]
    quality = dict(condition=experiment.name, run=setup["run_label"], samples=len(rows),
                   median_sample_interval_ms=float(np.median(dt) * 1000),
                   maximum_sample_interval_ms=float(max(dt) * 1000),
                   maximum_steady_sample_interval_ms=float(max(steady_dt) * 1000),
                   intervals_over_3x_median=int(np.sum(dt > 3 * np.median(dt))),
                   telemetry_start_s=float(t[0]), telemetry_end_s=float(t[-1]),
                   legacy_elapsed_offset_mean_ms=float(np.mean(t - [float(r["elapsed_s"]) for r in rows]) * 1000),
                   recorded_raw_exists=(experiment / setup["raw_file"]).is_file(),
                   sync_mode=setup["sync_mode"], manual_evk4_motion_s=setup["manual_evk4_motion_s"],
                   studio_duration_s=setup["studio_duration_s"], camera_healthy=camera.get("camera_healthy_at_finalize"),
                   applied_biases=json.dumps(camera.get("applied_biases", {}), sort_keys=True))
    result = dict(condition=experiment.name, run=setup["run_label"], nominal_motor_rpm=target,
                  nominal_platform_rpm=target / 2, steady_first_rev=low, steady_last_rev=high,
                  steady_start_s=start, steady_end_s=end, steady_duration_s=end - start,
                  mean_platform_rpm=stats["mean"], error_measured_minus_command_rpm=stats["mean"] - target / 2,
                  error_percent=100 * (stats["mean"] - target / 2) / (target / 2),
                  steady_speed_mae_platform_rpm=weighted_absolute_mean(t, platform_speed-target/2, start, end),
                  steady_speed_rmse_platform_rpm=math.hypot(stats['mean']-target/2, stats['sd']),
                  within_run_sd_platform_rpm=float(np.std(steady_samples, ddof=1)),
                  interpolated_time_weighted_sd_platform_rpm=stats["sd"],
                  steady_samples=len(steady_samples), peak_to_peak_platform_rpm=stats["peak_to_peak"],
                  minimum_platform_rpm=stats["minimum"], maximum_platform_rpm=stats["maximum"],
                  command_write_completed_start_s=command_start, command_write_completed_stop_s=command_stop,
                  assumed_ramp_seconds_at_reference=ramp_seconds, assumed_ramp_reference_motor_rpm=ramp_limit,
                  estimated_full_ramp_s=predicted_full_ramp, estimated_10_90_ramp_s=.8 * predicted_full_ramp,
                  ramp_model_comparison_duration_s=compare_end - command_start,
                  ramp_model_mae_platform_rpm=ramp_mae,
                  ramp_model_rmse_platform_rpm=math.sqrt(residual_stats['mean'] ** 2 + residual_stats['sd'] ** 2),
                  rise_10_90_s=None if rise10 is None or rise90 is None else rise90 - rise10,
                  fall_90_10_s=None if fall90 is None or fall10 is None else fall10 - fall90,
                  command_to_settled_s=None if settled is None else settled - command_start,
                  zero_command_to_stopped_s=None if stopped is None else stopped - command_stop,
                  initial_wrapped_angle_deg=initial % 360,
                  initial_stationary_samples=int(initial_mask.sum()),
                  initial_stationary_span_s=float(np.ptp(t[initial_mask])),
                  final_wrapped_angle_deg=None if final is None else final % 360,
                  total_platform_travel_deg=None if final is None else final - initial)
    periods = []
    for rev in range(low, high + 1):
        a = crossing(t, angles, angles[0] + rev * 360)
        b = crossing(t, angles, angles[0] + (rev + 1) * 360)
        periods.append(dict(condition=experiment.name, run=setup["run_label"], revolution=rev,
                            period_s=b - a, encoder_travel_mean_platform_rpm=60 / (b - a)))
    sources = [recorded, metadata_path, events_path, camera_path, experiment / "experiment_setup.csv"]
    trace = dict(condition=experiment.name, run=setup["run_label"], t=t, speed=platform_speed,
                 angles=angles, displacement=angles - initial, commands=commands / 2,
                 steady_start=start, steady_end=end, rows=rows, setup=setup, experiment=experiment,
                 selected_rev=selected_rev, initial_angle=initial)
    trace.update(reference_t=compare_t, reference_speed=expected, reference_error=residual,
                 ramp_reference_seconds=ramp_seconds, ramp_reference_motor_rpm=ramp_limit)
    return result, quality, periods, trace, sources


def summarise(results):
    summaries = []
    for condition in CONDITIONS:
        runs = [row for row in results if row["condition"] == condition]
        first = runs[0]
        means = [row["mean_platform_rpm"] for row in runs]
        initial = np.array([row["initial_wrapped_angle_deg"] for row in runs])
        offsets = circular_delta(initial, initial[0])
        travels = [row["total_platform_travel_deg"] for row in runs]
        summaries.append(dict(condition=condition, independent_runs=len(runs),
            nominal_platform_rpm=first["nominal_platform_rpm"],
            mean_platform_rpm=statistics.mean(means), between_run_mean_speed_sd_rpm=statistics.stdev(means),
            mean_error_platform_rpm=statistics.mean(means) - first["nominal_platform_rpm"],
            mean_within_run_sd_rpm=statistics.mean(row["within_run_sd_platform_rpm"] for row in runs),
            start_orientation_range_deg=float(np.ptp(offsets)),
            start_orientation_sd_deg=float(np.std(offsets, ddof=1)),
            mean_total_travel_deg=statistics.mean(travels), total_travel_sd_deg=statistics.stdev(travels),
            mean_rise_10_90_s=statistics.mean(row["rise_10_90_s"] for row in runs),
            mean_fall_90_10_s=statistics.mean(row["fall_90_10_s"] for row in runs),
            mean_zero_command_to_stopped_s=statistics.mean(row["zero_command_to_stopped_s"] for row in runs)))
    return summaries


def angular_edges(start, end):
    if not (math.isfinite(start) and math.isfinite(end) and 0 <= start < 360
            and start < end <= start + 360):
        raise ValueError("Event interval needs a wrapped start in [0,360) and extent in (0,360]")
    return np.r_[np.arange(start, end, 5.0), end]


def event_interval(trace, start_angle=4.0, end_angle=364.0):
    """Locate one continuous arc, including a crossing of wrapped zero."""
    angular_edges(start_angle, end_angle)
    setup = trace["setup"]
    rows = trace["rows"]
    feedback = np.array([float(row["recorded_motor_feedback_rpm"]) for row in rows])
    motion = next(i for i in range(len(rows) - 2) if np.all(abs(feedback[i:i + 3]) >= .5))
    latte = np.array([float(row["lattepanda_time_s"]) for row in rows])
    camera_times = trace.get('accepted_camera_times')
    if camera_times is None:
        camera_times = float(setup["manual_evk4_motion_s"]) + latte - latte[motion]
    angles = trace["angles"]
    level = angle_level(angles[0], trace["selected_rev"], start_angle)
    first = crossing(camera_times, angles, level)
    last = crossing(camera_times, angles, level + end_angle - start_angle)
    if first is None or last is None or first < 0 or last > float(setup["studio_duration_s"]):
        raise ValueError("Selected sector lies outside the camera/encoder overlap")
    if 'commands' in trace:
        target = float(np.max(trace['commands'])) * 2
        stats = weighted_stats(camera_times, feedback, first, last)
        if max(abs(stats['minimum'] - target), abs(stats['maximum'] - target)) > .05 * target:
            raise ValueError("Selected full event interval is not entirely steady within 5%")
        mask = (camera_times >= first) & (camera_times <= last)
        if np.any(abs(trace['commands'][mask] * 2 - target) > .01 * target):
            raise ValueError("Speed command changes within the selected event interval")
    return first, last, level, camera_times


def event_statistics(trace, start_angle=4.0, end_angle=364.0, artifact_root=None):
    """Count a continuous arc; default is one complete 360-degree steady rotation."""
    from rig_vpr.algorithm import EventSource
    setup = trace["setup"]
    path = trace["experiment"] / setup["raw_file"]
    angles = trace["angles"]
    first, last, level, camera_times = event_interval(trace, start_angle, end_angle)
    edges = angular_edges(start_angle, end_angle)
    source = EventSource(path, float(setup['studio_duration_s']))
    origin = source.origin_us
    t0, t1 = round(first * 1e6), round(last * 1e6)
    full_windows = (t1 - t0) // 2000
    if full_windows < 1:
        source.close()
        raise ValueError('Event interval must contain at least one complete 2 ms window')
    counts = np.zeros(full_windows, dtype=np.int64)
    active = np.zeros(full_windows, dtype=np.int64)
    width, height = source.sensor_size
    spatial_on = np.zeros(width * height, dtype=np.uint64)
    spatial_off = np.zeros(width * height, dtype=np.uint64)
    centred_counts = np.zeros(12, dtype=np.int64)
    total = on = 0
    sectors = np.zeros(len(edges) - 1, dtype=np.int64)
    try:
        for start in range(t0, t1, 20_000):
            events = source.events_between(start, min(start + 20_000, t1))
            total += len(events)
            if not len(events):
                continue
            if (not np.all(np.isfinite(events)) or np.any(events[:, :2] != np.floor(events[:, :2]))
                    or np.any(events[:, 0] < 0) or np.any(events[:, 0] >= width)
                    or np.any(events[:, 1] < 0) or np.any(events[:, 1] >= height)
                    or not np.all(np.isin(events[:, 3], (0, 1)))):
                raise ValueError('Invalid event coordinates or polarity')
            is_on = events[:, 3] > 0
            on += int(np.sum(is_on))
            pixels = events[:, 1].astype(np.int64) * width + events[:, 0].astype(np.int64)
            spatial_on += np.bincount(pixels[is_on], minlength=width*height).astype(np.uint64)
            spatial_off += np.bincount(pixels[~is_on], minlength=width*height).astype(np.uint64)
            ids = ((events[:, 2].astype(np.int64) - origin - t0) // 2000)
            valid = (ids >= 0) & (ids < full_windows)
            counts += np.bincount(ids[valid], minlength=full_windows)
            keys = np.unique(ids[valid] * width * height + pixels[valid])
            active += np.bincount(keys // (width * height), minlength=full_windows)
            event_time = (events[:, 2] - origin) / 1e6
            positions = np.interp(event_time, camera_times, angles) - level + start_angle
            if positions.min() < start_angle - .01 or positions.max() > end_angle + .01:
                raise ValueError("Event-angle interpolation falls outside the selected arc")
            positions = np.clip(positions, start_angle, np.nextafter(end_angle, -np.inf))
            sectors += np.histogram(positions, bins=edges)[0]
            centred_ids = (((positions + 15) % 360) // 30).astype(np.int64)
            centred_counts += np.bincount(centred_ids, minlength=12)
            if (start-t0) % 1_000_000 == 0:
                print(f"[RAW] {trace['condition']} {trace['run']}: {(start-t0)/1e6:.1f}/{(t1-t0)/1e6:.2f} s", flush=True)
    finally:
        source.close()
    duration = (t1 - t0) / 1e6
    if not total:
        raise ValueError('No CD events in the selected continuous interval; check recording and alignment')
    spatial = spatial_on + spatial_off
    if int(spatial.sum()) != total or int(sectors.sum()) != total or int(centred_counts.sum()) != total:
        raise ValueError('Spatial and angular event totals do not reconcile')
    zero_edges = np.flatnonzero(np.diff(np.r_[False, counts == 0, False].astype(np.int8)))
    longest_empty = int(max(zero_edges[1::2] - zero_edges[::2], default=0))
    result = dict(condition=trace["condition"], run=trace["run"], status="processed",
                  revolution=trace["selected_rev"], sector_start_deg=start_angle, sector_end_deg=end_angle,
                  angular_extent_deg=end_angle - start_angle,
                  wrapped_start_angle_deg=start_angle, wrapped_end_angle_deg=end_angle % 360,
                  camera_start_s=first, camera_end_s=last, duration_s=duration, cd_events=total,
                  events_per_second=total / duration, events_per_degree=total / (end_angle - start_angle),
                  manual_evk4_motion_s_used=float(setup['manual_evk4_motion_s']),
                  studio_duration_s_used=float(setup['studio_duration_s']),
                  on_fraction=on / total if total else None, complete_2ms_windows=int(full_windows),
                  mean_events_per_2ms=float(np.mean(counts)), p95_events_per_2ms=float(np.percentile(counts, 95)),
                  empty_2ms_fraction=float(np.mean(counts == 0)),
                  mean_active_pixel_fraction_2ms=float(np.mean(active) / (width * height)),
                  sensor_width=width, sensor_height=height, off_fraction=(total-on)/total if total else None,
                  union_active_pixel_fraction=float(np.count_nonzero(spatial)/(width*height)),
                  top_1percent_pixel_event_fraction=float(np.sort(spatial)[-math.ceil(width*height*.01):].sum()/total) if total else None,
                  longest_empty_2ms_sequence=longest_empty,
                  longest_empty_2ms_duration_ms=longest_empty*2,
                  complete_window_events=int(counts.sum()), incomplete_tail_events=total-int(counts.sum()),
                  raw_origin_us=origin)
    centred = []
    for i, count in enumerate(centred_counts):
        lower = -15 + i*30
        seconds = extent = 0.
        for turn in (-1, 0, 1, 2):
            a, b = max(start_angle, lower+360*turn), min(end_angle, lower+30+360*turn)
            if a < b:
                seconds += crossing(camera_times, angles, level+b-start_angle) - crossing(camera_times, angles, level+a-start_angle)
                extent += b-a
        if extent > 0:
            centred.append(dict(condition=trace['condition'], run=trace['run'],
                angle_start_deg=lower, angle_end_deg=lower+30, cd_events=int(count),
                measured_angular_extent_deg=extent, duration_s=seconds,
                events_per_degree=float(count/extent), events_per_second=float(count/seconds)))
    if artifact_root is not None:
        artifact_root = Path(artifact_root)
        arrays = artifact_root/'event_arrays'
        arrays.mkdir(parents=True, exist_ok=True)
        name = f"{trace['condition']}_{trace['run']}.npz"
        np.savez_compressed(arrays/name, on_counts=spatial_on.reshape(height, width),
            off_counts=spatial_off.reshape(height, width), window_event_counts=counts,
            window_active_pixels=active, camera_start_us=t0, camera_end_us=t1,
            window_start_us=t0+np.arange(full_windows)*2000, accumulation_us=2000,
            duration_s=duration, centred_30deg_counts=centred_counts)
        result['event_arrays_file'] = f'event_arrays/{name}'
        write_csv(arrays/f"{trace['condition']}_{trace['run']}_sectors_30deg.csv", centred)
    detail = [dict(condition=trace["condition"], run=trace["run"],
                   angle_start_deg=float(edges[i]), angle_end_deg=float(edges[i + 1]),
                   cd_events=int(count), events_per_degree=float(count / (edges[i + 1] - edges[i])))
              for i, count in enumerate(sectors)]
    return result, detail


def import_event_results(folder, traces, start_angle=4.0, end_angle=364.0):
    """Validate and reuse Linux RAW outputs without requiring RAW files locally."""
    folder = folder.expanduser().resolve()
    results = read_csv(folder / "event_statistics.csv")
    sectors = read_csv(folder / "event_sector_counts.csv")
    quality = read_csv(folder / "data_quality.csv")
    expected = {(trace['condition'], trace['run']): trace for trace in traces}
    if len(results) != len(expected) or {(r['condition'], r['run']) for r in results} != set(expected):
        raise ValueError("Imported event results must contain exactly the selected nine acquisitions")
    text_fields = {'condition', 'run', 'raw_file', 'status', 'source_mode', 'event_arrays_file'}
    integer_fields = {'revolution', 'cd_events', 'complete_2ms_windows'}
    parsed = []
    for source in results:
        key = source['condition'], source['run']
        trace = expected[key]
        if source['status'] != 'processed':
            raise ValueError(f"Imported RAW was not processed: {key}")
        if Path(source['raw_file']).name != Path(trace['setup']['raw_file']).name:
            raise ValueError(f"Imported RAW identity differs: {key}")
        row = {k: (v if k in text_fields else int(v) if k in integer_fields else float(v))
               for k, v in source.items()}
        if (row['revolution'], row['sector_start_deg'], row['sector_end_deg']) != (trace['selected_rev'], start_angle, end_angle):
            raise ValueError(f"Imported angular selection differs for {key}: expected {start_angle:g}–{end_angle:g} deg. Reprocess the RAW files for the current scope.")
        if not math.isclose(row['events_per_second'], row['cd_events'] / row['duration_s'], rel_tol=1e-8):
            raise ValueError(f"Imported event rate does not reconcile: {key}")
        if not math.isclose(row['events_per_degree'], row['cd_events'] / (end_angle - start_angle), rel_tol=1e-8):
            raise ValueError(f"Imported angular density does not reconcile: {key}")
        q = next(r for r in quality if (r['condition'], r['run']) == key)
        row['manual_evk4_motion_s_used'] = float(q['manual_evk4_motion_s'])
        row['studio_duration_s_used'] = float(q['studio_duration_s'])
        row['local_manual_evk4_motion_s'] = float(trace['setup']['manual_evk4_motion_s'])
        row['source_mode'] = 'imported Linux RAW output'
        selected_sectors = [r for r in sectors if (r['condition'], r['run']) == key]
        expected_bins = len(angular_edges(start_angle, end_angle)) - 1
        if len(selected_sectors) != expected_bins:
            raise ValueError(f"Expected {expected_bins} angular bins for {key}")
        row['events_not_assigned_to_angular_bins'] = row['cd_events'] - sum(int(r['cd_events']) for r in selected_sectors)
        parsed.append(row)
    return parsed, [{k: (v if k in {'condition', 'run'} else int(v) if k == 'cd_events' else float(v))
                     for k, v in source.items()} for source in sectors]


def event_summary(rows):
    metrics = ['events_per_second', 'events_per_degree', 'mean_events_per_2ms',
               'mean_active_pixel_fraction_2ms', 'on_fraction', 'empty_2ms_fraction']
    summary = []
    for condition in CONDITIONS:
        group = [r for r in rows if r['condition'] == condition and r['status'] == 'processed']
        if not group:
            continue
        item = dict(condition=condition, independent_runs=len(group))
        for field in metrics:
            values = [float(r[field]) for r in group]
            item[field + '_mean'] = statistics.mean(values)
            item[field + '_between_run_sd'] = statistics.stdev(values) if len(values) > 1 else None
        summary.append(item)
    return summary


def apply_vpr_results(folder, root, traces, qualities):
    """Bind continuous-event analysis to the accepted VPR mapping and hashes."""
    from rig_vpr.data import check_inputs, load_trace, validate_protocol, sha256
    folder = Path(folder).expanduser().resolve()
    snapshot = folder/'protocol_snapshot.json'
    protocol = json.loads(snapshot.read_text(encoding='utf-8'))
    validate_protocol(protocol)
    completed = json.loads((folder/'run_complete.json').read_text(encoding='utf-8'))
    provenance = json.loads((folder/'provenance.json').read_text(encoding='utf-8'))
    digest = sha256(snapshot)
    if completed['protocol_sha256'] != digest or provenance['protocol_sha256'] != digest:
        raise ValueError('VPR completion/provenance does not match its frozen protocol')
    if protocol.get('discovery_warnings'):
        raise ValueError('Resolve VPR alignment warnings before rig evaluation')
    expected = {f'{condition}/run{i}' for condition in CONDITIONS for i in (1,2,3)}
    entries = {entry['key']: entry for entry in protocol['acquisitions']}
    if set(entries) != expected or len(protocol['acquisitions']) != 9:
        raise ValueError('VPR source must contain exactly the nine Low/Medium/High acquisitions')
    plans, audit = check_inputs(protocol, root, require_raw=False)
    windows = read_csv(folder/'frame_windows.csv')
    for trace, quality in zip(traces, qualities):
        key = f"{trace['condition']}/{trace['run']}"
        entry = entries[key]
        accepted = load_trace(root, entry)
        if not np.allclose(trace['angles'], accepted['angles'], rtol=0, atol=1e-8):
            raise ValueError(f'{key}: rig and VPR encoder samples differ')
        selected = sorted((r for r in windows if r['acquisition']==key), key=lambda r:int(r['frame_index']))
        centres = np.array([round(float(r['center_s'])*1e6) for r in selected])
        if not np.array_equal(centres, plans[key]['center_us']):
            raise ValueError(f'{key}: frozen VPR frame centres do not match accepted mapping')
        trace['accepted_camera_times'] = accepted['times']
        trace['selected_rev'] = entry['revolution']
        trace['setup'] = dict(trace['setup'], manual_evk4_motion_s=str(entry['manual_evk4_motion_s']),
                              studio_duration_s=str(entry['studio_duration_s']))
        trace['expected_raw'] = provenance['inputs'][key]['raw']
        quality.update(manual_evk4_motion_s=entry['manual_evk4_motion_s'],
                       studio_duration_s=entry['studio_duration_s'], alignment_source='frozen VPR mapping',
                       vpr_protocol_sha256=digest)
    sources = [snapshot, folder/'provenance.json', folder/'run_complete.json', folder/'frame_windows.csv']
    for row in audit:
        sources.extend(root/value['path'] for value in row['sources'].values())
    return sources, dict(source_results=str(folder), protocol_sha256=digest,
                         accepted_onsets={k:e['manual_evk4_motion_s'] for k,e in entries.items()})


def trajectory_repeatability(traces):
    """Pairwise displacement differences at common scheduled-T0 times."""
    pairs = []
    for condition in CONDITIONS:
        group = [t for t in traces if t['condition']==condition]
        for i, first in enumerate(group):
            for second in group[i+1:]:
                start = max(first['steady_start'], second['steady_start'])
                end = min(first['steady_end'], second['steady_end'])
                times = np.unique(np.r_[start, end,
                    first['t'][(first['t']>start)&(first['t']<end)],
                    second['t'][(second['t']>start)&(second['t']<end)]])
                delta = np.interp(times, second['t'], second['displacement']) - np.interp(times, first['t'], first['displacement'])
                stats = weighted_stats(times, delta, start, end)
                pairs.append(dict(condition=condition, reference_run=first['run'], query_run=second['run'],
                    comparison_start_s=start, comparison_end_s=end, duration_s=end-start,
                    mean_signed_displacement_difference_deg=stats['mean'],
                    displacement_mae_deg=weighted_absolute_mean(times, delta, start, end),
                    displacement_rmse_deg=math.hypot(stats['mean'], stats['sd']),
                    maximum_absolute_displacement_difference_deg=float(np.max(abs(delta))),
                    initial_orientation_difference_deg=float(circular_delta(second['initial_angle'], first['initial_angle']))))
    return pairs


def apply_alignment_results(folder, traces, qualities):
    """Use accepted alignment values from returned processing without editing inputs."""
    quality_path = folder / 'data_quality.csv'
    imported = read_csv(quality_path)
    for trace, quality in zip(traces, qualities):
        matches = [row for row in imported if (row['condition'], row['run']) == (trace['condition'], trace['run'])]
        if len(matches) != 1:
            raise ValueError("Alignment results need one data-quality row per acquisition")
        source = matches[0]
        trace['setup'] = dict(trace['setup'])
        for key in ('manual_evk4_motion_s', 'studio_duration_s'):
            quality['local_' + key] = quality[key]
            value = float(source[key])
            if not math.isfinite(value) or value < 0 or (key == 'studio_duration_s' and value == 0):
                raise ValueError(f"Invalid imported alignment {key}")
            trace['setup'][key] = str(value)
            quality[key] = str(value)
        quality['alignment_source'] = str(quality_path.resolve())
    return quality_path


def figures(traces, results, summaries, event_rows, output, sector_rows=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    colours = ["#0072B2", "#D55E00", "#009E73"]
    styles = ['-', '--', ':']

    def save(fig, name):
        fig.savefig(output / (name + ".png"), dpi=190)
        fig.savefig(output / (name + ".svg"))
        fig.savefig(output / (name + ".pdf"))
        plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(13, 7), layout="constrained")
    for column, condition in enumerate(CONDITIONS):
        selected = [trace for trace in traces if trace["condition"] == condition]
        nominal = next(row["nominal_platform_rpm"] for row in summaries if row["condition"] == condition)
        for i, trace in enumerate(selected):
            t, v = trace["t"], trace["speed"]
            axes[0, column].plot(t, v, color=colours[i], ls=styles[i], label=trace["run"], lw=1.1)
            mask = (t >= trace["steady_start"]) & (t <= trace["steady_end"])
            axes[1, column].plot(t[mask], v[mask] - nominal, color=colours[i], ls=styles[i], lw=1)
        axes[0, column].step(selected[0]["t"], selected[0]["commands"], where="post", color="black", ls="--", label="Requested speed (run1)", lw=1)
        axes[0, column].set(title=f"{condition}: {nominal:g} platform rpm", xlabel="Time from scheduled T0 (s)", ylabel="Platform speed (rpm)")
        axes[1, column].axhline(0, color="black", ls=":", lw=1)
        axes[1, column].set(xlabel="Time from scheduled T0 (s)", ylabel="Steady speed error (rpm)")
        for ax in axes[:, column]:
            ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    save(fig, "speed_tracking")

    fig, axes = plt.subplots(2, 3, figsize=(13, 7), layout="constrained")
    for column, condition in enumerate(CONDITIONS):
        selected = [trace for trace in traces if trace["condition"] == condition]
        for i, trace in enumerate(selected):
            axes[0, column].plot(trace["t"], trace["speed"], color=colours[i], ls=styles[i], label=trace["run"])
            axes[1, column].plot(trace["reference_t"], trace["reference_error"], color=colours[i], ls=styles[i])
        trace = selected[0]
        axes[0, column].plot(trace["reference_t"], trace["reference_speed"], color="black", ls="--", label="Linear ramp estimate (run1)")
        axes[0, column].set(title=condition, xlabel="Time from scheduled T0 (s)", ylabel="Platform speed (rpm)")
        axes[1, column].axhline(0, color="black", ls=":", lw=1)
        axes[1, column].set(xlabel="Time from scheduled T0 (s)", ylabel="Recorded minus estimated speed (rpm)")
        for ax in axes[:, column]:
            ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(f"Ramp estimate: {traces[0]['ramp_reference_seconds']:g} s at {traces[0]['ramp_reference_motor_rpm']:g} motor rpm")
    save(fig, "ramp_reference_comparison")

    fig, axes = plt.subplots(2, 3, figsize=(13, 7), layout="constrained")
    for column, condition in enumerate(CONDITIONS):
        selected = [trace for trace in traces if trace["condition"] == condition]
        base = selected[0]
        for i, trace in enumerate(selected):
            axes[0, column].plot(trace["t"], trace["displacement"], color=colours[i], ls=styles[i], label=trace["run"])
            mask = (trace["t"] >= base["t"][0]) & (trace["t"] <= base["t"][-1])
            values = trace["displacement"][mask] - np.interp(trace["t"][mask], base["t"], base["displacement"])
            axes[1, column].plot(trace["t"][mask], values, color=colours[i], ls=styles[i])
        axes[0, column].set(title=condition, xlabel="Time from scheduled T0 (s)", ylabel="Displacement from initial rest (deg)")
        axes[1, column].set(xlabel="Time from scheduled T0 (s)", ylabel="Displacement difference to run1 (deg)")
        for ax in axes[:, column]:
            ax.grid(alpha=.2)
    axes[0, 0].legend()
    save(fig, "position_repeatability")

    fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
    metrics = [("error_measured_minus_command_rpm", "Steady mean error (platform rpm)"),
               ("within_run_sd_platform_rpm", "Within-run speed SD (platform rpm)"),
               ("zero_command_to_stopped_s", "Zero command to stopped (s)")]
    for ax, (key, label) in zip(axes, metrics):
        for i, row in enumerate(results):
            ax.scatter(row["nominal_platform_rpm"], row[key], color=colours[i % 3], s=35,
                       label=row["run"] if i < 3 else None)
        ax.set(xlabel="Nominal platform speed (rpm)", ylabel=label)
        ax.grid(alpha=.2)
    axes[0].legend()
    save(fig, "motion_summary")

    processed = [row for row in event_rows if row["status"] == "processed"]
    if processed:
        fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
        keys = [("events_per_second", "CD event rate (million events/s)", 1e-6),
                ("events_per_degree", "CD event density (thousand events/degree)", 1e-3),
                ("mean_active_pixel_fraction_2ms", "Mean active pixels per 2 ms (%)", 100)]
        for ax, (key, label, scale) in zip(axes, keys):
            for row in processed:
                motion = next(m for m in results if (m["condition"], m["run"]) == (row["condition"], row["run"]))
                ax.scatter(motion["mean_platform_rpm"], row[key]*scale, color=colours[int(row["run"][-1]) - 1],
                           label=row['run'] if row['condition'] == 'Low-Speed' else None)
            for condition in CONDITIONS:
                group = [row for row in processed if row['condition'] == condition]
                x = statistics.mean(row['mean_platform_rpm'] for row in results if row['condition'] == condition)
                values = [row[key]*scale for row in group]
                ax.errorbar(x, statistics.mean(values), yerr=statistics.stdev(values),
                            fmt='_', color='black', capsize=4, lw=1, zorder=1)
            ax.set(xlabel="Measured platform speed (rpm)", ylabel=label)
            ax.grid(alpha=.2)
        axes[0].legend()
        extent = processed[0]['sector_end_deg'] - processed[0]['sector_start_deg']
        fig.suptitle(f"Event characteristics over {extent:g} degrees; three independent runs per speed")
        save(fig, "event_characteristics")

        fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
        keys = [('mean_events_per_2ms', 'Mean CD events per 2 ms (thousands)', 1e-3),
                ('on_fraction', 'ON events (%)', 100), ('empty_2ms_fraction', 'Empty complete 2 ms windows (%)', 100)]
        for ax, (key, label, scale) in zip(axes, keys):
            for row in processed:
                motion = next(m for m in results if (m['condition'],m['run']) == (row['condition'],row['run']))
                ax.scatter(motion['mean_platform_rpm'],row[key]*scale,color=colours[int(row['run'][-1])-1],
                           label=row['run'] if row['condition'] == 'Low-Speed' else None)
            ax.set(xlabel='Measured platform speed (rpm)',ylabel=label)
            ax.grid(alpha=.2)
        axes[0].legend()
        fig.suptitle(f"Temporal windows and polarity over {extent:g} degrees")
        save(fig, 'event_windows_and_polarity')

        if sector_rows:
            fig, axes = plt.subplots(2, 3, figsize=(13, 7), layout='constrained', sharex=True)
            start = processed[0]['sector_start_deg']
            for column, condition in enumerate(CONDITIONS):
                for i, run in enumerate(('run1','run2','run3')):
                    selected = sorted((row for row in sector_rows if (row['condition'],row['run']) == (condition,run)),
                                      key=lambda row: row['angle_start_deg'])
                    total = next(row['cd_events'] for row in processed if (row['condition'],row['run']) == (condition,run))
                    centres = [(row['angle_start_deg']+row['angle_end_deg'])/2-start for row in selected]
                    axes[0,column].plot(centres,[row['events_per_degree']/1000 for row in selected],
                                        color=colours[i],label=run,lw=1.2)
                    axes[1,column].plot(centres,[row['cd_events']/total*100 for row in selected],
                                        color=colours[i],lw=1.2)
                axes[0,column].set(title=condition,ylabel='Thousand CD events/degree')
                axes[1,column].set(xlabel=f'Angle travelled from wrapped {start:g}° start (deg)',
                                  ylabel='Share of full-turn events per 5° bin (%)')
                for ax in axes[:,column]:
                    ax.set_xlim(0,extent)
                    ax.set_xticks(np.arange(0,extent+1,90))
                    ax.grid(alpha=.2)
            axes[0,0].legend()
            fig.suptitle('Event activity across the full rotation; each column uses its own vertical scale')
            save(fig,'event_density_by_angle')


def spatial_figures(output, event_rows):
    """Native-pixel spatial rates, disjoint temporal activity and centred sectors."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm
    colours = ['#0072B2', '#D55E00', '#009E73']
    styles = ['-', '--', ':']
    loaded = {}
    maximum = 1.
    for row in event_rows:
        if row['status'] != 'processed' or not row.get('event_arrays_file'):
            continue
        with np.load(output/row['event_arrays_file'],allow_pickle=False) as saved:
            data = {k:saved[k].copy() for k in saved.files}
        loaded[row['condition'],row['run']] = data
        maximum = max(maximum, float((data['on_counts']+data['off_counts']).max()/float(data['duration_s'])))
    if len(loaded) != 9:
        return
    fig, axes = plt.subplots(3,3,figsize=(12,8),layout='constrained')
    for i, condition in enumerate(CONDITIONS):
        for j in range(3):
            data = loaded[condition,f'run{j+1}']
            rate = (data['on_counts']+data['off_counts'])/float(data['duration_s'])
            image = axes[i,j].imshow(np.ma.masked_equal(rate,0),cmap='viridis',norm=LogNorm(vmin=1,vmax=max(2,maximum)))
            axes[i,j].set(title=f'{condition}, run {j+1}',xlabel='Sensor x (pixels)',ylabel='Sensor y (pixels)')
    fig.colorbar(image,ax=axes,label='Recorded events per pixel per second (log scale)',shrink=.8)
    fig.suptitle('Spatial event distribution over one continuous full revolution')
    for extension in ('pdf','png','svg'):
        fig.savefig(output/'figures'/f'event_spatial_distribution.{extension}',dpi=190)
    plt.close(fig)
    fig, axes = plt.subplots(3,3,figsize=(12,8),layout='constrained')
    for i, condition in enumerate(CONDITIONS):
        for j in range(3):
            data = loaded[condition,f'run{j+1}']
            times = (data['window_start_us']-int(data['camera_start_us'])+1000)/1e6
            axes[i,j].plot(times,data['window_event_counts']/.002/1e6,color=colours[j],lw=.7)
            axes[i,j].set(title=f'{condition}, run {j+1}',xlabel='Time into selected revolution (s)',ylabel='Recorded rate (million events/s)')
            axes[i,j].grid(alpha=.2)
    fig.suptitle('Disjoint complete 2 ms windows; final incomplete window omitted')
    for extension in ('pdf','png','svg'):
        fig.savefig(output/'figures'/f'event_temporal_activity.{extension}',dpi=190)
    plt.close(fig)
    rows = read_csv(output/'event_sector_counts_30deg_centred.csv')
    fig, axes = plt.subplots(2,3,figsize=(13,7),layout='constrained')
    for column, condition in enumerate(CONDITIONS):
        for i in range(3):
            selected = sorted((r for r in rows if r['condition']==condition and r['run']==f'run{i+1}'),key=lambda r:float(r['angle_start_deg']))
            centres = [float(r['angle_start_deg'])+15 for r in selected]
            for ax,field,scale,label in ((axes[0,column],'events_per_degree',1e-3,'Thousand events per degree'),
                                        (axes[1,column],'events_per_second',1e-6,'Million events per second')):
                ax.plot(centres,[float(r[field])*scale for r in selected],color=colours[i],ls=styles[i],marker='o',ms=3,label=f'Run {i+1}')
                ax.set(title=condition,xlabel='Sector centre (degrees)',ylabel=label)
                ax.set_xticks(range(0,360,60));ax.grid(alpha=.2)
    axes[0,0].legend()
    fig.suptitle('Continuous recorded event activity in centred 30-degree sectors')
    for extension in ('pdf','png','svg'):
        fig.savefig(output/'figures'/f'event_activity_30deg_centred.{extension}',dpi=190)
    plt.close(fig)


def event_vpr_comparison(vpr_results, centred_rows):
    sector_root = Path(vpr_results).parent/'sectors_30deg_centred'
    stamp = json.loads((sector_root/'sector_analysis_complete.json').read_text(encoding='utf-8'))
    if stamp['sector_deg'] != 30 or stamp['sector_start_deg'] != -15:
        raise ValueError('VPR sectors must be centred 30-degree sectors')
    sector_provenance = json.loads((sector_root/'sector_provenance.json').read_text(encoding='utf-8'))
    if sector_provenance['source_protocol'] != json.loads((Path(vpr_results)/'protocol_snapshot.json').read_text(encoding='utf-8')):
        raise ValueError('VPR sector protocol differs from accepted motion alignment')
    continuous = {(r['condition'],r['run'],float(r['angle_start_deg'])):r for r in centred_rows}
    result = []
    for row in read_csv(sector_root/'sectors_per_pair.csv'):
        condition, run = row['query'].split('/')
        events = continuous[condition,run,float(row['sector_start_deg'])]
        item = dict(row)
        item.update({f'continuous_{key}':events[key] for key in ('cd_events','duration_s','events_per_degree','events_per_second')})
        result.append(item)
    return result


def report(output, summaries, qualities, event_rows, results, start_angle, end_angle):
    lines = ["# Rig evaluation of Low Medium and High speed", "",
             "The analysis includes Low, Medium and High, with three independent acquisitions per speed (nine total). Event processing status and the measured angular scope are reported below.", "",
             "## Speed and response results", "",
             "| Condition | Nominal platform rpm | Measured mean rpm | Between-run SD rpm | Mean within-run SD rpm | Rise 10–90 s | Fall 90–10 s | Zero-command to stopped s |",
             "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for row in summaries:
        keys = ["nominal_platform_rpm", "mean_platform_rpm", "between_run_mean_speed_sd_rpm",
                "mean_within_run_sd_rpm", "mean_rise_10_90_s", "mean_fall_90_10_s", "mean_zero_command_to_stopped_s"]
        lines.append("| " + row["condition"] + " | " + " | ".join(f"{row[key]:.5f}" for key in keys) + " |")
    lines += ["", "## Estimated ramp versus recorded response", "",
              "This comparison uses the operator-supplied ramp assumption, rather than historical per-run drive-register readbacks. Estimated speed follows a linear ramp triggered by completed software command-write timestamps; the actual drive can begin responding during the write transaction. Therefore residuals include command-timing and sampling effects as well as dynamic differences.", "",
              "| Condition | Expected full ramp s | Expected 10–90 s | Recorded rise 10–90 s | Recorded fall 90–10 s | Mean active-profile MAE platform rpm | Mean active-profile RMSE platform rpm |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for condition in CONDITIONS:
        runs = [row for row in results if row['condition'] == condition]
        keys = ['estimated_full_ramp_s', 'estimated_10_90_ramp_s', 'rise_10_90_s', 'fall_90_10_s',
                'ramp_model_mae_platform_rpm', 'ramp_model_rmse_platform_rpm']
        lines.append('| ' + condition + ' | ' + ' | '.join(f"{statistics.mean(row[key] for row in runs):.5f}" for key in keys) + ' |')
    lines += ["", "The active-profile comparison spans the first nonzero completed write to the later of model stop and confirmed observed stop. Motor-rpm errors are twice the reported platform-rpm errors. The ramp estimate is distinct from the un-ramped requested speed step.",
              "", "## Position repeatability", "",
              "| Condition | Starting orientation range deg | Starting orientation SD deg | Mean total travel deg | Between-run travel SD deg |",
              "|---|---:|---:|---:|---:|"]
    for row in summaries:
        keys = ["start_orientation_range_deg", "start_orientation_sd_deg", "mean_total_travel_deg", "total_travel_sd_deg"]
        lines.append("| " + row["condition"] + " | " + " | ".join(f"{row[key]:.5f}" for key in keys) + " |")
    biases = sorted({row["applied_biases"] for row in qualities})
    lines += ["", "## Definitions and interpretation", "",
        "- Time is the recorded motor-PC monotonic timestamp minus the scheduled motor-PC T0 from run metadata. This matches the event-log clock domain. Legacy elapsed_s is not mixed with this reference.",
        "- Platform speed equals motor feedback divided by the recorded 2:1 transmission ratio. Speed error is measured minus commanded platform speed.",
        "- The previously identified common complete steady revolutions are Low 1, Medium 1–6, High 6–17 (inclusive and zero-based). They were selected using the observed data, not a prospective engineering pass/fail requirement.",
        "- Steady means integrate linearly interpolated speed over exact angle-crossing boundaries, weighting by elapsed time. The primary within-run SD is the sample SD (ddof=1) of actual telemetry rows inside the interval, preserving observed fluctuations rather than smoothing them through interpolation. The supplementary interpolated_time_weighted_sd field integrates a linearly interpolated signal. Between-run SD uses the sample SD of the three run means (n=3).",
        "- Rise and fall times use first interpolated 10%/90% crossings after the relevant completed command write. A command-to-settled diagnostic uses a continuous observed ±5% interval lasting at least 0.25 s. Stopped means absolute motor feedback below 0.5 rpm for at least 0.25 s. The reported stop instant is the beginning of that confirmed interval.",
        "- Command-write completion is a software timestamp, not a measured physical command edge. Rise/fall threshold timings do not include command latency. Threshold-crossing precision is limited by telemetry sampling.",
        "- Initial position is the mean stationary encoder-derived platform angle before the first completed nonzero command. Final position is the last 0.25 s stationary mean. Starting-position scatter preserves original wrapped references and uses circular differences.",
        "- Position traces subtract initial rest only to compare displacement. The separate original starting-orientation statistics prevent that subtraction from hiding initial differences. These speed profiles have no commanded position target; no position-target accuracy or direct platform calibration is claimed.",
        "- Full-turn periods, angle travel and speed all derive from the same motor encoder. Agreement is an internal diagnostic, not an independent check for belt slip.",
        "- Manual alignment is accepted as supplied. The prepared catalogues may carry an assumed 100 ms uncertainty; that field is not a measured alignment error and is not replaced by visual acceptance or by 2 ms accumulation.",
        f"- Event counting uses all continuous CD events from unwrapped comparison angle {start_angle:g} to {end_angle:g} degrees ({end_angle-start_angle:g} degrees of travel), beginning in Low rev1, Medium rev3 and High rev11. The default is a complete turn from wrapped 4 degrees, through zero, back to wrapped 4 degrees. All nine recordings start just below wrapped 4 degrees, so this shared phase avoids taking the 0-degree crossing from the end of an indexed revolution. Each full arc is checked against the 5% steady-speed rule and a constant command before counting.",
        "- Full-interval events/s and events/degree include the entire half-open interval. Two-ms window statistics exclude the final incomplete window. ON and OFF are counted separately; external triggers are not CD events. Integer-microsecond boundary rounding is reconciled to edge angular bins without dropping interval events.",
        "- Event 5-degree sector bins are descriptive aggregation, not a claim of 5-degree alignment accuracy. No additional denoising or resizing is applied.",
        "", "## Camera configuration audit", "",
        f"Distinct recorded bias configurations across nine camera metadata files: {len(biases)}."]
    lines += [f"- `{bias}`" for bias in biases]
    lines += ["", "These metadata agree between the analysed conditions. Hardware event-rate control and other filtering states are not established by the recorded bias fields.",
              "", "## Sampling quality", "",
              f"Median inter-sample intervals span {min(q['median_sample_interval_ms'] for q in qualities):.3f}–{max(q['median_sample_interval_ms'] for q in qualities):.3f} ms. The largest interval inside the selected steady ranges is {max(q['maximum_steady_sample_interval_ms'] for q in qualities):.3f} ms; the largest anywhere in a run is {max(q['maximum_sample_interval_ms'] for q in qualities):.3f} ms. Interpolation does not remove that sampling limitation.",
              "", "## Event processing status", ""]
    for row in event_rows:
        lines.append(f"- {row['condition']} {row['run']}: {row['status']}")
    summary = event_summary(event_rows)
    if summary:
        lines += ['', f'## Event-stream results over the common {end_angle-start_angle:g} degree rotation', '',
                  'All summaries weight the three independent acquisitions equally. ± values are between-run sample SDs.', '',
                  '| Condition | Million CD events/s | Thousand events/degree | Mean events per 2 ms | Active pixels per 2 ms | ON fraction |',
                  '|---|---:|---:|---:|---:|---:|']
        for row in summary:
            lines.append(f"| {row['condition']} | {row['events_per_second_mean']/1e6:.3f} ± {row['events_per_second_between_run_sd']/1e6:.3f} | "
                         f"{row['events_per_degree_mean']/1e3:.3f} ± {row['events_per_degree_between_run_sd']/1e3:.3f} | "
                         f"{row['mean_events_per_2ms_mean']:.0f} | {100*row['mean_active_pixel_fraction_2ms_mean']:.2f}% | {100*row['on_fraction_mean']:.2f}% |")
        lines += ['', 'The table describes the measured recorded output. Differences in rate, angular density or active-pixel coverage do not by themselves establish a sensor bandwidth ceiling, event loss, reduced scene information, or a change in VPR accuracy. Fixed camera biases alone do not establish equal effective sensor response or unchanged undocumented hardware filters.', '']
        for row in event_rows:
            if row.get('empty_2ms_fraction', 0) > 0:
                lines.append(f"- Review anomaly: {row['condition']} {row['run']} has {100*row['empty_2ms_fraction']:.2f}% empty complete 2 ms windows. This acquisition is retained; no dropout cause is established. Compare its angular activity with the other acquisitions in event_density_by_angle.")
        changed = [r for r in event_rows if 'manual_evk4_motion_s_used' in r and
                   'local_manual_evk4_motion_s' in r and
                   not math.isclose(r['manual_evk4_motion_s_used'], r['local_manual_evk4_motion_s'], abs_tol=1e-9)]
        if not changed:
            changed = [dict(condition=q['condition'],run=q['run'],
                            manual_evk4_motion_s_used=float(q['manual_evk4_motion_s']),
                            local_manual_evk4_motion_s=float(q['local_manual_evk4_motion_s']))
                       for q in qualities if 'local_manual_evk4_motion_s' in q and
                       not math.isclose(float(q['manual_evk4_motion_s']),float(q['local_manual_evk4_motion_s']),abs_tol=1e-9)]
        if changed:
            lines += ['', '### Imported alignment values', '',
                      'The imported event calculations use the accepted Linux alignment values. Local source setup files were preserved. These differ from the local copy as follows:', '',
                      '| Condition/run | Linux value used s | Local setup value s |', '|---|---:|---:|']
            for row in changed:
                lines.append(f"| {row['condition']} {row['run']} | {row['manual_evk4_motion_s_used']:.6f} | {row['local_manual_evk4_motion_s']:.6f} |")
        unmatched = [r.get('events_not_assigned_to_angular_bins', 0) / r['cd_events'] for r in event_rows if r['status'] == 'processed']
        if any(unmatched):
            lines += ['', f"Angular-bin reconciliation: at most {100*max(unmatched):.6f}% of full-interval events were outside the angular histogram after boundary-time rounding/interpolation. Full-interval totals are retained; imported bins were not adjusted."]
    lines += ["", "Visual landmark correspondence and motor-encoder trajectory repeatability are different measurements. Manual alignment acceptance is not an independently quantified angle error. The event arrays, when generated, retain native-pixel ON/OFF counts as well as angular activity.",
              "", "## Files", "",
              "- motion_per_run.csv: speed, response and position measurements for each acquisition.",
              "- condition_summary.csv: equally weighted independent-run summaries.",
              "- revolution_periods.csv: encoder-derived full-turn durations.",
              "- data_quality.csv: sampling and camera configuration audit.",
              "- event_statistics.csv and event_sector_counts.csv: actual RAW results, when available.",
              "- event_condition_summary.csv: full-rotation event measures averaged across independent acquisitions, with between-run SDs.",
              "- event_revolution_plan.csv: checked full-rotation boundaries and recording-relative times for all nine runs.",
              "- provenance.json: source hashes and processing assumptions.",
              "- position_trajectory_repeatability.csv: pairwise displacement differences at common scheduled-T0 times during overlapping steady intervals.",
              "- event_sector_counts_30deg_centred.csv: continuous event counts and rates in sectors centred on 0,30,...330 degrees.",
              "- event_arrays/: native-pixel ON/OFF counts and disjoint complete 2 ms temporal counts, when RAW decoding is enabled.",
              "- figures/: PDF, PNG and SVG plots, when plotting is enabled."]
    (output / "rig_evaluation_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiments-root", type=Path, default=Path(__file__).resolve().parent.parent / "Experiments")
    parser.add_argument("--output-root", type=Path, required=True)
    event_input = parser.add_mutually_exclusive_group()
    event_input.add_argument("--events", action="store_true", help="Decode available RAW recordings")
    event_input.add_argument("--event-results", type=Path, help="Import returned Linux Rig-Evaluation results")
    alignment = parser.add_mutually_exclusive_group()
    alignment.add_argument("--alignment-results", type=Path, help="Reuse accepted manual times from a returned data_quality.csv")
    alignment.add_argument("--vpr-results", type=Path, help="Completed final VPR results directory; validate and reuse its frozen encoder-to-camera mapping")
    parser.add_argument("--event-start-angle", type=float, default=4.0, help="Shared wrapped phase at start of event arc (default: 4 deg)")
    parser.add_argument("--event-end-angle", type=float, default=364.0, help="Unwrapped end of event arc (default: 364 deg, one complete turn)")
    parser.add_argument("--no-plots", action="store_true", help="Do not require matplotlib")
    parser.add_argument("--ramp-seconds", type=float, default=8.0, help="Operator-supplied accel/decel time at reference speed")
    parser.add_argument("--ramp-reference-rpm", type=float, default=500.0, help="Motor rpm defining the supplied ramp time")
    args = parser.parse_args()
    angular_edges(args.event_start_angle, args.event_end_angle)
    if not all(math.isfinite(value) and value > 0 for value in (args.ramp_seconds, args.ramp_reference_rpm)):
        parser.error("Ramp time and reference rpm must be finite and positive")
    root, output = args.experiments_root.expanduser().resolve(), args.output_root.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Output is not empty; select a new output directory')
    output.mkdir(parents=True, exist_ok=True)
    results, qualities, periods, traces, sources = [], [], [], [], []
    for condition in CONDITIONS:
        experiment = root / condition
        setup = read_csv(experiment / "experiment_setup.csv")
        for label in ("run1", "run2", "run3"):
            matches = [row for row in setup if row["run_label"] == label]
            if len(matches) != 1:
                raise ValueError(f"Expected one setup row for {condition} {label}")
            result, quality, turns, trace, inputs = analyse_run(experiment, matches[0], args)
            results.append(result); qualities.append(quality); periods.extend(turns)
            traces.append(trace); sources.extend(inputs)
            print(f"[MOTION] {condition} {label}: {result['mean_platform_rpm']:.5f} platform rpm", flush=True)
    summaries = summarise(results)
    if args.alignment_results:
        sources.append(apply_alignment_results(args.alignment_results, traces, qualities))
    vpr_alignment = None
    if args.vpr_results:
        accepted_sources, vpr_alignment = apply_vpr_results(args.vpr_results, root, traces, qualities)
        sources.extend(accepted_sources)
    if args.events:
        from rig_vpr.algorithm import raw_fingerprint
        for trace in traces:
            raw = trace['experiment']/trace['setup']['raw_file']
            if not raw.is_file():
                raise FileNotFoundError(f'RAW recording is required: {raw}. Run --events on Linux.')
            actual = raw_fingerprint(raw)
            expected = trace.get('expected_raw')
            if expected and any(actual[k] != expected[k] for k in ('size_bytes','first_and_last_mib_sha256')):
                raise ValueError(f'{raw}: RAW identity differs from the final VPR extraction')
    plans = []
    for trace in traces:
        first, last, level, camera_times = event_interval(trace, args.event_start_angle, args.event_end_angle)
        plans.append(dict(condition=trace['condition'], run=trace['run'], revolution=trace['selected_rev'],
            comparison_start_deg=args.event_start_angle, comparison_end_deg=args.event_end_angle,
            angular_extent_deg=args.event_end_angle-args.event_start_angle,
            continuous_encoder_start_deg=level, continuous_encoder_end_deg=level+args.event_end_angle-args.event_start_angle,
            wrapped_start_deg=args.event_start_angle, wrapped_end_deg=args.event_end_angle % 360,
            camera_start_s=first, camera_end_s=last, duration_s=last-first,
            motor_start_s=float(np.interp(first,camera_times,trace['t'])),
            motor_end_s=float(np.interp(last,camera_times,trace['t'])),
            manual_evk4_motion_s_used=trace['setup']['manual_evk4_motion_s'],
            studio_duration_s_used=trace['setup']['studio_duration_s'], steady_check='passed'))
    write_csv(output/'event_revolution_plan.csv', plans)
    event_rows, sector_rows = [], []
    if args.event_results:
        event_rows, sector_rows = import_event_results(args.event_results, traces, args.event_start_angle, args.event_end_angle)
        if args.vpr_results:
            for row, plan in zip(sorted(event_rows,key=lambda r:(r['condition'],r['run'])), sorted(plans,key=lambda r:(r['condition'],r['run']))):
                for imported, current in [('manual_evk4_motion_s_used','manual_evk4_motion_s_used'),('studio_duration_s_used','studio_duration_s_used'),('camera_start_s','camera_start_s'),('camera_end_s','camera_end_s')]:
                    if not math.isclose(float(row[imported]),float(plan[current]),rel_tol=0,abs_tol=2e-6):
                        raise ValueError('Imported event results disagree with final VPR alignment; reprocess RAW')
        print(f"[EVENTS] Imported and reconciled {len(event_rows)} Linux RAW results", flush=True)
    for trace in ([] if args.event_results else traces):
        raw = trace["experiment"] / trace["setup"]["raw_file"]
        row = dict(condition=trace["condition"], run=trace["run"], raw_file=str(raw))
        if not raw.is_file():
            row["status"] = "RAW not present on this computer"
        elif not args.events:
            row["status"] = "RAW available; run with --events"
        else:
            event_result, detail = event_statistics(trace, args.event_start_angle, args.event_end_angle, output)
            row.update(event_result); sector_rows.extend(detail)
        event_rows.append(row)
        print(f"[EVENTS] {row['condition']} {row['run']}: {row['status']}", flush=True)
    for name, rows in [("motion_per_run", results), ("condition_summary", summaries),
                       ("revolution_periods", periods), ("data_quality", qualities),
                       ("event_statistics", event_rows), ("event_sector_counts", sector_rows)]:
        write_csv(output / (name + ".csv"), rows)
    trajectory = trajectory_repeatability(traces)
    write_csv(output/'position_trajectory_repeatability.csv', trajectory)
    centred = []
    for path in sorted((output/'event_arrays').glob('*_sectors_30deg.csv')):
        centred.extend(read_csv(path))
    write_csv(output/'event_sector_counts_30deg_centred.csv', centred)
    if centred and args.vpr_results:
        write_csv(output/'event_vpr_sector_comparison.csv', event_vpr_comparison(args.vpr_results, centred))
    write_csv(output / 'event_condition_summary.csv', event_summary(event_rows))
    provenance = dict(source_files=[dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                                   for path in sorted(set(sources))],
                      script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      common_revolutions=CONDITIONS, stopped_motor_rpm=.5, confirmation_s=.25,
                      operator_supplied_ramp_seconds=args.ramp_seconds, operator_supplied_ramp_reference_motor_rpm=args.ramp_reference_rpm,
                      included_conditions=list(CONDITIONS),
                      event_sector_deg=[args.event_start_angle, args.event_end_angle], accumulation_ms=2,
                      manual_alignment="supplied setup or accepted alignment-results values; not refitted")
    if vpr_alignment:
        provenance['accepted_vpr_alignment'] = vpr_alignment
    provenance['event_30deg_sector_start_deg'] = -15
    provenance['event_window_policy'] = 'Disjoint complete 2 ms windows; total event counts include the final incomplete tail'
    provenance['runtime'] = dict(python=__import__('sys').version, numpy=np.__version__)
    if args.event_results:
        provenance['imported_event_results'] = [dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for path in sorted(args.event_results.glob('*.csv'))]
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    report(output, summaries, qualities, event_rows, results, args.event_start_angle, args.event_end_angle)
    with (output/'rig_evaluation_report.md').open('a',encoding='utf-8') as handle:
        handle.write('\n## Pairwise angular-trajectory repeatability\n\n')
        handle.write('Displacement is compared at the same time relative to scheduled motor T0, after subtracting each initial stationary angle, over each pair\'s overlapping steady interval. Original starting-orientation scatter is reported separately. This is encoder-derived temporal repeatability, not independently calibrated platform angle accuracy.\n\n')
        handle.write('| Condition | Run pair | MAE deg | RMSE deg | Maximum absolute difference deg |\n|---|---|---:|---:|---:|\n')
        for row in trajectory:
            handle.write(f"| {row['condition']} | {row['reference_run']} / {row['query_run']} | {row['displacement_mae_deg']:.4f} | {row['displacement_rmse_deg']:.4f} | {row['maximum_absolute_displacement_difference_deg']:.4f} |\n")
        if vpr_alignment:
            handle.write(f"\nAccepted VPR protocol SHA256: `{vpr_alignment['protocol_sha256']}`. Camera-relative times use the same prepared encoder samples as the final VPR analysis.\n")
        if centred:
            handle.write('\nContinuous event rates and density are additionally reported in twelve centred 30-degree sectors, [-15,15), [15,45), ..., [315,345). The zero-centred sector combines the portions across the wrapped boundary; all events are retained. These bins share the orientation convention of the VPR analysis.\n')
            handle.write('\nNative-pixel spatial maps show the recorded signal over the selected full turn, not a measurement of sensor noise. The event-VPR comparison table pairs continuous event statistics with sampled VPR recall by query acquisition, representation, reference and sector; it is descriptive and does not establish a cause.\n')
    if not args.no_plots:
        figure_root = output / "figures"
        figure_root.mkdir(exist_ok=True)
        figures(traces, results, summaries, event_rows, figure_root, sector_rows)
        if centred:
            spatial_figures(output, event_rows)
    stamp = dict(completed_utc=datetime.now(timezone.utc).isoformat(), motion_acquisitions=len(results),
        processed_event_acquisitions=sum(r['status']=='processed' for r in event_rows),
        accepted_vpr_alignment=vpr_alignment,
        provenance_sha256=hashlib.sha256((output/'provenance.json').read_bytes()).hexdigest())
    (output/'rig_motion_complete.json').write_text(json.dumps(stamp,indent=2)+'\n',encoding='utf-8')
    if stamp['processed_event_acquisitions'] == 9:
        (output/'rig_evaluation_complete.json').write_text(json.dumps(stamp,indent=2)+'\n',encoding='utf-8')
    print(f"[DONE] {output}", flush=True)


if __name__ == "__main__":
    main()
