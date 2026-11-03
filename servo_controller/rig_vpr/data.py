"""Read-only acquisition discovery and camera/encoder frame planning."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from decimal import Decimal
from pathlib import Path

import numpy as np

from generate_vpr_time_bins import build_motion_start_anchor, load_motion_rows, map_lattepanda_time_to_camera_us

CONDITIONS = {"Low-Speed": (15.0, 1), "Medium-Speed": (50.0, 3), "High-Speed": (150.0, 11)}
REFERENCE = "Low-Speed/run1"


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path, parent):
    return Path(os.path.relpath(Path(path).resolve(), Path(parent).resolve())).as_posix()


def resolve(parent, value):
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (Path(parent) / path).resolve()


def discover(root, protocol_path, settings, all_pairs=False):
    """Snapshot the existing prepared mappings, without running preparation again."""
    root, protocol_path = Path(root).resolve(), Path(protocol_path).resolve()
    acquisitions, warnings = [], []
    for condition, (platform_rpm, rev) in CONDITIONS.items():
        folder = root / condition
        setup_path = folder / "experiment_setup.csv"
        setup = read_csv(setup_path)
        catalog_path = folder / "outputs" / "vpr_runs.csv"
        catalog = read_csv(catalog_path) if catalog_path.is_file() else []
        for label in ("run1", "run2", "run3"):
            matches = [r for r in setup if r["run_label"] == label]
            if len(matches) != 1 or matches[0]["include"].lower().strip() not in ("true", "1", "yes", "y"):
                raise ValueError(f"{condition}/{label}: expected one included setup row")
            row = matches[0]
            prepared = [r for r in catalog if r["run_label"] == label]
            if len(prepared) > 1:
                raise ValueError(f"{condition}/{label}: duplicate catalogue rows")
            recorded, raw = resolve(folder, row["recorded_csv"]), resolve(folder, row["raw_file"])
            bins_path, alignment = None, "manual_setup"
            chosen = row
            if prepared:
                chosen = prepared[0]
                if resolve(catalog_path.parent, chosen["recorded_csv"]) != recorded or resolve(catalog_path.parent, chosen["raw_file"]) != raw:
                    raise ValueError(f"{condition}/{label}: setup/catalogue acquisition paths disagree")
                bins_path = resolve(catalog_path.parent, chosen["time_bins_csv"])
                if not bins_path.is_file():
                    raise FileNotFoundError(f"Prepared catalogue points to missing bins: {bins_path}")
                alignment = "prepared_encoder_sample_camera_us"
                for field in ("manual_evk4_motion_s", "studio_duration_s"):
                    a, b = row.get(field, ""), chosen.get(field, "")
                    if a and b and not math.isclose(float(a), float(b), abs_tol=1e-7):
                        warnings.append(f"{condition}/{label}: setup {field}={a}, prepared={b}; prepared value retained")
            elif row.get("sync_mode", "manual").lower() != "manual":
                raise ValueError(f"{condition}/{label}: prepare trigger alignment before VPR")
            elif not row.get("manual_evk4_motion_s"):
                raise ValueError(f"{condition}/{label}: manual motion onset is missing")
            else:
                warnings.append(f"{condition}/{label}: no prepared catalogue entry; using the setup onset with the shared motion-onset mapper")
            duration = float(chosen["studio_duration_s"])
            onset = chosen.get("manual_evk4_motion_s", "")
            acquisitions.append(dict(key=f"{condition}/{label}", condition=condition, run=label,
                run_id=row["run_id"], nominal_platform_rpm=platform_rpm, revolution=rev,
                recorded_csv=relative(recorded, root), raw_file=relative(raw, root),
                camera_metadata=relative(resolve(folder, row["camera_metadata"]), root),
                setup_csv=relative(setup_path, root), catalog_csv=relative(catalog_path, root) if prepared else None,
                time_bins_csv=relative(bins_path, root) if bins_path else None,
                alignment_source=alignment, sync_mode=chosen.get("sync_mode", "manual"),
                manual_evk4_motion_s=float(onset) if onset else None, studio_duration_s=duration))
    result = dict(schema="rig-sad-vpr-v1", protocol_status="baseline_settings_provisional",
        upstream="https://github.com/Tobias-Fischer/sparse-event-vpr",
        experiments_root=relative(root, protocol_path.parent), reference=REFERENCE,
        pair_mode="all-independent-pairs" if all_pairs else "fixed-reference",
        settings=settings, acquisitions=acquisitions, discovery_warnings=warnings)
    validate_protocol(result)
    return result


def validate_protocol(protocol):
    if protocol.get("schema") != "rig-sad-vpr-v1":
        raise ValueError("Unsupported protocol schema")
    entries, cfg = protocol["acquisitions"], protocol["settings"]
    for field in ("key", "run_id", "recorded_csv", "raw_file"):
        values = [r[field] for r in entries]
        if len(values) != len(set(values)) or any(not v for v in values):
            raise ValueError(f"Acquisition {field} must be unique and non-empty")
    if protocol["reference"] not in {r["key"] for r in entries}:
        raise ValueError("Reference acquisition is not in the protocol")
    if protocol["pair_mode"] not in ("fixed-reference", "all-independent-pairs"):
        raise ValueError("Invalid pair_mode")
    for key in ("accumulation_ms", "angle_step_deg", "tolerance_deg", "steady_tolerance_percent"):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if cfg["tolerance_deg"] > 180 or cfg["angle_step_deg"] > 180:
        raise ValueError("Angular tolerance and step must not exceed 180 degrees")
    if cfg.get("accumulation_mode","time") not in ("time","angle"):
        raise ValueError("Accumulation mode must be time or angle")
    if cfg.get("motion_screen","steady") not in ("steady","forward"):
        raise ValueError("Motion screen must be steady or forward")
    if cfg.get("accumulation_mode","time")=="angle" and (not math.isfinite(cfg.get("accumulation_deg",float("nan"))) or not 0<cfg["accumulation_deg"]<=360):
        raise ValueError("Angular accumulation must be in (0,360] degrees")
    if not math.isclose(360 / cfg["angle_step_deg"], round(360 / cfg["angle_step_deg"]), abs_tol=1e-8):
        raise ValueError("angle_step_deg must divide 360 evenly")
    if not 0 <= cfg["start_wrapped_deg"] < 360:
        raise ValueError("start_wrapped_deg must be in [0, 360)")
    if cfg["width"] < 1 or cfg["height"] < 1 or cfg["pixels"] < 0:
        raise ValueError("Use positive spatial dimensions and pixels >= 0")
    if any(not isinstance(cfg[k], int) or isinstance(cfg[k], bool) for k in ("width", "height", "pixels", "seed")) or cfg["seed"] < 0:
        raise ValueError("Dimensions, pixels and seed must be integers; seed must be non-negative")
    if cfg["pixels"] > cfg["width"] * cfg["height"]:
        raise ValueError("Sparse pixel count exceeds the spatial grid")
    if cfg["polarity"] not in ("sum", "split") or cfg["normalization"] not in ("none", "l1"):
        raise ValueError("Invalid polarity or normalization")
    for entry in entries:
        if entry["condition"] not in CONDITIONS or not isinstance(entry["revolution"], int) or entry["revolution"] < 0:
            raise ValueError("Invalid condition or revolution")
        if not math.isfinite(entry["studio_duration_s"]) or entry["studio_duration_s"] <= 0:
            raise ValueError("Studio duration must be positive and finite")
        onset = entry["manual_evk4_motion_s"]
        if entry["sync_mode"] == "manual" and (onset is None or not 0 <= onset < entry["studio_duration_s"]):
            raise ValueError("Manual onset must be inside the Studio duration")


def load_protocol(path):
    path = Path(path).expanduser().resolve()
    protocol = json.loads(path.read_text(encoding="utf-8"))
    validate_protocol(protocol)
    return protocol, resolve(path.parent, protocol["experiments_root"])


def load_trace(root, entry):
    recorded = resolve(root, entry["recorded_csv"])
    motion = load_motion_rows(recorded, entry["run_id"])
    if entry["time_bins_csv"]:
        rows = read_csv(resolve(root, entry["time_bins_csv"]))
        if len(rows) != len(motion) or {r["run_id"] for r in rows} != {entry["run_id"]}:
            raise ValueError(f"{entry['key']}: prepared bins do not match recorded acquisition")
        for a, b in zip(rows, motion):
            if a["encoder_sample_motor_pc_monotonic_ns"] != b["motor_pc_monotonic_ns"] or not math.isclose(float(a["platform_angle_deg"]), float(b["platform_angle_deg"]), abs_tol=1e-8):
                raise ValueError(f"{entry['key']}: stale or mismatched prepared encoder bins")
        times = np.array([float(r["encoder_sample_camera_us"]) / 1e6 for r in rows])
        # Validate the frozen onset against the actual mapping rather than trusting labels alone.
        if entry["sync_mode"] == "manual":
            anchor = build_motion_start_anchor(motion, round(entry["manual_evk4_motion_s"] * 1e6))
            expected = np.array([map_lattepanda_time_to_camera_us(anchor, r["lattepanda_time_s"]) / 1e6 for r in motion])
            if not np.allclose(times, expected, rtol=0, atol=1.1e-6):
                raise ValueError(f"{entry['key']}: catalogue onset and prepared sample mapping disagree; prepare the experiment again")
    else:
        anchor = build_motion_start_anchor(motion, round(entry["manual_evk4_motion_s"] * 1e6), Decimal("0.5"), 3)
        times = np.array([map_lattepanda_time_to_camera_us(anchor, r["lattepanda_time_s"]) / 1e6 for r in motion])
    angles = np.array([float(r["platform_angle_deg"]) for r in motion])
    speed = np.array([float(r["recorded_platform_feedback_rpm"]) for r in motion])
    command = np.array([float(r["motor_speed_command_rpm"]) / 2 for r in motion])
    if not np.all(np.isfinite(np.r_[times, angles, speed, command])) or not np.all(np.diff(times) > 0):
        raise ValueError(f"{entry['key']}: non-finite or non-increasing telemetry")
    if not np.allclose(speed*2, [float(r["recorded_motor_feedback_rpm"]) for r in motion], rtol=0, atol=.002):
        raise ValueError(f"{entry['key']}: platform feedback does not agree with the nominal 2:1 reduction")
    return dict(times=times, angles=angles, speed=speed, command=command)


def crossing_time(times, angles, level):
    indices = np.flatnonzero((angles[:-1] <= level) & (angles[1:] >= level) & (np.diff(angles) > 0))
    if not len(indices):
        raise ValueError(f"No forward encoder crossing at {level:.6f} degrees")
    i = indices[0]
    return float(times[i] + (level - angles[i]) / (angles[i+1] - angles[i]) * (times[i+1] - times[i]))


def accumulation_label(cfg):
    if cfg.get("accumulation_mode","time")=="angle":
        return f"{cfg['accumulation_deg']:g}° angular"
    return f"{cfg['accumulation_ms']:g} ms temporal"


def frame_plan(trace, entry, cfg):
    """Sample a full turn at fixed angle centres, with time or angle apertures."""
    t, a = trace["times"], trace["angles"]
    start = cfg["start_wrapped_deg"] + 360 * math.ceil((a[0] + 360 * entry["revolution"] - cfg["start_wrapped_deg"]) / 360)
    levels = start + np.arange(round(360 / cfg["angle_step_deg"])) * cfg["angle_step_deg"]
    # Require the entire selected turn, including the crossing at its end.
    turn_end = crossing_time(t, a, start + 360)
    centers_us = np.rint(np.array([crossing_time(t, a, v) for v in levels]) * 1e6).astype(np.int64)
    if cfg.get("accumulation_mode","time")=="angle":
        half=cfg["accumulation_deg"]/2
        starts_us=np.rint([crossing_time(t,a,v-half)*1e6 for v in levels]).astype(np.int64)
        ends_us=np.rint([crossing_time(t,a,v+half)*1e6 for v in levels]).astype(np.int64)
        if np.any(ends_us<=starts_us):
            raise ValueError("Angular windows must resolve to at least 1 microsecond")
    else:
        duration_us = round(cfg["accumulation_ms"] * 1000)
        if duration_us < 1:
            raise ValueError("Accumulation must be at least 1 microsecond")
        starts_us = centers_us - duration_us // 2
        ends_us = starts_us + duration_us
    lower, upper = starts_us[0] / 1e6, max(turn_end, ends_us[-1] / 1e6)
    if not t[0] <= lower < upper <= t[-1] or lower < 0 or upper > entry["studio_duration_s"]:
        raise ValueError(f"{entry['key']}: centred event windows extend outside recorded camera/encoder coverage")
    i = max(0, np.searchsorted(t, lower, side="right")-1)
    j = min(len(t), np.searchsorted(t, upper, side="left")+1)
    if np.any(np.diff(a[i:j]) <= 0):
        raise ValueError(f"{entry['key']}: selected motion is not strictly forward")
    nominal = entry["nominal_platform_rpm"]
    worst = float(np.max(abs(trace["speed"][i:j] - nominal)) / nominal * 100)
    if cfg.get("motion_screen","steady")=="steady" and (worst > cfg["steady_tolerance_percent"] or np.max(abs(trace["command"][i:j] - nominal)) > nominal * .01):
        raise ValueError(f"{entry['key']}: selected windows fail steady screening ({worst:.3f}% worst speed error)")
    return dict(center_us=centers_us, start_us=starts_us, end_us=ends_us,
        angle_deg=np.interp(centers_us / 1e6, t, a) % 360,
        requested_angle_deg=levels % 360,
        window_angle_span_deg=np.interp(ends_us/1e6, t, a)-np.interp(starts_us/1e6, t, a),
        speed_rpm=np.interp(centers_us/1e6, t, trace["speed"]),
        turn_start_s=float(centers_us[0]/1e6),turn_end_s=turn_end,
        minimum_feedback_rpm=float(trace["speed"][i:j].min()),maximum_feedback_rpm=float(trace["speed"][i:j].max()),
        worst_speed_error_percent=worst,
        max_telemetry_interval_ms=float(np.max(np.diff(t[i:j])) * 1000))


def check_inputs(protocol, root, require_raw=True):
    plans, audit = {}, []
    biases = []
    for entry in protocol["acquisitions"]:
        trace = load_trace(root, entry)
        plan = frame_plan(trace, entry, protocol["settings"])
        raw = resolve(root, entry["raw_file"])
        if require_raw and not raw.is_file():
            raise FileNotFoundError(f"{entry['key']}: missing RAW recording {raw}. Run on the Linux recording machine.")
        if raw.suffix.lower() == ".raw" and not raw.name.lower().startswith(entry["run_id"].lower()):
            raise ValueError(f"{entry['key']}: RAW filename does not match run_id")
        sources = {field:dict(path=entry[field], sha256=sha256(resolve(root, entry[field])))
            for field in ("recorded_csv", "setup_csv", "catalog_csv", "time_bins_csv", "camera_metadata") if entry.get(field)}
        frozen = protocol.get("frozen_input_sha256", {}).get(entry["key"])
        if frozen is not None and frozen != {k:v["sha256"] for k,v in sources.items()}:
            raise ValueError(f"{entry['key']}: an input changed since protocol creation. Create a new protocol after reviewing the change.")
        camera = json.loads(resolve(root, entry["camera_metadata"]).read_text(encoding="utf-8"))
        if camera.get("run_id") != entry["run_id"]:
            raise ValueError(f"{entry['key']}: camera metadata run_id mismatch")
        if camera.get("camera_healthy_at_finalize") is False:
            raise ValueError(f"{entry['key']}: camera metadata flags an unhealthy recording")
        biases.append(json.dumps(camera.get("applied_biases"), sort_keys=True))
        plans[entry["key"]] = plan
        audit.append(dict(key=entry["key"], frames=len(plan["center_us"]), raw_exists=raw.is_file(),
            alignment_source=entry["alignment_source"], sources=sources,
            maximum_telemetry_interval_ms=plan["max_telemetry_interval_ms"],
            worst_speed_error_percent=plan["worst_speed_error_percent"],
            first_camera_center_s=float(plan["center_us"][0]/1e6), last_camera_center_s=float(plan["center_us"][-1]/1e6), applied_biases=biases[-1]))
    if len(set(biases)) != 1:
        raise ValueError("Recorded camera bias configurations differ across acquisitions")
    return plans, audit
