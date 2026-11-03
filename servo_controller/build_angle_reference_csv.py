"""Index wrapped-angle crossings and link them to recorded encoder rows.

Used by vpr_media.py query-angle and by this script's legacy CLI. Inputs must
have one-to-one recorded/bin rows. Output is a reference index, not a new
measurement. Default paths point to a historical run; pass explicit paths.
Current crossing lookup returns the first visit to each unwrapped angle level;
reversals/repeated visits are not exhaustively enumerated. Sample times are
currently bin midpoints, which differ from explicit encoder timestamps near
irregular or extended bins. Preserve these caveats when interpreting results.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BINS = ROOT / "Speed Profiles" / "50-rpm_run1_EVK4_time_bins.csv"
DEFAULT_RECORDED = ROOT / "Speed Profiles" / "50-rpm_20261007_145359_recorded.csv"
DEFAULT_RAW = ROOT / "vids" / "recordings" / "73d57ea887144bf7a54729d1b13c34ba_20261007_145323.raw"
DEFAULT_OUTPUT = ROOT / "Speed Profiles" / "50-rpm_120deg_reference_with_recorded_angles.csv"


@dataclass(frozen=True)
class Sample:
    time_s: float
    angle_deg: float
    timing_uncertainty_ms: float
    angle_uncertainty_deg: float


def load_samples(path: Path) -> list[Sample]:
    # Historical behavior uses bin midpoints, not encoder_sample_camera_us.
    # Changing this can improve timing semantics but will change existing
    # references, especially extended first/last bins; update fixtures explicitly.
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or ())
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError("At least two encoder bins are required")
    seconds_schema = {"evk4_bin_start_s", "evk4_bin_end_s"}.issubset(fields)
    microseconds_schema = {"camera_start_us", "camera_end_us"}.issubset(fields)
    if not seconds_schema and not microseconds_schema:
        raise ValueError("Time-bin CSV needs EVK4 second columns or camera microsecond columns")
    return [
        Sample(
            (
                (float(row["evk4_bin_start_s"]) + float(row["evk4_bin_end_s"])) / 2.0
                if seconds_schema
                else (float(row["camera_start_us"]) + float(row["camera_end_us"])) / 2_000_000.0
            ),
            float(row["platform_angle_deg"]),
            float(row["timing_uncertainty_ms"]),
            float(row.get("angle_uncertainty_deg", "") or 0.0),
        )
        for row in rows
    ]


def load_recorded(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("Recorded CSV has no samples")
    return rows


def crossing_time(samples: list[Sample], level_deg: float) -> float | None:
    # First crossing only; oscillation can revisit this level later. A None
    # result means no segment spans the level, not an extrapolated estimate.
    for left, right in zip(samples, samples[1:]):
        low, high = sorted((left.angle_deg, right.angle_deg))
        if low <= level_deg <= high and right.angle_deg != left.angle_deg:
            fraction = (level_deg - left.angle_deg) / (right.angle_deg - left.angle_deg)
            return left.time_s + fraction * (right.time_s - left.time_s)
    return None


def build_rows(
    samples: list[Sample],
    recorded: list[dict[str, str]],
    target_deg: float,
    tolerance_deg: float,
    run_label: str,
    raw_path: Path,
    bins_path: Path,
    recorded_path: Path,
    raw_duration_s: float,
    clip_half_width_s: float,
    source_fps: float,
    accumulation_ms: float,
) -> list[dict[str, object]]:
    # Recorded and bin rows must retain one-to-one order. The actual row is
    # nearest in angle, not nearest in time; revisits can therefore select a
    # different pass. This index was designed around monotonic turn experiments.
    if len(recorded) != len(samples):
        raise ValueError("Recorded CSV and time-bin CSV must have one-to-one rows")
    if not 0 <= target_deg < 360:
        raise ValueError("Target angle must be in [0, 360)")
    if tolerance_deg <= 0:
        raise ValueError("Tolerance must be greater than zero")
    minimum = min(sample.angle_deg for sample in samples)
    maximum = max(sample.angle_deg for sample in samples)
    first_k = math.ceil((minimum - target_deg) / 360.0)
    last_k = math.floor((maximum - target_deg) / 360.0)
    origin = samples[0].angle_deg
    rows: list[dict[str, object]] = []
    for occurrence, k in enumerate(range(first_k, last_k + 1), start=1):
        level = target_deg + 360.0 * k
        target_time = crossing_time(samples, level)
        entry_time = crossing_time(samples, level - tolerance_deg)
        exit_time = crossing_time(samples, level + tolerance_deg)
        if target_time is None or entry_time is None or exit_time is None:
            continue
        range_start, range_end = sorted((entry_time, exit_time))
        direction = "forward" if exit_time >= entry_time else "reverse"
        raw_available = 0.0 <= target_time <= raw_duration_s
        clip_start = max(0.0, target_time - clip_half_width_s) if raw_available else None
        clip_end = min(raw_duration_s, target_time + clip_half_width_s) if raw_available else None
        revolution = max(0, math.floor(max(0.0, level - origin) / 360.0 + 1e-9))
        actual_index = min(
            range(len(recorded)),
            key=lambda index: abs(float(recorded[index]["platform_angle_deg"]) - level),
        )
        actual = recorded[actual_index]
        actual_evk4_time_s = samples[actual_index].time_s
        rows.append(
            {
                "query_angle_deg": f"{target_deg:.6f}",
                "tolerance_deg": f"{tolerance_deg:.6f}",
                "run_label": run_label,
                "run_id": actual["run_id"],
                "occurrence": occurrence,
                "revolution": revolution,
                "direction": direction,
                "range_start_evk4_s": f"{range_start:.9f}",
                "target_time_evk4_s": f"{target_time:.9f}",
                "range_end_evk4_s": f"{range_end:.9f}",
                "actual_sample_index": actual_index,
                "actual_elapsed_s": actual["elapsed_s"],
                "actual_lattepanda_time_s": actual["lattepanda_time_s"],
                "actual_evk4_time_s": f"{actual_evk4_time_s:.9f}",
                "actual_platform_angle_deg": actual["platform_angle_deg"],
                "actual_platform_wrapped_angle_deg": actual["platform_wrapped_angle_deg"],
                "actual_encoder_position_pulses": actual["encoder_position_pulses"],
                "actual_motor_speed_command_rpm": actual.get("motor_speed_command_rpm", ""),
                "actual_recorded_motor_feedback_rpm": actual.get("recorded_motor_feedback_rpm", ""),
                "actual_recorded_platform_feedback_rpm": actual.get("recorded_platform_feedback_rpm", ""),
                "actual_scene_displacement_ain0_v": actual.get("scene_displacement_ain0_v", ""),
                "actual_scene_vertical_displacement_mm": actual.get("scene_vertical_displacement_mm", ""),
                "actual_acquisition_duration_ms": actual["acquisition_duration_ms"],
                "clip_start_evk4_s": f"{clip_start:.9f}" if clip_start is not None else "",
                "clip_end_evk4_s": f"{clip_end:.9f}" if clip_end is not None else "",
                "source_fps": f"{source_fps:.3f}",
                "accumulation_ms": f"{accumulation_ms:.3f}",
                "timing_uncertainty_ms": f"{max(s.timing_uncertainty_ms for s in samples):.3f}",
                "angle_uncertainty_deg": f"{max(s.angle_uncertainty_deg for s in samples):.3f}",
                "raw_available": str(raw_available).lower(),
                "raw_recording_duration_s": f"{raw_duration_s:.6f}",
                "raw_file": str(raw_path.resolve()),
                "bins_file": str(bins_path.resolve()),
                "recorded_file": str(recorded_path.resolve()),
                "gif_filename": f"{run_label}_rev{revolution:02d}_{target_deg:g}deg.gif" if raw_available else "",
            }
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bins", type=Path, default=DEFAULT_BINS)
    parser.add_argument("--recorded", type=Path, default=DEFAULT_RECORDED)
    parser.add_argument("--raw", type=Path, default=DEFAULT_RAW)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--run-label", default="run1")
    parser.add_argument("--target-angle", type=float, default=120.0)
    parser.add_argument("--tolerance", type=float, default=0.5)
    parser.add_argument("--raw-duration-s", type=float, default=31.761727)
    parser.add_argument("--clip-half-width-ms", type=float, default=150.0)
    parser.add_argument("--source-fps", type=float, default=100.0)
    parser.add_argument("--accumulation-ms", type=float, default=2.0)
    args = parser.parse_args()

    samples = load_samples(args.bins)
    recorded = load_recorded(args.recorded)
    rows = build_rows(
        samples,
        recorded,
        args.target_angle,
        args.tolerance,
        args.run_label,
        args.raw,
        args.bins,
        args.recorded,
        args.raw_duration_s,
        args.clip_half_width_ms / 1000.0,
        args.source_fps,
        args.accumulation_ms,
    )
    if not rows:
        raise SystemExit("No crossings found")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"[DONE] {args.output.resolve()}")
    print(f"[DONE] {len(rows)} occurrences; {sum(row['raw_available'] == 'true' for row in rows)} available in RAW")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
