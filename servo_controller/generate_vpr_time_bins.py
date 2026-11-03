"""Generate one half-open camera interval for each timestamped encoder row.

The standalone CLI supports recording-start or manual-motion anchors.
prepare_experiment also supplies a trigger-affine CameraAnchor from trigger_sync.
Mapping is camera_T0_us + scale * LattePanda_seconds_since_T0. Anchor construction
defines the outer coverage; internal boundaries are adjacent sample midpoints.
Manual coverage extends half an interval beyond sampled data, whereas the
recording-start/trigger workflows can extend labels to recording edges.
An interval label is nearest-sample assignment, not an interpolation or a
claim that the platform stayed at that angle for the full interval.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
from typing import Iterable


REQUIRED_MOTION_COLUMNS = {
    "run_id",
    "lattepanda_time_s",
    "motor_pc_monotonic_ns",
    "motor_speed_command_rpm",
    "recorded_motor_feedback_rpm",
    "encoder_position_pulses",
    "platform_angle_deg",
    "platform_wrapped_angle_deg",
}

OUTPUT_COLUMNS = (
    "run_id",
    "bin_index",
    "camera_start_us",
    "camera_end_us",
    "camera_interval",
    "camera_duration_us",
    "encoder_sample_camera_us",
    "encoder_sample_lattepanda_time_s",
    "encoder_sample_motor_pc_monotonic_ns",
    "encoder_position_pulses",
    "platform_angle_deg",
    "platform_wrapped_angle_deg",
    "motor_speed_command_rpm",
    "recorded_motor_feedback_rpm",
    "recorded_platform_feedback_rpm",
    "lattepanda_monotonic_ns",
    "acquisition_duration_ms",
    "scene_displacement_ain0_v",
    "scene_vertical_displacement_mm",
    "sync_method",
    "sync_anchor_camera_us",
    "sync_anchor_lattepanda_time_s",
    "sync_scale_camera_us_per_lattepanda_s",
    "sync_drift_ppm",
    "sync_fit_rms_us",
    "sync_fit_max_abs_us",
    "sync_match_count",
    "boundary_method",
    "extends_to_recording_edge",
    "timing_uncertainty_ms",
    "angle_uncertainty_deg",
)


@dataclass(frozen=True)
class CameraAnchor:
    camera_start_us: int
    camera_end_us: int
    camera_t0_us: int
    pre_t0_us: int
    sync_method: str = "recording_start_anchor"
    sync_anchor_camera_us: int = 0
    sync_anchor_lattepanda_time_s: str = ""
    scale_camera_us_per_lattepanda_s: Decimal = Decimal("1000000")
    sync_drift_ppm: str = ""
    sync_fit_rms_us: str = ""
    sync_fit_max_abs_us: str = ""
    sync_match_count: int | str = ""


def _round_decimal(value: Decimal) -> int:
    return int(value.to_integral_value(rounding=ROUND_HALF_UP))


def map_lattepanda_time_to_camera_us(anchor: CameraAnchor, lattepanda_time_s: str) -> int:
    return anchor.camera_t0_us + _round_decimal(
        Decimal(lattepanda_time_s) * anchor.scale_camera_us_per_lattepanda_s
    )


def load_camera_anchor(metadata_path: Path, camera_start_us: int) -> tuple[dict, CameraAnchor]:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    required = (
        "run_id",
        "session_start_lattepanda_monotonic_ns",
        "session_end_lattepanda_monotonic_ns",
        "lattepanda_t0_monotonic_ns",
    )
    missing = [name for name in required if metadata.get(name) is None]
    if missing:
        raise ValueError(f"Camera metadata is missing: {', '.join(missing)}")

    session_start_ns = int(metadata["session_start_lattepanda_monotonic_ns"])
    session_end_ns = int(metadata["session_end_lattepanda_monotonic_ns"])
    t0_ns = int(metadata["lattepanda_t0_monotonic_ns"])
    if not session_start_ns <= t0_ns <= session_end_ns:
        raise ValueError("LattePanda T0 must fall inside the camera recording session")

    pre_t0_us = _round_decimal(Decimal(t0_ns - session_start_ns) / Decimal(1000))
    duration_us = _round_decimal(Decimal(session_end_ns - session_start_ns) / Decimal(1000))
    return metadata, CameraAnchor(
        camera_start_us=camera_start_us,
        camera_end_us=camera_start_us + duration_us,
        camera_t0_us=camera_start_us + pre_t0_us,
        pre_t0_us=pre_t0_us,
        sync_anchor_camera_us=camera_start_us,
        sync_anchor_lattepanda_time_s=f"{-pre_t0_us / 1_000_000:.9f}",
    )


def load_motion_rows(motion_path: Path, expected_run_id: str) -> list[dict[str, str]]:
    with motion_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        missing = sorted(REQUIRED_MOTION_COLUMNS - columns)
        if missing:
            raise ValueError(f"Motion CSV is missing columns: {', '.join(missing)}")
        rows = list(reader)

    if not rows:
        raise ValueError("Motion CSV contains no encoder samples")
    run_ids = {row["run_id"] for row in rows}
    if run_ids != {expected_run_id}:
        raise ValueError(
            f"Motion run IDs {sorted(run_ids)!r} do not match camera run ID {expected_run_id!r}"
        )

    previous_time: Decimal | None = None
    for index, row in enumerate(rows):
        current_time = Decimal(row["lattepanda_time_s"])
        if previous_time is not None and current_time <= previous_time:
            raise ValueError(
                f"LattePanda sample times must increase strictly; row {index + 2} is out of order"
            )
        previous_time = current_time
    return rows


def build_motion_start_anchor(
    motion_rows: Iterable[dict[str, str]],
    camera_motion_start_us: int,
    threshold_rpm: Decimal = Decimal("0.5"),
    stable_samples: int = 3,
) -> CameraAnchor:
    """Anchor an EVK4 timestamp to the first sustained encoder motion sample."""
    # The first sample of a sustained above-threshold window is paired with
    # the manually observed camera onset. This is a visual estimate, not an
    # electrical edge alignment; no drift is estimated from a single anchor.
    rows = list(motion_rows)
    if threshold_rpm <= 0:
        raise ValueError("Motion-start threshold must be greater than zero")
    if stable_samples < 1:
        raise ValueError("Motion-start stable sample count must be at least one")
    if len(rows) < max(2, stable_samples):
        raise ValueError("Not enough encoder samples to establish a motion-start anchor")

    anchor_index: int | None = None
    for index in range(0, len(rows) - stable_samples + 1):
        window = rows[index : index + stable_samples]
        if all(
            abs(Decimal(row["recorded_motor_feedback_rpm"])) >= threshold_rpm
            for row in window
        ):
            anchor_index = index
            break
    if anchor_index is None:
        raise ValueError(
            f"No {stable_samples}-sample motion interval reached ±{threshold_rpm} rpm"
        )

    anchor_lattepanda_s = Decimal(rows[anchor_index]["lattepanda_time_s"])
    camera_t0_us = camera_motion_start_us - _round_decimal(
        anchor_lattepanda_s * Decimal(1_000_000)
    )
    mapped = [
        camera_t0_us
        + _round_decimal(Decimal(row["lattepanda_time_s"]) * Decimal(1_000_000))
        for row in rows
    ]
    first_half_interval = max(1, (mapped[1] - mapped[0]) // 2)
    last_half_interval = max(1, (mapped[-1] - mapped[-2]) // 2)
    return CameraAnchor(
        camera_start_us=mapped[0] - first_half_interval,
        camera_end_us=mapped[-1] + last_half_interval,
        camera_t0_us=camera_t0_us,
        pre_t0_us=0,
        sync_method="manual_motion_start_anchor",
        sync_anchor_camera_us=camera_motion_start_us,
        sync_anchor_lattepanda_time_s=f"{anchor_lattepanda_s:.9f}",
    )


def build_time_bins(
    motion_rows: Iterable[dict[str, str]],
    anchor: CameraAnchor,
    timing_uncertainty_ms: str = "",
) -> list[dict[str, object]]:
    # Internal midpoint edges implement nearest-sample assignment. Outer edges
    # come from the chosen anchor and can label pre-roll/tail without nearby
    # measurements. The uncertainty conversion below includes timing only.
    rows = list(motion_rows)
    sample_camera_us = [
        map_lattepanda_time_to_camera_us(anchor, row["lattepanda_time_s"])
        for row in rows
    ]
    if any(later <= earlier for earlier, later in zip(sample_camera_us, sample_camera_us[1:])):
        raise ValueError("Mapped EVK4 encoder-sample times must increase strictly")

    internal_boundaries = [
        (earlier + later) // 2
        for earlier, later in zip(sample_camera_us, sample_camera_us[1:])
    ]
    boundaries = [anchor.camera_start_us, *internal_boundaries, anchor.camera_end_us]
    if any(later <= earlier for earlier, later in zip(boundaries, boundaries[1:])):
        raise ValueError(
            "Camera recording boundaries do not contain the mapped encoder samples; "
            "check --evk4-start-us and the matching metadata file"
        )

    result: list[dict[str, object]] = []
    for index, (row, sample_us) in enumerate(zip(rows, sample_camera_us)):
        start_us = boundaries[index]
        end_us = boundaries[index + 1]
        platform_rpm_text = row.get("recorded_platform_feedback_rpm", "")
        angle_uncertainty = ""
        if timing_uncertainty_ms != "" and platform_rpm_text != "":
            angle_uncertainty = str(
                abs(Decimal(platform_rpm_text))
                * Decimal(6)
                * Decimal(timing_uncertainty_ms)
                / Decimal(1000)
            )
        result.append({
            "run_id": row["run_id"],
            "bin_index": index,
            "camera_start_us": start_us,
            "camera_end_us": end_us,
            "camera_interval": f"[{start_us},{end_us})",
            "camera_duration_us": end_us - start_us,
            "encoder_sample_camera_us": sample_us,
            "encoder_sample_lattepanda_time_s": row["lattepanda_time_s"],
            "encoder_sample_motor_pc_monotonic_ns": row["motor_pc_monotonic_ns"],
            "encoder_position_pulses": row["encoder_position_pulses"],
            "platform_angle_deg": row["platform_angle_deg"],
            "platform_wrapped_angle_deg": row["platform_wrapped_angle_deg"],
            "motor_speed_command_rpm": row["motor_speed_command_rpm"],
            "recorded_motor_feedback_rpm": row["recorded_motor_feedback_rpm"],
            "recorded_platform_feedback_rpm": row.get("recorded_platform_feedback_rpm", ""),
            "lattepanda_monotonic_ns": row.get("lattepanda_monotonic_ns", ""),
            "acquisition_duration_ms": row.get("acquisition_duration_ms", ""),
            "scene_displacement_ain0_v": row.get("scene_displacement_ain0_v", ""),
            "scene_vertical_displacement_mm": row.get("scene_vertical_displacement_mm", ""),
            "sync_method": anchor.sync_method,
            "sync_anchor_camera_us": anchor.sync_anchor_camera_us,
            "sync_anchor_lattepanda_time_s": anchor.sync_anchor_lattepanda_time_s,
            "sync_scale_camera_us_per_lattepanda_s": str(
                anchor.scale_camera_us_per_lattepanda_s
            ),
            "sync_drift_ppm": anchor.sync_drift_ppm,
            "sync_fit_rms_us": anchor.sync_fit_rms_us,
            "sync_fit_max_abs_us": anchor.sync_fit_max_abs_us,
            "sync_match_count": anchor.sync_match_count,
            "boundary_method": "nearest_encoder_sample_midpoints",
            "extends_to_recording_edge": "true" if index in (0, len(rows) - 1) else "false",
            "timing_uncertainty_ms": timing_uncertainty_ms,
            "angle_uncertainty_deg": angle_uncertainty,
        })
    return result


def write_time_bins(path: Path, rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def default_output_path(motion_path: Path) -> Path:
    stem = motion_path.stem
    if stem.endswith("_recorded"):
        stem = stem[: -len("_recorded")]
    return motion_path.with_name(f"{stem}_vpr_time_bins.csv")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate half-open EVK4 event-time bins labelled with the nearest "
            "timestamped encoder angle."
        )
    )
    parser.add_argument("--motion-csv", required=True, type=Path)
    parser.add_argument("--camera-metadata", required=True, type=Path)
    anchor_group = parser.add_mutually_exclusive_group(required=True)
    anchor_group.add_argument(
        "--evk4-start-us",
        type=int,
        help="EVK4 timestamp in microseconds at which RAW recording began",
    )
    anchor_group.add_argument(
        "--evk4-motion-start-us",
        type=int,
        help=(
            "EVK4 timestamp in microseconds at which rotation visibly began; "
            "paired to the first sustained encoder motion sample"
        ),
    )
    parser.add_argument(
        "--motion-start-threshold-rpm",
        default=Decimal("0.5"),
        type=Decimal,
        help="Absolute encoder speed threshold for manual motion anchoring (default: 0.5)",
    )
    parser.add_argument(
        "--motion-start-stable-samples",
        default=3,
        type=int,
        help="Consecutive above-threshold samples required (default: 3)",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--timing-uncertainty-ms",
        default="",
        help="Optional provisional uncertainty copied into every output row",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    metadata = json.loads(args.camera_metadata.read_text(encoding="utf-8"))
    if not metadata.get("run_id"):
        raise ValueError("Camera metadata is missing run_id")
    motion_rows = load_motion_rows(args.motion_csv, str(metadata["run_id"]))
    if args.evk4_start_us is not None:
        metadata, anchor = load_camera_anchor(args.camera_metadata, args.evk4_start_us)
    else:
        anchor = build_motion_start_anchor(
            motion_rows,
            args.evk4_motion_start_us,
            args.motion_start_threshold_rpm,
            args.motion_start_stable_samples,
        )
    bins = build_time_bins(motion_rows, anchor, args.timing_uncertainty_ms)
    output_path = args.output or default_output_path(args.motion_csv)
    write_time_bins(output_path, bins)
    print(f"Created {output_path}")
    print(f"Run ID: {metadata['run_id']}")
    print(f"Bins: {len(bins)}")
    print(f"EVK4 recording: [{anchor.camera_start_us},{anchor.camera_end_us}) us")
    print(f"EVK4 T0 estimate: {anchor.camera_t0_us} us")
    print(f"Synchronization: {anchor.sync_method} (provisional)")
    print(
        f"Anchor: EVK4 {anchor.sync_anchor_camera_us} us = "
        f"LattePanda {anchor.sync_anchor_lattepanda_time_s} s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
