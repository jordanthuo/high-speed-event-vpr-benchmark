"""Primary CLI for experiment preparation and derived EVK4 media.

Dispatch sequence: parse options, handle preparation/catalog creation, load a
catalog for media commands, then render/query. Pass --catalog and --output-root
before the subcommand. Catalog paths resolve relative to the catalog file;
reference CSVs produced by query-angle contain absolute paths.
Run query-angle before compare-angle. compare-runs-by-angle aligns orientation,
not physical elapsed time. Deterministic output names can be overwritten by a
rerun; retain raw acquisitions and setup files as the reproducible evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import re
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

import cv2
import eventcv

from build_angle_reference_csv import build_rows, load_recorded, load_samples
from make_angle_comparison_gif import make_gif
from make_angle_comparison_video import make_video
from make_angle_synchronised_video import AngleVideoInput, render_angle_synchronised_video
from make_angle_video import (
    label_bin_at_time,
    last_raw_timestamp_us,
    load_bins,
    render_events,
    render_video,
    revolution_time_range,
)
from prepare_experiment import prepare_experiment


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG = ROOT / "vpr_runs.csv"
DEFAULT_OUTPUT_ROOT = ROOT / "vids" / "vpr_outputs"
CATALOG_FIELDS = ("run_label", "recorded_csv", "time_bins_csv", "raw_file", "studio_duration_s")


@dataclass(frozen=True)
class RunConfig:
    label: str
    recorded_csv: Path
    time_bins_csv: Path
    raw_file: Path
    studio_duration_s: float


def resolve_catalog_path(catalog_path: Path, value: str) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (catalog_path.parent / path).resolve()


def load_catalog(path: Path) -> dict[str, RunConfig]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    required = {"run_label", "recorded_csv", "time_bins_csv", "raw_file", "studio_duration_s"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"Run catalogue must contain: {', '.join(sorted(required))}")
    runs: dict[str, RunConfig] = {}
    for row in rows:
        label = row["run_label"].strip()
        if not label or label in runs:
            raise ValueError(f"Run labels must be non-empty and unique: {label!r}")
        run = RunConfig(
            label,
            resolve_catalog_path(path, row["recorded_csv"]),
            resolve_catalog_path(path, row["time_bins_csv"]),
            resolve_catalog_path(path, row["raw_file"]),
            float(row["studio_duration_s"]),
        )
        for source in (run.recorded_csv, run.time_bins_csv, run.raw_file):
            if not source.is_file():
                raise FileNotFoundError(f"{label}: missing {source}")
        runs[label] = run
    return runs


def catalogue_path_value(catalog_path: Path, source: Path) -> str:
    source = source.resolve()
    try:
        return Path(os.path.relpath(source, catalog_path.parent.resolve())).as_posix()
    except ValueError:
        return str(source)


def command_add_run(args: argparse.Namespace, catalog_path: Path) -> None:
    # This writes the minimal five-field catalog. It is not a safe editor for
    # a richer prepare-experiment catalog with extra diagnostic columns; the
    # DictWriter schema below does not preserve those fields.
    recorded = args.recorded.resolve()
    bins = args.time_bins.resolve()
    raw = args.raw.resolve()
    for source in (recorded, bins, raw):
        if not source.is_file():
            raise FileNotFoundError(source)
    if args.studio_duration <= 0:
        raise ValueError("Studio duration must be greater than zero")
    recorded_rows = load_recorded(recorded)
    bin_rows = load_samples(bins)
    if len(recorded_rows) != len(bin_rows):
        raise ValueError(
            f"Recorded CSV has {len(recorded_rows)} rows but time-bin CSV has {len(bin_rows)} rows"
        )
    run_ids = {row["run_id"] for row in recorded_rows}
    if len(run_ids) != 1:
        raise ValueError("Recorded CSV must contain exactly one run_id")
    run_id = next(iter(run_ids))
    if not raw.name.lower().startswith(run_id.lower()):
        raise ValueError(f"RAW filename does not begin with recorded run_id {run_id}")

    existing: list[dict[str, str]] = []
    if catalog_path.is_file():
        with catalog_path.open("r", newline="", encoding="utf-8-sig") as handle:
            existing = list(csv.DictReader(handle))
    matching = [index for index, row in enumerate(existing) if row.get("run_label") == args.label]
    if matching and not args.replace:
        raise ValueError(f"Run label {args.label!r} already exists; pass --replace to update it")
    new_row = {
        "run_label": args.label,
        "recorded_csv": catalogue_path_value(catalog_path, recorded),
        "time_bins_csv": catalogue_path_value(catalog_path, bins),
        "raw_file": catalogue_path_value(catalog_path, raw),
        "studio_duration_s": f"{args.studio_duration:.6f}",
    }
    if matching:
        existing[matching[0]] = new_row
    else:
        existing.append(new_row)
    catalog_path.parent.mkdir(parents=True, exist_ok=True)
    with catalog_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CATALOG_FIELDS)
        writer.writeheader()
        writer.writerows(existing)
    print(f"[DONE] {'Updated' if matching else 'Added'} {args.label}: {catalog_path.resolve()}")
    print(f"[CHECK] {len(recorded_rows)} matched encoder/time-bin rows; run_id={run_id}")


def angle_slug(angle: float) -> str:
    text = f"{angle:g}".replace("-", "m").replace(".", "p")
    return f"{text}deg"


def selected_runs(catalog: dict[str, RunConfig], labels: list[str] | None) -> list[RunConfig]:
    if not labels:
        return list(catalog.values())
    missing = sorted(set(labels).difference(catalog))
    if missing:
        raise ValueError(f"Unknown run label(s): {', '.join(missing)}")
    return [catalog[label] for label in labels]


def parse_instances(values: list[str] | None) -> set[tuple[str, int]] | None:
    if not values:
        return None
    result: set[tuple[str, int]] = set()
    for value in values:
        match = re.fullmatch(r"([^:]+):(\d+)", value)
        if not match:
            raise ValueError(f"Invalid instance {value!r}; use run_label:revolution, for example run1:3")
        result.add((match.group(1), int(match.group(2))))
    return result


def selection_suffix(runs: set[str] | None, instances: set[tuple[str, int]] | None) -> str:
    if instances:
        expanded = "_".join(f"{run}-rev{revolution}" for run, revolution in sorted(instances))
        if len(expanded) <= 64:
            return expanded
        digest = hashlib.sha256(expanded.encode("utf-8")).hexdigest()[:10]
        return f"selected-{len(instances)}-{digest}"
    if runs:
        return "runs-" + "-".join(sorted(runs))
    return "all"


def reference_path(output_root: Path, angle: float) -> Path:
    slug = angle_slug(angle)
    return output_root / "angles" / slug / f"{slug}_reference.csv"


def export_principal_frames(rows: list[dict[str, object]], output_dir: Path) -> None:
    # Quantize onto the requested source-fps grid, then accumulate events over
    # a finite aperture. The PNG is not an instantaneous observation at the
    # exact interpolated crossing time.
    output_dir.mkdir(parents=True, exist_ok=True)
    readers: dict[str, object] = {}
    origins: dict[str, int] = {}
    bins_cache: dict[str, tuple[list, list[float]]] = {}
    for row in rows:
        if row["raw_available"] != "true":
            row["principal_frame"] = ""
            continue
        raw_path = str(row["raw_file"])
        if raw_path not in readers:
            reader = eventcv.open(raw_path, dt_ms=10)
            readers[raw_path] = reader
            origins[raw_path] = round(
                last_raw_timestamp_us(reader) - float(row["raw_recording_duration_s"]) * 1_000_000.0
            )
        bins_path = str(row["bins_file"])
        if bins_path not in bins_cache:
            bins = load_bins(Path(bins_path))
            bins_cache[bins_path] = (bins, [item.start_s for item in bins])
        reader = readers[raw_path]
        width, height = reader.sensor_size
        source_fps = float(row["source_fps"])
        accumulation_s = float(row["accumulation_ms"]) / 1000.0
        frame_time_s = round(float(row["target_time_evk4_s"]) * source_fps) / source_fps
        stream = reader.slice(
            t0_us=origins[raw_path] + round(frame_time_s * 1_000_000.0),
            t1_us=origins[raw_path] + round((frame_time_s + accumulation_s) * 1_000_000.0),
        )
        frame = render_events(stream, width, height)
        bins, starts = bins_cache[bins_path]
        assigned = label_bin_at_time(bins, starts, frame_time_s + accumulation_s / 2.0)
        sample_time_s = (assigned.start_s + assigned.end_s) / 2.0
        speed_text = (
            "n/a" if assigned.platform_speed_rpm is None
            else f"{assigned.platform_speed_rpm:.2f} rpm"
        )
        vertical_text = (
            "n/a" if assigned.vertical_translation_mm is None
            else f"{assigned.vertical_translation_mm:.3f} mm"
        )
        cv2.rectangle(frame, (18, 16), (620, 226), (0, 0, 0), thickness=-1)
        lines = (
            f"{row['run_label']} | Revolution {row['revolution']}",
            f"EVK4 frame: {frame_time_s:.6f} s",
            f"Wrapped angle: {assigned.wrapped_deg:.6f} deg",
            f"Encoder sample: {sample_time_s:.6f} s",
            f"Platform speed: {speed_text}",
            f"Vertical translation: {vertical_text}",
        )
        for index, line in enumerate(lines):
            cv2.putText(
                frame, line, (34, 46 + index * 32), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (255, 255, 255), 2, cv2.LINE_AA,
            )
        filename = (
            f"{row['run_label']}_rev{int(row['revolution']):02d}_"
            f"evk4_{frame_time_s:.6f}s.png"
        )
        destination = output_dir / filename
        if not cv2.imwrite(str(destination), frame):
            raise RuntimeError(f"Could not write {destination}")
        row["principal_frame"] = str(destination.resolve())


def run_query(args: argparse.Namespace, catalog: dict[str, RunConfig], output_root: Path) -> Path:
    # The angle-specific path is deterministic. Repeating a query with a new
    # selection/settings overwrites the reference consumed by compare-angle.
    rows: list[dict[str, object]] = []
    for run in selected_runs(catalog, args.runs):
        rows.extend(
            build_rows(
                load_samples(run.time_bins_csv),
                load_recorded(run.recorded_csv),
                args.angle,
                args.tolerance,
                run.label,
                run.raw_file,
                run.time_bins_csv,
                run.recorded_csv,
                run.studio_duration_s,
                args.window_ms / 2000.0,
                args.source_fps,
                args.accumulation_ms,
            )
        )
    if not rows:
        raise ValueError("No angle crossings were found")
    path = reference_path(output_root, args.angle)
    principal_dir = path.parent / "principal_frames"
    export_principal_frames(rows, principal_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"[DONE] Reference: {path.resolve()}")
    print(f"[DONE] Principal frames: {principal_dir.resolve()}")
    print(f"[DONE] {len(rows)} crossings; {sum(row['raw_available'] == 'true' for row in rows)} RAW-available")
    return path


def command_run_video(args: argparse.Namespace, catalog: dict[str, RunConfig], output_root: Path) -> None:
    run = selected_runs(catalog, [args.run])[0]
    time_selection = args.start_s is not None or args.end_s is not None
    revolution_selection = args.start_rev is not None or args.end_rev is not None
    if time_selection and revolution_selection:
        raise ValueError("Choose a time range or a revolution range, not both")
    suffix = ""
    if time_selection:
        start_s = 0.0 if args.start_s is None else args.start_s
        end_s = run.studio_duration_s if args.end_s is None else args.end_s
        if not all(math.isfinite(value) for value in (start_s, end_s)):
            raise ValueError("Time bounds must be finite")
        if start_s < 0 or end_s <= start_s:
            raise ValueError("Time range requires 0 <= start-s < end-s")
        if end_s > run.studio_duration_s:
            raise ValueError("Selected time range extends beyond the RAW recording duration")
        suffix = f"_time{start_s:g}-{end_s:g}s"
        print(f"[SELECT] camera recording time {start_s:g} to {end_s:g} s")
    else:
        start_s, end_s = revolution_time_range(load_bins(run.time_bins_csv), args.start_rev, args.end_rev)
    if revolution_selection:
        first = str(args.start_rev) if args.start_rev is not None else "start"
        last = str(args.end_rev) if args.end_rev is not None else "end"
        suffix = f"_rev{first}-{last}"
        if end_s is not None and end_s > run.studio_duration_s:
            raise ValueError("Selected revolution range extends beyond the RAW recording duration")
        if start_s is not None and start_s >= run.studio_duration_s:
            raise ValueError("Selected start revolution is outside the RAW recording duration")
        print(f"[SELECT] revolutions {first} through {last}; camera bounds {start_s} to {end_s} s")
    output = output_root / "runs" / run.label / f"{run.label}_angle_overlay{suffix}.mp4"
    render_video(
        run.raw_file, run.time_bins_csv, output, args.accumulation_ms, args.source_fps,
        args.speed, run.studio_duration_s, start_s, end_s,
    )


def command_compare(args: argparse.Namespace, output_root: Path) -> None:
    reference = reference_path(output_root, args.angle)
    if not reference.is_file():
        raise FileNotFoundError(
            f"Reference not found: {reference}. Run 'vpr_media.py query-angle {args.angle}' first."
        )
    runs = set(args.runs) if args.runs else None
    instances = parse_instances(args.instances)
    suffix = selection_suffix(runs, instances)
    if instances:
        selected = " ".join(f"{run}:{revolution}" for run, revolution in sorted(instances))
        print(f"[SELECT] {selected}")
    folder = reference.parent
    if args.format == "video":
        output = folder / f"{angle_slug(args.angle)}_{suffix}_{args.window_ms:g}ms_{args.speed:g}x.mp4"
        make_video(
            reference, output, args.tile_width, args.window_ms, args.speed, 30.0,
            runs, instances,
        )
    else:
        output = folder / f"{angle_slug(args.angle)}_{suffix}_{args.window_ms:g}ms_{args.speed:g}x.gif"
        make_gif(reference, output, args.tile_width, args.speed, args.window_ms, runs, instances)


def command_compare_runs_by_angle(
    args: argparse.Namespace, catalog: dict[str, RunConfig], output_root: Path,
) -> None:
    instances = parse_instances(args.instances)
    if instances is None or not 2 <= len(instances) <= 3:
        raise ValueError("--instances must select two or three run:revolution pairs")
    unknown = sorted({label for label, _ in instances}.difference(catalog))
    if unknown:
        raise ValueError(f"Unknown run label(s): {', '.join(unknown)}")
    ordered = [
        AngleVideoInput(
            label, revolution, catalog[label].raw_file,
            catalog[label].time_bins_csv, catalog[label].studio_duration_s,
        )
        for label, revolution in sorted(instances)
    ]
    suffix = selection_suffix(None, instances)
    if args.start_angle != 0.0 or args.end_angle != 360.0:
        suffix += f"_angle{args.start_angle:g}-{args.end_angle:g}"
    output = output_root / "angle_synchronised" / f"{suffix}_{args.angle_step:g}deg.mp4"
    render_angle_synchronised_video(
        ordered, output, args.angle_step, args.accumulation_ms, args.fps, args.tile_width,
        args.start_angle, args.end_angle,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    subparsers = parser.add_subparsers(dest="command", required=True)

    add_run = subparsers.add_parser("add-run", help="Create or append a validated run-catalogue entry")
    add_run.add_argument("--label", required=True)
    add_run.add_argument("--recorded", type=Path, required=True)
    add_run.add_argument("--time-bins", type=Path, required=True)
    add_run.add_argument("--raw", type=Path, required=True)
    add_run.add_argument("--studio-duration", type=float, required=True)
    add_run.add_argument("--replace", action="store_true", help="Replace an existing row with the same label")

    prepare = subparsers.add_parser(
        "prepare-experiment", help="Discover run files and generate manual/trigger-sync bins and catalogue"
    )
    prepare.add_argument("--folder", type=Path, required=True)
    prepare.add_argument("--motion-threshold-rpm", type=Decimal, default=Decimal("0.5"))
    prepare.add_argument("--stable-samples", type=int, default=3)
    prepare.add_argument("--timing-uncertainty-ms", default="100")

    run_video = subparsers.add_parser("run-video", help="Create an angle-overlay video of a full run or selected revolutions")
    run_video.add_argument("--run", required=True)
    run_video.add_argument("--source-fps", type=float, default=100.0)
    run_video.add_argument("--accumulation-ms", type=float, default=2.0)
    run_video.add_argument("--speed", type=float, default=0.3)
    run_video.add_argument("--start-s", type=float, help="Start time in seconds from camera recording start, not motor T0; cannot combine with revolution bounds")
    run_video.add_argument("--end-s", type=float, help="End time in seconds from camera recording start; cannot combine with revolution bounds")
    run_video.add_argument("--start-rev", type=int, help="First revolution label to include (zero-based)")
    run_video.add_argument("--end-rev", type=int, help="Last revolution label to include, inclusive; must be complete")

    query = subparsers.add_parser("query-angle", help="Index an angle and export one principal frame per instance")
    query.add_argument("angle", type=float)
    query.add_argument("--tolerance", type=float, default=0.5)
    query.add_argument("--runs", nargs="+")
    query.add_argument("--window-ms", type=float, default=30.0)
    query.add_argument("--source-fps", type=float, default=100.0)
    query.add_argument("--accumulation-ms", type=float, default=2.0)

    compare = subparsers.add_parser("compare-angle", help="Create a selected or all-instance GIF/MP4")
    compare.add_argument("angle", type=float)
    compare.add_argument("--format", choices=("video", "gif"), default="video")
    compare.add_argument("--runs", nargs="+")
    compare.add_argument("--instances", nargs="+", help="Selections such as run1:0 run1:3")
    compare.add_argument("--window-ms", type=float, default=30.0)
    compare.add_argument("--speed", type=float, default=0.02)
    compare.add_argument("--tile-width", type=int, default=384)

    angle_compare = subparsers.add_parser(
        "compare-runs-by-angle",
        help="Create a side-by-side video synchronized by wrapped angle",
    )
    angle_compare.add_argument(
        "--instances", nargs="+", required=True,
        help="Two or three selections such as run1:5 run2:5 run3:5",
    )
    angle_compare.add_argument("--angle-step", type=float, default=0.5)
    angle_compare.add_argument("--start-angle", type=float, default=0.0,
                               help="First wrapped angle in degrees (default: 0)")
    angle_compare.add_argument("--end-angle", type=float, default=360.0,
                               help="Exclusive upper wrapped-angle limit in degrees (default: 360)")
    angle_compare.add_argument("--accumulation-ms", type=float, default=2.0)
    angle_compare.add_argument("--fps", type=float, default=30.0)
    angle_compare.add_argument("--tile-width", type=int, default=512)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    catalog_path = args.catalog.resolve()
    output_root = args.output_root.resolve()
    if args.command == "add-run":
        command_add_run(args, catalog_path)
        return 0
    if args.command == "prepare-experiment":
        setup, prepared_catalog, messages = prepare_experiment(
            args.folder, args.motion_threshold_rpm, args.stable_samples, args.timing_uncertainty_ms
        )
        print(f"[DONE] Setup: {setup}")
        print(f"[DONE] Catalogue: {prepared_catalog}")
        for message in messages:
            print(f"[RUN] {message}")
        return 0
    catalog = load_catalog(catalog_path)
    if args.command == "run-video":
        command_run_video(args, catalog, output_root)
    elif args.command == "query-angle":
        run_query(args, catalog, output_root)
    elif args.command == "compare-runs-by-angle":
        command_compare_runs_by_angle(args, catalog, output_root)
    else:
        command_compare(args, output_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
