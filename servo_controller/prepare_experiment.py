"""Discover, organize and derive a reproducible experiment dataset.

Called by vpr_media.py prepare-experiment; this module has no standalone CLI.
Runs join by recorded run_id and UUID-prefixed camera filenames, not by human
labels or wall-clock proximity. Setup paths are experiment-relative; catalog
paths are relative to outputs/, where the catalog is written.

This workflow mutates the filesystem: it writes setup/catalog files, moves
acquisitions and may update copied camera metadata after trigger extraction.
Organization happens before include filtering and is not transactional.
Use a backed-up experiment; a failure can leave some files already moved.
Missing manual inputs are reported per run; invalid trigger fitting raises
rather than quietly substituting a manual synchronization method.
"""

from __future__ import annotations

import csv
import json
import os
import re
import shutil
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

import eventcv
from evt3_trigger_extract import extract_evt3_trigger_events, update_camera_metadata

from generate_vpr_time_bins import (
    build_motion_start_anchor,
    build_time_bins,
    load_motion_rows,
    write_time_bins,
)
from make_angle_video import last_raw_timestamp_us
from trigger_sync import (
    build_trigger_anchor,
    fit_trigger_sync,
    load_trigger_events,
    load_uart_writes,
    write_trigger_diagnostics,
)


SETUP_FIELDS = (
    "include",
    "run_label",
    "sync_mode",
    "run_id",
    "recorded_csv",
    "raw_file",
    "camera_metadata",
    "uart_writes_csv",
    "trigger_events_csv",
    "manual_evk4_motion_s",
    "studio_duration_s",
    "nominal_speed_rpm",
    "vertical_condition",
    "notes",
)


@dataclass(frozen=True)
class DiscoveredRun:
    run_id: str
    recorded_csv: Path
    raw_file: Path
    camera_metadata: Path
    uart_writes_csv: Path | None
    trigger_events_csv: Path | None


def relative_to_experiment(path: Path, experiment: Path) -> str:
    return path.resolve().relative_to(experiment.resolve()).as_posix()


def discover_runs(experiment: Path) -> list[DiscoveredRun]:
    results: list[DiscoveredRun] = []
    for recorded in sorted(experiment.rglob("*_recorded.csv")):
        with recorded.open("r", newline="", encoding="utf-8-sig") as handle:
            first = next(csv.DictReader(handle), None)
        if not first or not first.get("run_id"):
            continue
        run_id = first["run_id"]
        raws = list(experiment.rglob(f"{run_id}_*.raw"))
        metadata = [
            path for path in experiment.rglob(f"{run_id}_*_metadata.json")
            if not path.name.endswith("_recorded_metadata.json")
        ]
        writes = list(experiment.rglob(f"{run_id}_*_0x00_writes.csv"))
        trigger_events = list(experiment.rglob(f"{run_id}_*_trigger_events.csv"))
        if len(raws) != 1 or len(metadata) != 1:
            raise ValueError(
                f"{recorded.name}: expected one RAW and one camera metadata file for {run_id}; "
                f"found {len(raws)} RAW and {len(metadata)} metadata"
            )
        camera_meta = json.loads(metadata[0].read_text(encoding="utf-8"))
        if camera_meta.get("run_id") != run_id:
            raise ValueError(f"Camera metadata run_id does not match {recorded.name}")
        if len(writes) > 1 or len(trigger_events) > 1:
            raise ValueError(
                f"{recorded.name}: expected at most one UART-write and one Trigger In CSV; "
                f"found {len(writes)} writes and {len(trigger_events)} trigger files"
            )
        results.append(DiscoveredRun(
            run_id, recorded, raws[0], metadata[0],
            writes[0] if writes else None,
            trigger_events[0] if trigger_events else None,
        ))
    if not results:
        raise ValueError(f"No *_recorded.csv run exports found below {experiment}")
    return results


def infer_nominal_speed(path: Path) -> str:
    match = re.match(r"([+-]?\d+(?:\.\d+)?)-rpm", path.name, re.IGNORECASE)
    return match.group(1) if match else ""


def read_setup(path: Path) -> dict[str, dict[str, str]]:
    if not path.is_file():
        return {}
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        return {row["run_id"]: row for row in csv.DictReader(handle)}


def write_setup(experiment: Path, discovered: list[DiscoveredRun]) -> Path:
    path = experiment / "experiment_setup.csv"
    previous = read_setup(path)
    rows: list[dict[str, str]] = []
    for index, run in enumerate(discovered, start=1):
        old = previous.get(run.run_id, {})
        rows.append({
            "include": old.get("include", "true"),
            "run_label": old.get("run_label", f"run{index}"),
            "sync_mode": old.get("sync_mode", "manual"),
            "run_id": run.run_id,
            "recorded_csv": relative_to_experiment(run.recorded_csv, experiment),
            "raw_file": relative_to_experiment(run.raw_file, experiment),
            "camera_metadata": relative_to_experiment(run.camera_metadata, experiment),
            "uart_writes_csv": relative_to_experiment(run.uart_writes_csv, experiment) if run.uart_writes_csv else "",
            "trigger_events_csv": (
                relative_to_experiment(run.trigger_events_csv, experiment)
                if run.trigger_events_csv else ""
            ),
            "manual_evk4_motion_s": old.get("manual_evk4_motion_s", ""),
            "studio_duration_s": old.get("studio_duration_s", ""),
            "nominal_speed_rpm": old.get("nominal_speed_rpm", infer_nominal_speed(run.recorded_csv)),
            "vertical_condition": old.get("vertical_condition", ""),
            "notes": old.get("notes", ""),
        })
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SETUP_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def run_folder_name(label: str) -> str:
    """Return a Windows-safe folder name while retaining a readable run label."""
    safe = re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", label.strip()).rstrip(". ")
    if not safe or safe in {".", ".."}:
        raise ValueError(f"Run label cannot be used as a folder name: {label!r}")
    return safe


def move_without_overwrite(source: Path, destination: Path) -> Path:
    source = source.resolve()
    destination = destination.resolve()
    if source == destination:
        return destination
    if destination.exists():
        raise FileExistsError(
            f"Cannot organize {source.name}: destination already exists: {destination}"
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(source), str(destination))
    return destination


def organize_runs(
    experiment: Path,
    discovered: list[DiscoveredRun],
    setup: dict[str, dict[str, str]],
) -> list[DiscoveredRun]:
    """Move RAW files to raw/ and each run's other artifacts to runs/<label>/."""
    # Moves are sequential and include excluded runs. A later error does not
    # roll back earlier moves; repeated preparation should use updated paths.
    organized: list[DiscoveredRun] = []
    folder_owners: dict[str, str] = {}
    for run in discovered:
        label = setup[run.run_id]["run_label"].strip()
        folder = run_folder_name(label)
        folded = folder.casefold()
        owner = folder_owners.setdefault(folded, run.run_id)
        if owner != run.run_id:
            raise ValueError(
                f"Run labels must map to unique folders; {label!r} conflicts with another run"
            )

        raw_path = move_without_overwrite(run.raw_file, experiment / "raw" / run.raw_file.name)
        run_dir = experiment / "runs" / folder

        # Motor exports share the '<profile>_<timestamp>_recorded' stem. Keep
        # the motion CSV, clock sync, event log and recorded metadata together.
        motor_files = sorted(experiment.rglob(f"{run.recorded_csv.stem}*"))
        for source in motor_files:
            if source.is_file() and source.resolve() != raw_path:
                move_without_overwrite(source, run_dir / source.name)

        # Camera-side auxiliary files are UUID-prefixed. The RAW itself has
        # already moved to the common raw/ directory.
        camera_files = sorted(experiment.rglob(f"{run.run_id}_*"))
        for source in camera_files:
            if source.is_file() and source.suffix.lower() != ".raw":
                move_without_overwrite(source, run_dir / source.name)

        organized.append(DiscoveredRun(
            run.run_id,
            run_dir / run.recorded_csv.name,
            raw_path,
            run_dir / run.camera_metadata.name,
            run_dir / run.uart_writes_csv.name if run.uart_writes_csv else None,
            run_dir / run.trigger_events_csv.name if run.trigger_events_csv else None,
        ))
    return organized


def raw_studio_origin_us(raw_path: Path, studio_duration_s: float) -> int:
    # Do not use the first contrast event as recording zero: quiet pre-roll
    # would disappear. This estimate depends on the supplied Studio duration.
    reader = eventcv.open(str(raw_path), dt_ms=10)
    return round(last_raw_timestamp_us(reader) - studio_duration_s * 1_000_000.0)


def prepare_experiment(
    experiment: Path,
    threshold_rpm: Decimal = Decimal("0.5"),
    stable_samples: int = 3,
    timing_uncertainty_ms: str = "100",
) -> tuple[Path, Path, list[str]]:
    # Setup is both operator input and generated manifest. Preserve custom
    # fields by run_id, then regenerate paths after organizing. Ready catalog
    # rows are a subset of discovered runs, so inspect the returned messages.
    experiment = experiment.resolve()
    discovered = discover_runs(experiment)
    setup_path = write_setup(experiment, discovered)
    setup = read_setup(setup_path)
    discovered = organize_runs(experiment, discovered, setup)
    setup_path = write_setup(experiment, discovered)
    setup = read_setup(setup_path)
    output_root = experiment / "outputs"
    bins_dir = output_root / "time_bins"
    catalog_path = output_root / "vpr_runs.csv"
    catalog_rows: list[dict[str, str]] = []
    messages: list[str] = []

    for run in discovered:
        row = setup[run.run_id]
        if row["include"].strip().lower() not in {"true", "1", "yes", "y"}:
            messages.append(f"{row['run_label']}: excluded")
            continue
        motion_rows = load_motion_rows(run.recorded_csv, run.run_id)
        mode = row.get("sync_mode", "manual").strip().lower()
        diagnostics_path: Path | None = None
        fit = None
        applied_uncertainty_ms = timing_uncertainty_ms
        if mode == "manual":
            if not row["manual_evk4_motion_s"].strip():
                messages.append(f"{row['run_label']}: needs manual_evk4_motion_s")
                continue
            motion_us = round(float(row["manual_evk4_motion_s"]) * 1_000_000.0)
            anchor = build_motion_start_anchor(
                motion_rows, motion_us, threshold_rpm, stable_samples,
            )
        elif mode == "trigger":
            missing = []
            if run.uart_writes_csv is None:
                missing.append("uart_writes_csv")
            if run.trigger_events_csv is None:
                trigger_path = run.camera_metadata.parent / f"{run.raw_file.stem}_trigger_events.csv"
                messages.append(f"{row['run_label']}: extracting Trigger In edges on motor PC")
                count = extract_evt3_trigger_events(run.raw_file, trigger_path)
                update_camera_metadata(run.camera_metadata, trigger_path, count)
                run = DiscoveredRun(
                    run.run_id, run.recorded_csv, run.raw_file, run.camera_metadata,
                    run.uart_writes_csv, trigger_path,
                )
                row["trigger_events_csv"] = relative_to_experiment(trigger_path, experiment)
            if not row["studio_duration_s"].strip():
                missing.append("studio_duration_s")
            if missing:
                messages.append(
                    f"{row['run_label']}: trigger sync needs {', '.join(missing)}"
                )
                continue
            studio_duration_s = float(row["studio_duration_s"])
            raw_origin_us = raw_studio_origin_us(run.raw_file, studio_duration_s)
            writes = load_uart_writes(run.uart_writes_csv, run.camera_metadata)
            events = load_trigger_events(run.trigger_events_csv, polarity=0, channel_id=0)
            fit = fit_trigger_sync(writes, events, raw_origin_us)
            anchor = build_trigger_anchor(fit, studio_duration_s)
            applied_uncertainty_ms = f"{fit.timing_uncertainty_us / 1000.0:.6f}"
            diagnostics_path = output_root / "trigger_sync" / f"{row['run_label']}_trigger_sync.csv"
            write_trigger_diagnostics(diagnostics_path, fit)
        else:
            raise ValueError(
                f"{row['run_label']}: sync_mode must be 'manual' or 'trigger', got {mode!r}"
            )

        bins = build_time_bins(motion_rows, anchor, applied_uncertainty_ms)
        bins_path = bins_dir / f"{row['run_label']}_time_bins.csv"
        write_time_bins(bins_path, bins)
        if not row["studio_duration_s"].strip():
            messages.append(
                f"{row['run_label']}: bins created; needs studio_duration_s before RAW video/query use"
            )
            continue
        catalog_rows.append({
            "run_label": row["run_label"],
            "recorded_csv": Path(os.path.relpath(run.recorded_csv.resolve(), output_root.resolve())).as_posix(),
            "time_bins_csv": Path(os.path.relpath(bins_path.resolve(), output_root.resolve())).as_posix(),
            "raw_file": Path(os.path.relpath(run.raw_file.resolve(), output_root.resolve())).as_posix(),
            "studio_duration_s": f"{float(row['studio_duration_s']):.6f}",
            "nominal_speed_rpm": row["nominal_speed_rpm"],
            "vertical_condition": row["vertical_condition"],
            "sync_mode": mode,
            "sync_method": anchor.sync_method,
            "manual_evk4_motion_s": (
                f"{float(row['manual_evk4_motion_s']):.6f}" if mode == "manual" else ""
            ),
            "encoder_motion_lattepanda_s": (
                anchor.sync_anchor_lattepanda_time_s if mode == "manual" else ""
            ),
            "trigger_events_csv": (
                Path(os.path.relpath(run.trigger_events_csv.resolve(), output_root.resolve())).as_posix()
                if run.trigger_events_csv else ""
            ),
            "trigger_sync_diagnostics_csv": (
                Path(os.path.relpath(diagnostics_path.resolve(), output_root.resolve())).as_posix()
                if diagnostics_path else ""
            ),
            "trigger_match_count": "" if fit is None else str(len(fit.matches)),
            "trigger_fit_rms_us": "" if fit is None else f"{fit.rms_residual_us:.3f}",
            "trigger_fit_max_abs_us": "" if fit is None else f"{fit.max_abs_residual_us:.3f}",
            "trigger_clock_drift_ppm": "" if fit is None else f"{fit.drift_ppm:.6f}",
            "timing_uncertainty_ms": applied_uncertainty_ms,
        })
        if mode == "manual":
            messages.append(
                f"{row['run_label']}: ready (manual); "
                f"EVK4 {float(row['manual_evk4_motion_s']):.6f}s = "
                f"LattePanda {anchor.sync_anchor_lattepanda_time_s}s"
            )
        else:
            messages.append(
                f"{row['run_label']}: ready (trigger); {len(fit.matches)} matched edges, "
                f"RMS {fit.rms_residual_us:.1f} us, drift {fit.drift_ppm:.3f} ppm"
            )

    catalog_fields = (
        "run_label", "recorded_csv", "time_bins_csv", "raw_file", "studio_duration_s",
        "nominal_speed_rpm", "vertical_condition", "sync_mode", "sync_method",
        "manual_evk4_motion_s", "encoder_motion_lattepanda_s", "trigger_events_csv",
        "trigger_sync_diagnostics_csv", "trigger_match_count", "trigger_fit_rms_us",
        "trigger_fit_max_abs_us", "trigger_clock_drift_ppm", "timing_uncertainty_ms",
    )
    # Persist any locally generated Trigger In path so the setup remains a
    # complete, reproducible description of the next preparation run.
    with setup_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SETUP_FIELDS)
        writer.writeheader()
        writer.writerows(setup.values())
    output_root.mkdir(parents=True, exist_ok=True)
    with catalog_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=catalog_fields)
        writer.writeheader()
        writer.writerows(catalog_rows)
    return setup_path, catalog_path, messages
