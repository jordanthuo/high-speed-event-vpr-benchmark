"""Render a GIF grid around selected angle crossings from a reference CSV.

Rows come from build_angle_reference_csv or vpr_media query-angle. Each tile
uses its own recording-relative crossing time, RAW origin and encoder bins.
The shared comparison timeline is relative to the crossing, not an absolute
clock across experiments. GIF delay quantization and event accumulation limit
visual timing precision; preserve the reference CSV for quantitative work.
Defaults reference old acquisitions; use the main vpr_media CLI for new runs.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import eventcv
from PIL import Image

from make_angle_video import label_bin_at_time, last_raw_timestamp_us, load_bins, render_events


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REFERENCE = ROOT / "Speed Profiles" / "50-rpm_120deg_reference_with_recorded_angles.csv"
DEFAULT_OUTPUT = ROOT / "vids" / "recordings" / "120deg_all_instances_synchronised.gif"


def load_available_rows(
    path: Path,
    selected_runs: set[str] | None = None,
    selected_instances: set[tuple[str, int]] | None = None,
) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        rows = [row for row in csv.DictReader(handle) if row["raw_available"].lower() == "true"]
    if selected_runs:
        rows = [row for row in rows if row["run_label"] in selected_runs]
    if selected_instances:
        rows = [
            row
            for row in rows
            if (row["run_label"], int(row["revolution"])) in selected_instances
        ]
    if not rows:
        raise ValueError("Reference CSV has no RAW-available occurrences")
    return rows


def gif_durations_ms(frame_count: int, source_fps: float, playback_speed: float) -> list[int]:
    """Distribute integer GIF delays without accumulating timing drift."""
    # Round cumulative ideal deadlines, then difference them, so per-frame
    # rounding does not accumulate additional total-duration error.
    exact_ms = 1000.0 / (source_fps * playback_speed)
    return [round((index + 1) * exact_ms) - round(index * exact_ms) for index in range(frame_count)]


def draw_tile_footer(
    canvas, x0: int, text_y0: int, row: dict[str, str], window_start_s: float,
    assigned_bin, sample_time_s: float,
) -> None:
    speed_text = (
        "n/a" if assigned_bin.platform_speed_rpm is None
        else f"{assigned_bin.platform_speed_rpm:.2f} rpm"
    )
    vertical_text = (
        "n/a" if assigned_bin.vertical_translation_mm is None
        else f"{assigned_bin.vertical_translation_mm:.3f} mm"
    )
    lines = (
        (
            f"{row['run_label']} rev {row['revolution']} | EVK4 {window_start_s:.6f} s",
            0.43,
        ),
        (
            f"angle {assigned_bin.wrapped_deg:.6f} deg | sample {sample_time_s:.6f} s",
            0.41,
        ),
        (f"speed {speed_text} | vertical {vertical_text}", 0.40),
    )
    for index, (text, scale) in enumerate(lines):
        cv2.putText(
            canvas, text, (x0 + 7, text_y0 + 21 + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA,
        )


def make_gif(
    reference: Path,
    output: Path,
    tile_width: int = 384,
    playback_speed: float = 0.1,
    window_ms: float = 30.0,
    selected_runs: set[str] | None = None,
    selected_instances: set[tuple[str, int]] | None = None,
) -> None:
    if playback_speed <= 0:
        raise ValueError("Playback speed must be greater than zero")
    if window_ms <= 0:
        raise ValueError("Window must be greater than zero")
    rows = load_available_rows(reference, selected_runs, selected_instances)
    source_fps = float(rows[0]["source_fps"])
    accumulation_s = float(rows[0]["accumulation_ms"]) / 1000.0
    query_angle = float(rows[0]["query_angle_deg"])
    frame_count = max(1, round(window_ms / 1000.0 * source_fps))
    half_index = frame_count // 2
    relative_times = [(index - half_index) / source_fps for index in range(frame_count)]

    readers: dict[str, object] = {}
    origins_us: dict[str, int] = {}
    angle_bins: dict[str, tuple[list, list[float]]] = {}
    for row in rows:
        raw_path = row["raw_file"]
        if raw_path in readers:
            continue
        reader = eventcv.open(raw_path, dt_ms=10)
        readers[raw_path] = reader
        duration_s = float(row["raw_recording_duration_s"])
        origins_us[raw_path] = round(last_raw_timestamp_us(reader) - duration_s * 1_000_000.0)
        bins_path = row["bins_file"]
        if bins_path not in angle_bins:
            bins = load_bins(Path(bins_path))
            angle_bins[bins_path] = (bins, [item.start_s for item in bins])

    first_reader = next(iter(readers.values()))
    sensor_width, sensor_height = first_reader.sensor_size
    tile_height = round(tile_width * sensor_height / sensor_width)
    title_height = 82
    header_height = 54
    columns = min(4, len(rows))
    grid_rows = math.ceil(len(rows) / columns)
    canvas_width = columns * tile_width
    canvas_height = header_height + grid_rows * (tile_height + title_height)

    frames: list[Image.Image] = []
    for frame_index, relative_s in enumerate(relative_times):
        relative_ms = 0.0 if abs(relative_s) < 0.0005 else relative_s * 1000.0
        canvas = cv2.UMat(canvas_height, canvas_width, cv2.CV_8UC3).get()
        canvas[:] = 0
        cv2.putText(
            canvas,
            f"Target {query_angle:.1f} deg   Relative time {relative_ms:+.0f} ms",
            (18, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )
        for tile_index, row in enumerate(rows):
            column = tile_index % columns
            grid_row = tile_index // columns
            x0 = column * tile_width
            y0 = header_height + grid_row * (tile_height + title_height)
            target_s = float(row["target_time_evk4_s"])
            aligned_target_frame_s = round(target_s * source_fps) / source_fps
            window_start_s = aligned_target_frame_s + relative_s
            reader = readers[row["raw_file"]]
            origin_us = origins_us[row["raw_file"]]
            stream = reader.slice(
                t0_us=origin_us + round(window_start_s * 1_000_000.0),
                t1_us=origin_us + round((window_start_s + accumulation_s) * 1_000_000.0),
            )
            sensor_frame = render_events(stream, sensor_width, sensor_height)
            tile = cv2.resize(sensor_frame, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
            canvas[y0 : y0 + tile_height, x0 : x0 + tile_width] = tile
            bins, starts = angle_bins[row["bins_file"]]
            assigned_bin = label_bin_at_time(bins, starts, window_start_s + accumulation_s / 2.0)
            sample_time_s = (assigned_bin.start_s + assigned_bin.end_s) / 2.0
            draw_tile_footer(
                canvas, x0, y0 + tile_height, row, window_start_s,
                assigned_bin, sample_time_s,
            )
        frames.append(Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)))
        print(f"[GIF] frame {frame_index + 1}/{frame_count} ({relative_ms:+.0f} ms)", flush=True)

    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=gif_durations_ms(len(frames), source_fps, playback_speed),
        loop=0,
        disposal=2,
        optimize=False,
    )
    print(f"[DONE] {output.resolve()}", flush=True)
    print(
        f"[DONE] {len(rows)} synchronized instances; {len(frames)} frames; "
        f"2 ms accumulation; {playback_speed:g}x playback",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--tile-width", type=int, default=384)
    parser.add_argument("--playback-speed", type=float, default=0.1)
    parser.add_argument("--window-ms", type=float, default=30.0)
    args = parser.parse_args()
    make_gif(args.reference.resolve(), args.output.resolve(), args.tile_width, args.playback_speed, args.window_ms)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
