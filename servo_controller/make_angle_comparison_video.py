"""Encode an MP4 grid around reference-CSV angle crossings.

Shares row filtering and tile annotations with the GIF renderer. Selected
run/revolution rows provide source times and RAW durations; output FPS and
playback speed control the presentation timeline. This is a crossing-centered
time comparison, unlike make_angle_synchronised_video's orientation sweep.
Output is derived media, not a lossless replacement for RAW or encoder CSVs.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import eventcv
import numpy as np

from make_angle_comparison_gif import draw_tile_footer, load_available_rows
from make_angle_video import label_bin_at_time, last_raw_timestamp_us, load_bins, render_events


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REFERENCE = ROOT / "Speed Profiles" / "50-rpm_120deg_reference_with_recorded_angles.csv"
DEFAULT_OUTPUT = ROOT / "vids" / "recordings" / "120deg_all_instances_30ms_0.02x.mp4"


def make_video(
    reference: Path,
    output: Path,
    tile_width: int = 384,
    window_ms: float = 30.0,
    playback_speed: float = 0.02,
    output_fps: float = 30.0,
    selected_runs: set[str] | None = None,
    selected_instances: set[tuple[str, int]] | None = None,
) -> None:
    if window_ms <= 0 or playback_speed <= 0 or output_fps <= 0:
        raise ValueError("Window, playback speed and output FPS must be positive")
    rows = load_available_rows(reference, selected_runs, selected_instances)
    source_fps = float(rows[0]["source_fps"])
    accumulation_s = float(rows[0]["accumulation_ms"]) / 1000.0
    query_angle = float(rows[0]["query_angle_deg"])
    unique_frame_count = max(1, round(window_ms / 1000.0 * source_fps))
    half_index = unique_frame_count // 2
    relative_times = [(index - half_index) / source_fps for index in range(unique_frame_count)]
    repeats_per_source_frame = max(1, round(output_fps / (source_fps * playback_speed)))

    readers: dict[str, object] = {}
    origins_us: dict[str, int] = {}
    angle_bins: dict[str, tuple[list, list[float]]] = {}
    for row in rows:
        raw_path = row["raw_file"]
        if raw_path not in readers:
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

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*"mp4v"), output_fps, (canvas_width, canvas_height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not create video: {output}")
    try:
        for frame_index, relative_s in enumerate(relative_times):
            relative_ms = relative_s * 1000.0
            canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
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
            for _ in range(repeats_per_source_frame):
                writer.write(canvas)
            print(
                f"[VIDEO] source frame {frame_index + 1}/{unique_frame_count} "
                f"({relative_ms:+.0f} ms), repeated {repeats_per_source_frame} times",
                flush=True,
            )
    finally:
        writer.release()
    total_frames = unique_frame_count * repeats_per_source_frame
    print(f"[DONE] {output.resolve()}", flush=True)
    print(
        f"[DONE] {len(rows)} instances; {window_ms:g} ms window; {playback_speed:g}x; "
        f"{total_frames} frames at {output_fps:g} fps",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--tile-width", type=int, default=384)
    parser.add_argument("--window-ms", type=float, default=30.0)
    parser.add_argument("--playback-speed", type=float, default=0.02)
    parser.add_argument("--output-fps", type=float, default=30.0)
    args = parser.parse_args()
    make_video(
        args.reference.resolve(),
        args.output.resolve(),
        args.tile_width,
        args.window_ms,
        args.playback_speed,
        args.output_fps,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
