"""Reconstruct a full or clipped RAW recording with encoder-bin overlays.

Shared rendering/label helpers are imported by all comparison tools. Event
timestamps are native RAW microseconds; overlay/bin times are recording-relative
seconds in the prepared workflow. RAW origin is inferred from the last event
timestamp and Studio duration, retaining silent recording lead-in.
source_fps sets a reconstruction grid; accumulation_ms is an event aperture,
not a sensor frame period. Playback scaling only changes presentation time.
Historical CLI defaults must be overridden for a new experiment.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import eventcv
import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RAW = WORKSPACE_ROOT / "vids" / "recordings" / "73d57ea887144bf7a54729d1b13c34ba_20261007_145323.raw"
DEFAULT_BINS = WORKSPACE_ROOT / "Speed Profiles" / "50-rpm_run1_EVK4_time_bins.csv"
DEFAULT_OUTPUT = WORKSPACE_ROOT / "vids" / "recordings" / "50-rpm_run1_angle_overlay_studio_timing.mp4"
DEFAULT_ACCUMULATION_MS = 2.0
DEFAULT_SOURCE_FPS = 100.0
DEFAULT_PLAYBACK_SPEED = 0.3
DEFAULT_STUDIO_DURATION_S = 31.761727


@dataclass(frozen=True)
class AngleBin:
    start_s: float
    end_s: float
    unwrapped_deg: float
    wrapped_deg: float
    platform_speed_rpm: float | None = None
    vertical_translation_mm: float | None = None


def optional_float(row: dict[str, str], column: str) -> float | None:
    value = row.get(column, "").strip()
    return None if value == "" else float(value)


def load_bins(path: Path) -> list[AngleBin]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or ())
        seconds_schema = {"evk4_bin_start_s", "evk4_bin_end_s"}.issubset(fields)
        microseconds_schema = {"camera_start_us", "camera_end_us"}.issubset(fields)
        required = {"platform_angle_deg", "platform_wrapped_angle_deg"}
        missing = required.difference(fields)
        if missing:
            raise ValueError(f"Bin CSV is missing columns: {', '.join(sorted(missing))}")
        if not seconds_schema and not microseconds_schema:
            raise ValueError("Bin CSV needs EVK4 second columns or camera microsecond columns")
        rows = [
            AngleBin(
                float(row["evk4_bin_start_s"]) if seconds_schema else float(row["camera_start_us"]) / 1e6,
                float(row["evk4_bin_end_s"]) if seconds_schema else float(row["camera_end_us"]) / 1e6,
                float(row["platform_angle_deg"]),
                float(row["platform_wrapped_angle_deg"]) % 360.0,
                optional_float(row, "recorded_platform_feedback_rpm"),
                optional_float(row, "scene_vertical_displacement_mm"),
            )
            for row in reader
        ]
    if not rows:
        raise ValueError("Bin CSV has no data rows")
    if any(row.end_s <= row.start_s for row in rows):
        raise ValueError("Every time bin must have end > start")
    if any(b.start_s < a.start_s for a, b in zip(rows, rows[1:])):
        raise ValueError("Time bins must be sorted by start time")
    return rows


def bin_at_time(bins: list[AngleBin], starts: list[float], time_s: float) -> AngleBin | None:
    index = bisect.bisect_right(starts, time_s) - 1
    if index < 0:
        return None
    row = bins[index]
    return row if row.start_s <= time_s < row.end_s else None


def label_bin_at_time(bins: list[AngleBin], starts: list[float], time_s: float) -> AngleBin:
    """Return a label throughout the recording, holding the nearest edge value.

    The EVK4 preroll begins before the first mapped encoder bin. The mechanism
    is stationary then, so the first measured encoder angle is held backwards
    to camera time zero. The final value is similarly held after the last bin.
    """
    index = bisect.bisect_right(starts, time_s) - 1
    if index < 0:
        return bins[0]
    return bins[min(index, len(bins) - 1)]


def interpolated_angle_at_time(bins: list[AngleBin], starts: list[float], time_s: float) -> AngleBin:
    """Interpolate overlay angle consistently with revolution clip boundaries."""
    row = label_bin_at_time(bins, starts, time_s)
    index = bisect.bisect_right(starts, time_s) - 1
    if 0 <= index < len(bins) - 1:
        following = bins[index + 1]
        span = following.start_s - row.start_s
        if span > 0:
            angle = row.unwrapped_deg + (following.unwrapped_deg - row.unwrapped_deg) * (time_s - row.start_s) / span
            return replace(row, unwrapped_deg=angle, wrapped_deg=angle % 360.0)
    return row


def revolution_count(unwrapped_deg: float, origin_deg: float) -> int:
    """Completed revolutions since the first encoder bin (starting at zero)."""
    return max(0, math.floor(max(0.0, unwrapped_deg - origin_deg) / 360.0 + 1e-9))


def revolution_time_range(
    bins: list[AngleBin], start_rev: int | None, end_rev: int | None,
) -> tuple[float | None, float | None]:
    """Map inclusive forward revolution labels to recording-relative bounds.

    End revolution N ends at the crossing into N+1, which must be recorded.
    Linear interpolation uses the same bin-start angle samples as the overlays.
    """
    if start_rev is None and end_rev is None:
        return None, None
    if not bins:
        raise ValueError("Revolution selection requires encoder time bins")
    if any(value is not None and value < 0 for value in (start_rev, end_rev)):
        raise ValueError("Revolution numbers must be non-negative")
    if start_rev is not None and end_rev is not None and end_rev < start_rev:
        raise ValueError("End revolution must be greater than or equal to start revolution")
    origin = bins[0].unwrapped_deg

    def crossing(revolution: int) -> float:
        if revolution == 0:
            return bins[0].start_s
        target = origin + revolution * 360.0
        for left, right in zip(bins, bins[1:]):
            if left.unwrapped_deg < target <= right.unwrapped_deg:
                fraction = (target - left.unwrapped_deg) / (right.unwrapped_deg - left.unwrapped_deg)
                return left.start_s + fraction * (right.start_s - left.start_s)
        raise ValueError(
            f"Recording does not reach the start of revolution {revolution}; "
            "choose a range of complete recorded revolutions"
        )

    start = crossing(start_rev) if start_rev is not None else None
    end = crossing(end_rev + 1) if end_rev is not None else None
    lower = bins[0].start_s if start is None else start
    upper = bins[-1].end_s if end is None else end
    selected = [row for row in bins if row.end_s > lower and row.start_s < upper]
    if any(right.unwrapped_deg < left.unwrapped_deg - 1e-6 for left, right in zip(selected, selected[1:])):
        raise ValueError("Revolution range selection requires forward motion without reversals")
    return start, end


def first_raw_timestamp_us(reader) -> int:
    for index in range(min(100, reader.n_slices)):
        events = reader.slice(index).numpy()
        if len(events):
            # EventCV arrays are [x, y, timestamp_us, polarity].
            return int(events[0, 2])
    raise ValueError("RAW file contains no events in its first 100 slices")


def last_raw_timestamp_us(reader) -> int:
    for index in range(reader.n_slices - 1, max(-1, reader.n_slices - 101), -1):
        events = reader.slice(index).numpy()
        if len(events):
            return int(events[-1, 2])
    raise ValueError("RAW file contains no events in its last 100 slices")


def render_events(stream, width: int, height: int) -> np.ndarray:
    polarity = stream.flatten("polarity").numpy()
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    if polarity.shape[0] >= 2:
        # EventCV planes are [positive/ON, negative/OFF], not raw p=0/p=1 order.
        frame[polarity[0] > 0] = (255, 255, 255)  # ON: white
        frame[polarity[1] > 0] = (255, 100, 0)  # OFF: blue (BGR); wins on overlap
    return frame


def add_overlay(
    frame: np.ndarray,
    time_s: float,
    angle_deg: float,
    revolution: int,
    platform_speed_rpm: float | None,
    vertical_translation_mm: float | None,
) -> None:
    speed_text = "n/a" if platform_speed_rpm is None else f"{platform_speed_rpm:.2f} rpm"
    vertical_text = (
        "n/a" if vertical_translation_mm is None else f"{vertical_translation_mm:.3f} mm"
    )
    cv2.rectangle(frame, (18, 16), (510, 212), (0, 0, 0), thickness=-1)
    style = dict(
        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.68,
        color=(255, 255, 255), thickness=2,
    )
    lines = (
        f"Time: {time_s:.3f} s",
        f"Angle: {angle_deg:.2f} deg",
        f"Revolution: {revolution}",
        f"Platform speed: {speed_text}",
        f"Vertical translation: {vertical_text}",
    )
    for index, line in enumerate(lines):
        cv2.putText(frame, line, (34, 48 + index * 36), **style)


def render_video(
    raw_path: Path,
    bins_path: Path,
    output_path: Path,
    accumulation_ms: float,
    source_fps: float,
    playback_speed: float,
    studio_duration_s: float | None,
    start_s: float | None,
    end_s: float | None,
) -> None:
    if accumulation_ms <= 0:
        raise ValueError("Accumulation time must be greater than zero")
    if playback_speed <= 0:
        raise ValueError("Playback speed must be greater than zero")
    if source_fps <= 0:
        raise ValueError("Source FPS must be greater than zero")
    accumulation_us = round(accumulation_ms * 1000.0)
    if accumulation_us <= 0:
        raise ValueError("Accumulation time is below timestamp resolution")
    output_fps = source_fps * playback_speed
    bins = load_bins(bins_path)
    starts = [row.start_s for row in bins]
    reader = eventcv.open(str(raw_path), dt_ms=10)
    width, height = reader.sensor_size
    first_event_us = first_raw_timestamp_us(reader)
    last_event_us = last_raw_timestamp_us(reader)
    if studio_duration_s is not None:
        if studio_duration_s <= 0:
            raise ValueError("Studio duration must be greater than zero")
        raw_duration_s = studio_duration_s
        raw_origin_us = round(last_event_us - studio_duration_s * 1_000_000.0)
    else:
        raw_origin_us = first_event_us
        raw_duration_s = (last_event_us - first_event_us) / 1_000_000.0
    initial_silent_s = (first_event_us - raw_origin_us) / 1_000_000.0
    print(
        f"[TIMING] EVK4 origin={raw_origin_us} us; first CD event={initial_silent_s:.6f} s; "
        f"Studio duration={raw_duration_s:.6f} s",
        flush=True,
    )

    requested_start = 0.0 if start_s is None else start_s
    requested_end = raw_duration_s if end_s is None else end_s
    render_start = max(0.0, requested_start)
    render_end = min(raw_duration_s, requested_end)
    if render_end <= render_start:
        raise ValueError("RAW recording and encoder time bins do not overlap")
    if requested_end > raw_duration_s:
        print(f"[WARNING] RAW ends at {raw_duration_s:.3f} s; requested end is {requested_end:.3f} s.", flush=True)
        print("[WARNING] Rendering only the available overlap.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), output_fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Could not create video: {output_path}")

    sample_period_s = 1.0 / source_fps
    accumulation_s = accumulation_us / 1_000_000.0
    frame_count = int(math.ceil((render_end - render_start) * source_fps))
    written = 0
    try:
        for index in range(frame_count):
            frame_start = render_start + index * sample_period_s
            frame_end = min(frame_start + accumulation_s, render_end)
            centre_s = (frame_start + frame_end) / 2.0
            angle_bin = interpolated_angle_at_time(bins, starts, centre_s)
            stream = reader.slice(
                t0_us=raw_origin_us + round(frame_start * 1_000_000),
                t1_us=raw_origin_us + round(frame_end * 1_000_000),
            )
            frame = render_events(stream, width, height)
            add_overlay(
                frame,
                frame_start,
                angle_bin.wrapped_deg,
                revolution_count(angle_bin.unwrapped_deg, bins[0].unwrapped_deg),
                angle_bin.platform_speed_rpm,
                angle_bin.vertical_translation_mm,
            )
            writer.write(frame)
            written += 1
            if written == 1 or written % 1000 == 0:
                print(f"[VIDEO] {frame_start:.3f} / {render_end:.3f} s ({written}/{frame_count} frames)", flush=True)
    finally:
        writer.release()

    if not written:
        output_path.unlink(missing_ok=True)
        raise RuntimeError("No video frames were written")
    print(f"[DONE] {output_path}", flush=True)
    print(
        f"[DONE] {written} frames, {width}x{height}, {output_fps:g} fps, "
        f"{source_fps:g} source fps, {accumulation_ms:g} ms accumulation, {playback_speed:g}x speed, "
        f"timeline {render_start:.3f}-{render_end:.3f} s",
        flush=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, default=DEFAULT_RAW)
    parser.add_argument("--bins", type=Path, default=DEFAULT_BINS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--accumulation-ms", type=float, default=DEFAULT_ACCUMULATION_MS)
    parser.add_argument("--source-fps", type=float, default=DEFAULT_SOURCE_FPS)
    parser.add_argument("--playback-speed", type=float, default=DEFAULT_PLAYBACK_SPEED)
    parser.add_argument(
        "--studio-duration-s",
        type=float,
        default=DEFAULT_STUDIO_DURATION_S,
        help="Metavision Studio recording duration; preserves silent time before the first CD event",
    )
    parser.add_argument("--start-s", type=float, help="Optional recording-relative start time")
    parser.add_argument("--end-s", type=float, help="Optional recording-relative end time")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    render_video(
        args.raw.resolve(),
        args.bins.resolve(),
        args.output.resolve(),
        args.accumulation_ms,
        args.source_fps,
        args.playback_speed,
        args.studio_duration_s,
        args.start_s,
        args.end_s,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
