"""Compare two or three runs on a shared wrapped-angle timeline.

Each output frame requests one angle and independently finds the source time
inside each selected revolution. Thus equal panel angles do not imply equal
physical speed or elapsed time. Missing coverage stays NO ANGLE MATCH rather
than borrowing another revolution. The companion CSV is one row per panel per
output frame and records the actual assigned bin values for audit.
Revolution/crossing conventions are shared with the reference/video helpers;
review their forward-turn assumptions before using reversing trajectories.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import eventcv
import numpy as np

from build_angle_reference_csv import Sample, crossing_time, load_samples
from make_angle_video import (
    AngleBin,
    label_bin_at_time,
    last_raw_timestamp_us,
    load_bins,
    render_events,
    revolution_count,
)


@dataclass(frozen=True)
class AngleVideoInput:
    label: str
    revolution: int
    raw_file: Path
    time_bins_csv: Path
    studio_duration_s: float


@dataclass
class PreparedInput:
    source: AngleVideoInput
    samples: list[Sample]
    bins: list[AngleBin]
    bin_starts: list[float]
    reader: object
    raw_origin_us: int
    width: int
    height: int


def crossing_for_revolution(
    samples: list[Sample], target_wrapped_deg: float, revolution: int,
) -> tuple[float, float] | None:
    """Return (EVK4 time, unwrapped level) using the established revolution numbering."""
    if not 0.0 <= target_wrapped_deg < 360.0:
        raise ValueError("Target wrapped angle must be in [0, 360)")
    if revolution < 0:
        raise ValueError("Revolution must be non-negative")
    origin = samples[0].angle_deg
    minimum = min(sample.angle_deg for sample in samples)
    maximum = max(sample.angle_deg for sample in samples)
    first_k = math.ceil((minimum - target_wrapped_deg) / 360.0)
    last_k = math.floor((maximum - target_wrapped_deg) / 360.0)
    for k in range(first_k, last_k + 1):
        level = target_wrapped_deg + 360.0 * k
        if revolution_count(level, origin) != revolution:
            continue
        time_s = crossing_time(samples, level)
        if time_s is not None:
            return time_s, level
    return None


def _prepare(source: AngleVideoInput) -> PreparedInput:
    samples = load_samples(source.time_bins_csv)
    bins = load_bins(source.time_bins_csv)
    reader = eventcv.open(str(source.raw_file), dt_ms=10)
    width, height = reader.sensor_size
    raw_origin_us = round(last_raw_timestamp_us(reader) - source.studio_duration_s * 1_000_000.0)
    return PreparedInput(
        source, samples, bins, [item.start_s for item in bins], reader,
        raw_origin_us, width, height,
    )


def _draw_footer(
    canvas: np.ndarray, x0: int, y0: int, tile_width: int,
    prepared: PreparedInput, target: float, time_s: float | None,
    assigned: AngleBin | None,
) -> None:
    cv2.rectangle(canvas, (x0, y0), (x0 + tile_width, y0 + 154), (0, 0, 0), -1)
    if assigned is None or time_s is None:
        lines = (
            f"{prepared.source.label} | revolution {prepared.source.revolution}",
            f"Target {target:.2f} deg | NO MATCH",
        )
    else:
        speed = "n/a" if assigned.platform_speed_rpm is None else f"{assigned.platform_speed_rpm:.2f} rpm"
        vertical = (
            "n/a" if assigned.vertical_translation_mm is None
            else f"{assigned.vertical_translation_mm:.3f} mm"
        )
        lines = (
            f"{prepared.source.label} | revolution {prepared.source.revolution}",
            f"Target {target:.2f} deg | recorded {assigned.wrapped_deg:.3f} deg",
            f"EVK4 {time_s:.6f} s",
            f"Speed {speed} | vertical {vertical}",
        )
    for index, line in enumerate(lines):
        cv2.putText(
            canvas, line, (x0 + 10, y0 + 29 + index * 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.53, (255, 255, 255), 1, cv2.LINE_AA,
        )


def render_angle_synchronised_video(
    sources: list[AngleVideoInput], output: Path, angle_step_deg: float = 0.5,
    accumulation_ms: float = 2.0, fps: float = 30.0, tile_width: int = 512,
    start_angle_deg: float = 0.0, end_angle_deg: float = 360.0,
) -> Path:
    # One CSV row per frame per panel allows auditing displayed feedback.
    # Keep missing matches explicit; substituting another revolution hides
    # incomplete experimental coverage.
    if not 2 <= len(sources) <= 3:
        raise ValueError("Select two or three run/revolution instances")
    if not math.isfinite(angle_step_deg) or angle_step_deg <= 0 or angle_step_deg > 360:
        raise ValueError("Angle step must be greater than zero and no more than 360 degrees")
    if not (math.isfinite(start_angle_deg) and math.isfinite(end_angle_deg)
            and 0 <= start_angle_deg < end_angle_deg <= 360):
        raise ValueError("Angle range requires 0 <= start angle < end angle <= 360")
    if accumulation_ms <= 0 or fps <= 0 or tile_width <= 0:
        raise ValueError("Accumulation, FPS and tile width must be greater than zero")

    prepared = [_prepare(source) for source in sources]
    sensor_aspect = prepared[0].height / prepared[0].width
    if any((item.width, item.height) != (prepared[0].width, prepared[0].height) for item in prepared):
        raise ValueError("All selected RAW files must have the same sensor resolution")
    tile_height = round(tile_width * sensor_aspect)
    header_height = 66
    footer_height = 154
    canvas_size = (tile_width * len(prepared), header_height + tile_height + footer_height)
    accumulation_s = accumulation_ms / 1000.0
    frame_count = math.ceil((end_angle_deg - start_angle_deg) / angle_step_deg)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output.with_suffix(".csv")
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, canvas_size)
    if not writer.isOpened():
        raise RuntimeError(f"Could not create video: {output}")
    csv_fields = (
        "output_frame", "target_wrapped_angle_deg", "run_label", "revolution",
        "matched", "evk4_time_s", "actual_unwrapped_angle_deg",
        "actual_wrapped_angle_deg", "platform_speed_rpm", "vertical_translation_mm",
    )
    written = 0
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_handle:
            csv_writer = csv.DictWriter(csv_handle, fieldnames=csv_fields)
            csv_writer.writeheader()
            for frame_index in range(frame_count):
                target = start_angle_deg + frame_index * angle_step_deg
                canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
                cv2.putText(
                    canvas, f"ANGLE-SYNCHRONISED COMPARISON   Target: {target:.2f} deg",
                    (18, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.88,
                    (255, 255, 255), 2, cv2.LINE_AA,
                )
                for panel, item in enumerate(prepared):
                    x0 = panel * tile_width
                    match = crossing_for_revolution(item.samples, target, item.source.revolution)
                    assigned = None
                    time_s = None
                    level = None
                    if match is not None:
                        time_s, level = match
                        stream = item.reader.slice(
                            t0_us=item.raw_origin_us + round(time_s * 1_000_000.0),
                            t1_us=item.raw_origin_us + round((time_s + accumulation_s) * 1_000_000.0),
                        )
                        frame = render_events(stream, item.width, item.height)
                        tile = cv2.resize(frame, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
                        assigned = label_bin_at_time(
                            item.bins, item.bin_starts, time_s + accumulation_s / 2.0,
                        )
                        canvas[header_height:header_height + tile_height, x0:x0 + tile_width] = tile
                    else:
                        cv2.putText(
                            canvas, "NO ANGLE MATCH", (x0 + 55, header_height + tile_height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 255), 2, cv2.LINE_AA,
                        )
                    _draw_footer(
                        canvas, x0, header_height + tile_height, tile_width,
                        item, target, time_s, assigned,
                    )
                    csv_writer.writerow({
                        "output_frame": frame_index,
                        "target_wrapped_angle_deg": f"{target:.6f}",
                        "run_label": item.source.label,
                        "revolution": item.source.revolution,
                        "matched": str(match is not None).lower(),
                        "evk4_time_s": "" if time_s is None else f"{time_s:.9f}",
                        "actual_unwrapped_angle_deg": "" if assigned is None else f"{assigned.unwrapped_deg:.9f}",
                        "actual_wrapped_angle_deg": "" if assigned is None else f"{assigned.wrapped_deg:.9f}",
                        "platform_speed_rpm": "" if assigned is None or assigned.platform_speed_rpm is None else f"{assigned.platform_speed_rpm:.9f}",
                        "vertical_translation_mm": "" if assigned is None or assigned.vertical_translation_mm is None else f"{assigned.vertical_translation_mm:.9f}",
                    })
                writer.write(canvas)
                written += 1
                if written == 1 or written % 100 == 0 or written == frame_count:
                    print(f"[ANGLE VIDEO] frame {written}/{frame_count}; target {target:.2f} deg", flush=True)
    finally:
        writer.release()
    if not written:
        output.unlink(missing_ok=True)
        csv_path.unlink(missing_ok=True)
        raise RuntimeError("No video frames were written")
    print(f"[DONE] Video: {output.resolve()}", flush=True)
    print(f"[DONE] Alignment CSV: {csv_path.resolve()}", flush=True)
    return output
