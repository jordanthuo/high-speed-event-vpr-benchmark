"""Make a timestamped RAW preview before encoder synchronization is available.

Studio duration sets recording zero using the same last-event-minus-duration
convention as prepare_experiment.py. Without it, times are first-event-relative
and must not be entered directly as manual_evk4_motion_s. No input is modified.
At the defaults, 2 ms apertures begin every 10 ms; playback is real time.
--playback-speed scales the MP4 rate without changing recording timestamps.
Use --setup experiment_setup.csv --run run1 to read the RAW path and duration
from a prepared experiment, or --raw and --studio-duration-s directly.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import eventcv
import numpy as np


def positive(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Use a finite number greater than zero")
    return number


def setup_run(path: Path, label: str) -> tuple[Path, float]:
    """Resolve inputs relative to the setup file, without needing timing bins."""
    path = path.expanduser().resolve()
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"run_label", "raw_file", "studio_duration_s"}
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError("Setup CSV needs run_label, raw_file and studio_duration_s columns")
        rows = list(reader)
    matches = [row for row in rows if (row.get("run_label") or "").strip() == label]
    if len(matches) != 1:
        labels = ", ".join((row.get("run_label") or "").strip() for row in rows)
        raise ValueError(f"Expected one setup row for {label!r}; found {len(matches)}. Run labels: {labels}")
    row = matches[0]
    raw_value = (row.get("raw_file") or "").strip()
    if not raw_value:
        raise ValueError(f"{label}: raw_file is blank in {path}")
    try:
        duration = float((row.get("studio_duration_s") or "").strip())
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError
    except ValueError:
        raise ValueError(f"{label}: enter a positive duration in studio_duration_s in {path}") from None
    raw = Path(raw_value).expanduser()
    if not raw.is_absolute():
        raw = path.parent / raw
    return raw.resolve(), duration


def render_preview(args: argparse.Namespace) -> int:
    setup = getattr(args, "setup", None)
    run = getattr(args, "run", None)
    duration = args.studio_duration_s
    if setup is not None:
        if not run:
            raise ValueError("With --setup, select a run using --run (for example --run run1)")
        if args.raw is not None or duration is not None:
            raise ValueError("Use --setup with --run, or --raw with --studio-duration-s")
        raw, duration = setup_run(setup, run)
        print(f"[SETUP] {run}: {raw}; Studio duration={duration:g} s", flush=True)
    else:
        if run:
            raise ValueError("--run requires --setup")
        raw = args.raw.expanduser().resolve()
    output = args.output.expanduser().resolve()
    frame_csv = output.with_name(output.stem + "_frames.csv")
    if not raw.is_file():
        raise FileNotFoundError(raw)
    if output.suffix.lower() != ".mp4":
        raise ValueError("Output must end in .mp4")
    for target in (output, frame_csv):
        if target.exists():
            raise FileExistsError(f"Choose a new output name; already exists: {target}")
    if not math.isfinite(args.start_s) or args.start_s < 0:
        raise ValueError("--start-s must be finite and nonnegative")
    if args.end_s is not None and not math.isfinite(args.end_s):
        raise ValueError("--end-s must be finite")
    if args.source_fps > 1_000_000 or round(args.accumulation_ms * 1000) < 1:
        raise ValueError("Frame spacing and accumulation must be at least 1 microsecond")
    playback_speed = args.playback_speed
    output_fps = args.source_fps * playback_speed
    if not math.isfinite(output_fps) or output_fps <= 0:
        raise ValueError("Playback speed and output frame rate must be finite and positive")

    reader = eventcv.open(str(raw), dt_ms=10)
    first_us, last_us = (int(round(t)) for t in reader.time_span("us"))
    studio = duration is not None
    if studio:
        duration_s = duration
        origin_us = round(last_us - duration_s * 1_000_000)
        if origin_us > first_us + 1:
            raise ValueError("Studio duration is shorter than the event span; check the RAW/duration pairing")
        time_label = "EVK4 recording"
    else:
        origin_us = first_us
        duration_s = (last_us - first_us + 1) / 1_000_000
        time_label = "FIRST EVENT relative"
        print("[WARNING] No Studio duration: these times are NOT manual_evk4_motion_s.", flush=True)
    end_s = min(duration_s, args.end_s) if args.end_s is not None else duration_s
    if end_s <= args.start_s:
        raise ValueError("Selected interval must overlap the recording and end after --start-s")
    width, height = reader.sensor_size
    banner_height = 88
    # MP4 encoders commonly crop odd dimensions. Pad instead of losing pixels.
    frame_width = width + width % 2
    frame_height = height + banner_height + (height + banner_height) % 2
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*"mp4v"), output_fps,
        (frame_width, frame_height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not create MP4: {output}")
    count = math.ceil((end_s - args.start_s) * args.source_fps)
    aperture_us = round(args.accumulation_ms * 1000)
    print(f"[TIMING] {time_label}; RAW origin={origin_us} us; last event={last_us} us", flush=True)
    print(f"[VIDEO] {args.source_fps:g} source fps; {args.accumulation_ms:g} ms aperture; "
          f"{playback_speed:g}x playback ({output_fps:g} output fps); {count} frames", flush=True)
    fields = (
        "frame_index", "video_time_s", "window_start_s", "window_end_s",
        "raw_start_us", "raw_end_us", "time_reference", "manual_evk4_motion_s",
    )
    try:
        with frame_csv.open("x", newline="", encoding="utf-8") as handle:
            rows = csv.DictWriter(handle, fieldnames=fields)
            rows.writeheader()
            for index in range(count):
                start = args.start_s + index / args.source_fps
                t0 = origin_us + round(start * 1_000_000)
                t1 = min(t0 + aperture_us, origin_us + round(end_s * 1_000_000), last_us + 1)
                frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
                if t1 > t0:
                    events = reader.slice(t0_us=t0, t1_us=t1).numpy()
                    for on, colour in ((False, (255, 255, 255)), (True, (255, 100, 0))):
                        selected = events[events[:, 3] > 0] if on else events[events[:, 3] <= 0]
                        if len(selected):
                            x = selected[:, 0].astype(np.intp)
                            y = selected[:, 1].astype(np.intp) + banner_height
                            frame[y, x] = colour
                stop = (t1 - origin_us) / 1_000_000
                labels = (
                    f"{time_label}: {start:.6f} s | frame {index}",
                    f"Window [{start:.6f}, {stop:.6f}) s | {args.accumulation_ms:g} ms | {args.source_fps:g} fps",
                    f"RAW timestamp: {t0} us" + ("" if studio else " | NOT Studio time"),
                )
                for line, label in enumerate(labels):
                    cv2.putText(frame, label, (10, 23 + line * 27), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (255, 255, 255), 1, cv2.LINE_AA)
                writer.write(frame)
                rows.writerow({
                    "frame_index": index, "video_time_s": f"{index / output_fps:.6f}",
                    "window_start_s": f"{start:.6f}", "window_end_s": f"{stop:.6f}",
                    "raw_start_us": t0, "raw_end_us": t1,
                    "time_reference": "studio_duration" if studio else "first_event",
                    "manual_evk4_motion_s": f"{start:.6f}" if studio else "",
                })
                if index == 0 or (index + 1) % 1000 == 0:
                    print(f"[VIDEO] {index + 1}/{count}; {start:.3f} s", flush=True)
    finally:
        writer.release()
    print(f"[DONE] {output}\n[FRAMES] {frame_csv}", flush=True)
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--raw", type=Path)
    source.add_argument("--setup", type=Path, help="Path to experiment_setup.csv")
    parser.add_argument("--run", help="Exact run_label from the setup CSV, such as run1")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--studio-duration-s", type=positive,
                        help="Studio duration for this RAW; needed for setup-compatible timestamps")
    parser.add_argument("--accumulation-ms", type=positive, default=2.0,
                        help="Event window per frame in milliseconds (default: 2)")
    parser.add_argument("--source-fps", type=positive, default=100.0,
                        help="Samples per second of recording time (default: 100)")
    parser.add_argument("--playback-speed", type=positive, default=1.0,
                        help="Playback multiplier: 1 real time, 0.25 quarter speed, 2 double speed")
    parser.add_argument("--start-s", type=float, default=0.0)
    parser.add_argument("--end-s", type=float)
    args = parser.parse_args()
    try:
        render_preview(args)
    except (ValueError, OSError, RuntimeError) as exc:
        parser.exit(1, f"[ERROR] {exc}\n")


if __name__ == "__main__":
    main()
