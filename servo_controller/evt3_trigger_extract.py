"""Extract external-trigger events from a Prophesee EVT3 RAW file.

This deliberately decodes only EVT_TIME_HIGH, EVT_TIME_LOW and EXT_TRIGGER
words. It therefore avoids loading or expanding the much larger CD event
stream and does not require Metavision SDK Python bindings on the motor PC.


Maintenance contract: output uses native, unshifted camera microseconds.
It is not Studio-relative time; preparation performs that conversion later.
Timestamp state and an odd trailing byte survive chunk boundaries. Decode
only timestamp/trigger words, preserving RAW data and bounded memory usage.
The CSV is replaced only after successful extraction; metadata is a separate
explicit update performed by the experiment workflow.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


EVT_TIME_LOW = 0x6
EVT_TIME_HIGH = 0x8
EXT_TRIGGER = 0xA
TIME_LOOP_US = 1 << 24
MAX_TIMESTAMP_BASE_US = ((1 << 12) - 1) << 12
LOOP_THRESHOLD_US = 10 << 12


def _skip_and_validate_header(stream, raw_path: Path) -> None:
    format_seen = False
    while True:
        position = stream.tell()
        first = stream.read(1)
        if first != b"%":
            stream.seek(position)
            break
        line = first + stream.readline()
        text = line.decode("ascii", errors="replace").strip()
        if text.startswith("% format "):
            format_name = text[9:].split(";", 1)[0]
            if format_name != "EVT3":
                raise ValueError(f"{raw_path.name}: expected EVT3 RAW, found {format_name!r}")
            format_seen = True
        elif text.startswith("% evt "):
            if text[6:] != "3.0":
                raise ValueError(f"{raw_path.name}: expected EVT3 RAW, found {text[6:]!r}")
            format_seen = True
        if text == "% end":
            break
    if not format_seen:
        raise ValueError(f"{raw_path.name}: RAW header does not declare EVT3 format")


def extract_evt3_trigger_events(
    raw_path: Path, output_path: Path, chunk_bytes: int = 16 * 1024 * 1024,
) -> int:
    """Stream *raw_path* and write Trigger In edges to *output_path*.

    Timestamps retain the camera's native, unshifted microsecond timebase.
    The output schema matches the existing Metavision-based extractor.
    """
    # Keep timestamp state across chunks and retain incomplete 16-bit words.
    # The temporary output prevents a decoder failure from publishing a partial
    # CSV under the final name; camera metadata is updated separately.
    raw_path = Path(raw_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    count = 0
    first_time_base_set = False
    current_time_base = 0
    current_time = 0
    time_high_loops = 0
    leftover = b""

    try:
        with raw_path.open("rb") as source, temporary.open(
            "w", newline="", encoding="utf-8"
        ) as destination:
            _skip_and_validate_header(source, raw_path)
            writer = csv.writer(destination)
            writer.writerow(("trigger_index", "raw_timestamp_us", "polarity", "channel_id"))
            while True:
                block = source.read(chunk_bytes)
                if not block:
                    break
                block = leftover + block
                complete = len(block) & ~1
                leftover = block[complete:]
                words = np.frombuffer(block[:complete], dtype="<u2")
                types = words >> 12
                relevant = words[
                    (types == EVT_TIME_LOW) | (types == EVT_TIME_HIGH) | (types == EXT_TRIGGER)
                ]
                for raw_word in relevant:
                    word = int(raw_word)
                    event_type = word >> 12
                    if not first_time_base_set:
                        if event_type != EVT_TIME_HIGH:
                            continue
                        current_time_base = (word & 0x0FFF) << 12
                        current_time = current_time_base
                        first_time_base_set = True
                        continue
                    if event_type == EVT_TIME_HIGH:
                        new_base = ((word & 0x0FFF) << 12) + time_high_loops * TIME_LOOP_US
                        if (
                            current_time_base > new_base
                            and current_time_base - new_base
                            >= MAX_TIMESTAMP_BASE_US - LOOP_THRESHOLD_US
                        ):
                            new_base += TIME_LOOP_US
                            time_high_loops += 1
                        current_time_base = new_base
                        current_time = current_time_base
                    elif event_type == EVT_TIME_LOW:
                        current_time = current_time_base + (word & 0x0FFF)
                    else:
                        writer.writerow((count, current_time, word & 0x1, (word >> 8) & 0xF))
                        count += 1
        temporary.replace(output_path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    return count


def update_camera_metadata(metadata_path: Path, trigger_path: Path, count: int) -> None:
    """Record successful motor-PC extraction in the copied camera metadata."""
    metadata_path = Path(metadata_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["trigger_events_file"] = Path(trigger_path).name
    metadata["trigger_event_count"] = count
    metadata["trigger_extraction_status"] = "complete_motor_pc"
    metadata["trigger_extraction_error"] = ""
    temporary = metadata_path.with_suffix(metadata_path.suffix + ".tmp")
    temporary.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    temporary.replace(metadata_path)
