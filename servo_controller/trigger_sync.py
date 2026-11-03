"""Pair 1 Hz UART attempts with camera edges and fit an affine clock map.

Input UART times are LattePanda perf_counter nanoseconds; trigger times are
native RAW microseconds. Subtract raw_origin_us before fitting camera time
against seconds since LattePanda T0. load_* validates/filter inputs; fitting
expects those prepared lists. The returned anchor feeds time-bin generation.
Residuals plus half software-write duration form a diagnostic uncertainty,
not a confidence bound on physical synchronization. Inspect missing initial
pulses: a periodic train can admit a whole-period pairing ambiguity.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from typing import Iterable

from generate_vpr_time_bins import CameraAnchor


NOMINAL_TRIGGER_PERIOD_US = 1_000_000.0
MAX_TRIGGER_GRID_ERROR_US = 200_000.0
MAX_CLOCK_DRIFT_PPM = 5_000.0


@dataclass(frozen=True)
class UartWrite:
    sequence: int
    requested_ns: int
    completed_ns: int
    lattepanda_time_s: float

    @property
    def midpoint_ns(self) -> float:
        return (self.requested_ns + self.completed_ns) / 2.0

    @property
    def half_duration_us(self) -> float:
        return (self.completed_ns - self.requested_ns) / 2_000.0


@dataclass(frozen=True)
class TriggerEvent:
    index: int
    raw_timestamp_us: int
    polarity: int
    channel_id: int


@dataclass(frozen=True)
class TriggerMatch:
    sequence: int
    trigger_index: int
    requested_ns: int
    completed_ns: int
    lattepanda_time_s: float
    camera_time_us: float
    predicted_camera_time_us: float
    residual_us: float
    write_half_duration_us: float


@dataclass(frozen=True)
class TriggerFit:
    intercept_camera_us_at_t0: float
    scale_camera_us_per_lattepanda_s: float
    drift_ppm: float
    rms_residual_us: float
    max_abs_residual_us: float
    timing_uncertainty_us: float
    matches: tuple[TriggerMatch, ...]


def _required_column(fieldnames: Iterable[str], choices: tuple[str, ...], description: str) -> str:
    available = set(fieldnames)
    for name in choices:
        if name in available:
            return name
    raise ValueError(f"Trigger CSV is missing {description}; expected one of {choices!r}")


def load_uart_writes(path: Path, camera_metadata_path: Path) -> list[UartWrite]:
    metadata = json.loads(camera_metadata_path.read_text(encoding="utf-8"))
    t0_ns = metadata.get("lattepanda_t0_monotonic_ns")
    if t0_ns is None:
        raise ValueError("Camera metadata is missing lattepanda_t0_monotonic_ns")
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {
            "sequence", "write_requested_lattepanda_monotonic_ns",
            "write_completed_lattepanda_monotonic_ns", "status",
        }
        missing = sorted(required.difference(reader.fieldnames or ()))
        if missing:
            raise ValueError(f"UART writes CSV is missing columns: {', '.join(missing)}")
        writes: list[UartWrite] = []
        for row in reader:
            if row["status"].strip().lower() != "completed":
                continue
            requested_ns = int(row["write_requested_lattepanda_monotonic_ns"])
            completed_ns = int(row["write_completed_lattepanda_monotonic_ns"])
            if completed_ns < requested_ns:
                raise ValueError("UART write completion precedes its request")
            midpoint_ns = (requested_ns + completed_ns) / 2.0
            writes.append(UartWrite(
                sequence=int(row["sequence"]),
                requested_ns=requested_ns,
                completed_ns=completed_ns,
                lattepanda_time_s=(midpoint_ns - int(t0_ns)) / 1_000_000_000.0,
            ))
    writes.sort(key=lambda item: item.sequence)
    if len(writes) < 3:
        raise ValueError("Trigger synchronization requires at least three completed UART writes")
    if len({item.sequence for item in writes}) != len(writes):
        raise ValueError("UART write sequences are not unique")
    return writes


def load_trigger_events(
    path: Path, polarity: int = 0, channel_id: int = 0,
) -> list[TriggerEvent]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        names = reader.fieldnames or []
        time_column = _required_column(
            names, ("raw_timestamp_us", "timestamp_us", "t"), "RAW timestamp",
        )
        polarity_column = _required_column(names, ("polarity", "p"), "polarity")
        channel_column = _required_column(names, ("channel_id", "id"), "channel ID")
        events: list[TriggerEvent] = []
        for source_index, row in enumerate(reader):
            value = row[polarity_column].strip().lower()
            event_polarity = 0 if value == "falling" else 1 if value == "rising" else int(value)
            event_channel = int(row[channel_column])
            if event_polarity != polarity or event_channel != channel_id:
                continue
            events.append(TriggerEvent(
                index=int(row.get("trigger_index", source_index)),
                raw_timestamp_us=int(row[time_column]),
                polarity=event_polarity,
                channel_id=event_channel,
            ))
    events.sort(key=lambda item: item.raw_timestamp_us)
    if len(events) < 3:
        edge = "falling" if polarity == 0 else "rising"
        raise ValueError(
            f"Trigger synchronization requires at least three {edge}-edge events "
            f"on channel {channel_id}; found {len(events)}"
        )
    return events


def _linear_fit(pairs: list[tuple[UartWrite, TriggerEvent]], raw_origin_us: int) -> tuple[float, float, list[float]]:
    xs = [write.lattepanda_time_s for write, _event in pairs]
    ys = [event.raw_timestamp_us - raw_origin_us for _write, event in pairs]
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denominator = sum((value - x_mean) ** 2 for value in xs)
    if denominator <= 0:
        raise ValueError("UART timestamps do not span enough time for an affine fit")
    scale = sum(
        (x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)
    ) / denominator
    intercept = y_mean - scale * x_mean
    residuals = [y - (intercept + scale * x) for x, y in zip(xs, ys)]
    return intercept, scale, residuals


def fit_trigger_sync(
    writes: Iterable[UartWrite], events: Iterable[TriggerEvent], raw_origin_us: int,
) -> TriggerFit:
    # Prefer most matched pairs, then lowest RMS. This heuristic is not proof
    # of unique pulse correspondence. The drift bound is currently applied
    # before robust refitting, not rechecked on the final filtered slope.
    write_list = list(writes)
    event_list = list(events)
    writes_by_sequence = {item.sequence: item for item in write_list}

    first_timestamp = event_list[0].raw_timestamp_us
    event_slots: dict[int, TriggerEvent] = {}
    for event in event_list:
        elapsed = event.raw_timestamp_us - first_timestamp
        slot = int(round(elapsed / NOMINAL_TRIGGER_PERIOD_US))
        grid_error = abs(elapsed - slot * NOMINAL_TRIGGER_PERIOD_US)
        if grid_error > MAX_TRIGGER_GRID_ERROR_US:
            continue
        previous = event_slots.get(slot)
        if previous is None:
            event_slots[slot] = event
        else:
            previous_error = abs(
                previous.raw_timestamp_us - first_timestamp - slot * NOMINAL_TRIGGER_PERIOD_US
            )
            if grid_error < previous_error:
                event_slots[slot] = event

    candidates: list[tuple[int, float, float, float, list[tuple[UartWrite, TriggerEvent]], list[float]]] = []
    for base_sequence in sorted(writes_by_sequence):
        pairs = [
            (writes_by_sequence[base_sequence + slot], event)
            for slot, event in sorted(event_slots.items())
            if base_sequence + slot in writes_by_sequence
        ]
        if len(pairs) < 3:
            continue
        intercept, scale, residuals = _linear_fit(pairs, raw_origin_us)
        drift_ppm = (scale / 1_000_000.0 - 1.0) * 1_000_000.0
        if abs(drift_ppm) > MAX_CLOCK_DRIFT_PPM:
            continue

        median_residual = statistics.median(residuals)
        mad = statistics.median(abs(value - median_residual) for value in residuals)
        cutoff = max(500.0, 6.0 * mad)
        filtered = [
            pair for pair, residual in zip(pairs, residuals)
            if abs(residual - median_residual) <= cutoff
        ]
        if len(filtered) < 3:
            continue
        intercept, scale, residuals = _linear_fit(filtered, raw_origin_us)
        drift_ppm = (scale / 1_000_000.0 - 1.0) * 1_000_000.0
        rms = math.sqrt(statistics.fmean(value * value for value in residuals))
        candidates.append((len(filtered), rms, intercept, scale, filtered, residuals))

    if not candidates:
        raise ValueError(
            "Could not match the 1 Hz falling-edge sequence to completed UART writes"
        )
    count, rms, intercept, scale, pairs, residuals = min(
        candidates, key=lambda item: (-item[0], item[1])
    )
    max_abs = max(abs(value) for value in residuals)
    max_half_write = max(write.half_duration_us for write, _event in pairs)
    matches = tuple(
        TriggerMatch(
            sequence=write.sequence,
            trigger_index=event.index,
            requested_ns=write.requested_ns,
            completed_ns=write.completed_ns,
            lattepanda_time_s=write.lattepanda_time_s,
            camera_time_us=event.raw_timestamp_us - raw_origin_us,
            predicted_camera_time_us=intercept + scale * write.lattepanda_time_s,
            residual_us=residual,
            write_half_duration_us=write.half_duration_us,
        )
        for (write, event), residual in zip(pairs, residuals)
    )
    return TriggerFit(
        intercept_camera_us_at_t0=intercept,
        scale_camera_us_per_lattepanda_s=scale,
        drift_ppm=(scale / 1_000_000.0 - 1.0) * 1_000_000.0,
        rms_residual_us=rms,
        max_abs_residual_us=max_abs,
        timing_uncertainty_us=max_abs + max_half_write,
        matches=matches,
    )


def build_trigger_anchor(fit: TriggerFit, studio_duration_s: float) -> CameraAnchor:
    # Camera bounds here are Studio-relative [0,duration). Native RAW trigger
    # timestamps were shifted by raw_origin_us during fitting.
    if studio_duration_s <= 0:
        raise ValueError("studio_duration_s must be greater than zero")
    camera_t0_us = round(fit.intercept_camera_us_at_t0)
    first = fit.matches[0]
    return CameraAnchor(
        camera_start_us=0,
        camera_end_us=round(studio_duration_s * 1_000_000.0),
        camera_t0_us=camera_t0_us,
        pre_t0_us=camera_t0_us,
        sync_method="trigger_affine",
        sync_anchor_camera_us=round(first.camera_time_us),
        sync_anchor_lattepanda_time_s=f"{first.lattepanda_time_s:.9f}",
        scale_camera_us_per_lattepanda_s=Decimal(
            f"{fit.scale_camera_us_per_lattepanda_s:.9f}"
        ),
        sync_drift_ppm=f"{fit.drift_ppm:.6f}",
        sync_fit_rms_us=f"{fit.rms_residual_us:.3f}",
        sync_fit_max_abs_us=f"{fit.max_abs_residual_us:.3f}",
        sync_match_count=len(fit.matches),
    )


def write_trigger_diagnostics(path: Path, fit: TriggerFit) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "sequence", "trigger_index", "write_requested_lattepanda_monotonic_ns",
        "write_completed_lattepanda_monotonic_ns", "lattepanda_time_s",
        "evk4_time_us", "predicted_evk4_time_us", "residual_us",
        "write_half_duration_us",
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for match in fit.matches:
            writer.writerow({
                "sequence": match.sequence,
                "trigger_index": match.trigger_index,
                "write_requested_lattepanda_monotonic_ns": match.requested_ns,
                "write_completed_lattepanda_monotonic_ns": match.completed_ns,
                "lattepanda_time_s": f"{match.lattepanda_time_s:.9f}",
                "evk4_time_us": f"{match.camera_time_us:.3f}",
                "predicted_evk4_time_us": f"{match.predicted_camera_time_us:.3f}",
                "residual_us": f"{match.residual_us:.3f}",
                "write_half_duration_us": f"{match.write_half_duration_us:.3f}",
            })
