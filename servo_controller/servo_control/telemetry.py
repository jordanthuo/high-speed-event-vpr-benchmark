"""Bounded measurement history, derived angles and CSV/log serialization.

The fast acquisition path supplies perf_counter midpoint timestamps. Angles
unwrap signed 32-bit counter changes using 2**24 counts per motor revolution;
platform values divide by the configured motor/platform ratio. A clear resets
the unwrap history and elapsed origin. These are not the session compass or
run-relative position origins maintained by the GUI.
CSV headers are downstream contracts: update README_DETAILED.md and export
tests when changing them. Missing optional measurements serialize as blanks.
Buffers are bounded, so a Trends export is not an unlimited run archive.
"""

import csv
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from math import radians, sin, sqrt
from pathlib import Path
from time import perf_counter

from .models import DriveSnapshot, MotionTelemetry


@dataclass(frozen=True)
class TelemetrySample:
    elapsed_s: float
    sampled_monotonic_s: float
    acquisition_duration_s: float | None
    requested_rpm: float
    integrated_command_rpm: float | None
    encoder_feedback_rpm: float | None
    tracking_error_rpm: float | None
    platform_speed_rpm: float | None
    encoder_position_pulses: int | None
    motor_position_deg: float | None
    platform_position_deg: float | None
    drawstring_voltage_v: float | None
    drawstring_extension_mm: float | None


class TelemetryBuffer:
    def __init__(self, max_samples: int = 18_000, platform_ratio: float = 2.0) -> None:
        self.samples: deque[TelemetrySample] = deque(maxlen=max_samples)
        self.platform_ratio = platform_ratio
        self._started = perf_counter()
        self._last_encoder_pulses: int | None = None
        self._unwrapped_encoder_pulses: int | None = None

    def clear(self) -> None:
        self.samples.clear()
        self._started = perf_counter()
        self._last_encoder_pulses = None
        self._unwrapped_encoder_pulses = None

    def append(self, snapshot: DriveSnapshot) -> TelemetrySample:
        sampled_at = perf_counter()
        return self._append_values(
            sampled_at,
            None,
            snapshot.requested_speed_rpm,
            snapshot.integrated_command_rpm,
            snapshot.feedback_speed_rpm,
            snapshot.encoder_position_pulses,
        )

    def append_motion(
        self,
        motion: MotionTelemetry,
        requested_rpm: float,
        drawstring_voltage_v: float | None = None,
        drawstring_extension_mm: float | None = None,
    ) -> TelemetrySample:
        return self._append_values(
            motion.sampled_monotonic_s,
            motion.acquisition_duration_s,
            requested_rpm,
            motion.integrated_command_rpm,
            motion.feedback_speed_rpm,
            motion.encoder_position_pulses,
            drawstring_voltage_v,
            drawstring_extension_mm,
        )

    def _append_values(
        self,
        sampled_at: float,
        acquisition_duration_s: float | None,
        requested_rpm: float,
        integrated_command_rpm: float | None,
        feedback_speed_rpm: float | None,
        encoder_position_pulses: int | None,
        drawstring_voltage_v: float | None = None,
        drawstring_extension_mm: float | None = None,
    ) -> TelemetrySample:
        # Rollover correction assumes successive count changes are less than half
        # the 32-bit range. Large acquisition gaps can violate that assumption.
        # Keep raw pulses in the sample while deriving angles from unwrapped history.
        motor_position_deg: float | None = None
        platform_position_deg: float | None = None
        current_pulses = encoder_position_pulses
        if current_pulses is not None:
            if self._last_encoder_pulses is None or self._unwrapped_encoder_pulses is None:
                self._unwrapped_encoder_pulses = current_pulses
            else:
                delta = current_pulses - self._last_encoder_pulses
                if delta > (1 << 31):
                    delta -= 1 << 32
                elif delta < -(1 << 31):
                    delta += 1 << 32
                self._unwrapped_encoder_pulses += delta
            self._last_encoder_pulses = current_pulses
            motor_position_deg = self._unwrapped_encoder_pulses * 360.0 / (1 << 24)
            if self.platform_ratio > 0:
                platform_position_deg = motor_position_deg / self.platform_ratio
        sample = TelemetrySample(
            elapsed_s=sampled_at - self._started,
            sampled_monotonic_s=sampled_at,
            acquisition_duration_s=acquisition_duration_s,
            requested_rpm=requested_rpm,
            integrated_command_rpm=integrated_command_rpm,
            encoder_feedback_rpm=feedback_speed_rpm,
            tracking_error_rpm=(
                None
                if integrated_command_rpm is None or feedback_speed_rpm is None
                else integrated_command_rpm - feedback_speed_rpm
            ),
            platform_speed_rpm=(
                None if feedback_speed_rpm is None or self.platform_ratio <= 0
                else feedback_speed_rpm / self.platform_ratio
            ),
            encoder_position_pulses=current_pulses,
            motor_position_deg=motor_position_deg,
            platform_position_deg=platform_position_deg,
            drawstring_voltage_v=drawstring_voltage_v,
            drawstring_extension_mm=drawstring_extension_mm,
        )
        self.samples.append(sample)
        return sample

    def recent(self, window_s: float = 60.0) -> list[TelemetrySample]:
        if not self.samples:
            return []
        cutoff = self.samples[-1].elapsed_s - window_s
        return [sample for sample in self.samples if sample.elapsed_s >= cutoff]

    def rms_tracking_error(self, window_s: float = 60.0) -> float | None:
        errors = [
            sample.tracking_error_rpm
            for sample in self.recent(window_s)
            if sample.tracking_error_rpm is not None
        ]
        if not errors:
            return None
        return sqrt(sum(error * error for error in errors) / len(errors))

    def peak_tracking_error(self, window_s: float = 60.0) -> float | None:
        errors = [
            abs(sample.tracking_error_rpm)
            for sample in self.recent(window_s)
            if sample.tracking_error_rpm is not None
        ]
        return max(errors) if errors else None

    def export_csv(self, path: str | Path, channel_group: str = "all") -> None:
        # These Trends schemas differ from ui.py recorded-profile exports. Do not
        # feed them to VPR preparation just because both files contain angles.
        destination = Path(path)
        if channel_group not in ("all", "speed", "position"):
            raise ValueError("Telemetry channel group must be all, speed, or position")
        with destination.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            speed_header = [
                    "elapsed_s",
                    "sampled_monotonic_s",
                    "acquisition_duration_ms",
                    "requested_rpm",
                    "integrated_command_rpm",
                    "encoder_feedback_rpm",
                    "tracking_error_rpm",
                    "platform_speed_rpm",
                    "scene_displacement_ain0_v",
                    "scene_vertical_displacement_mm",
            ]
            position_header = [
                "elapsed_s",
                "sampled_monotonic_s",
                "acquisition_duration_ms",
                "motor_position_deg",
                "motor_sin_theta",
                "platform_position_deg",
                "platform_sin_theta",
                "scene_displacement_ain0_v",
                "scene_vertical_displacement_mm",
            ]
            writer.writerow(
                speed_header if channel_group == "speed"
                else position_header if channel_group == "position"
                else [*speed_header[:-2], *position_header[3:]]
            )
            for sample in self.samples:
                speed_row = [
                        f"{sample.elapsed_s:.3f}",
                        f"{sample.sampled_monotonic_s:.6f}",
                        _optional_number(None if sample.acquisition_duration_s is None else sample.acquisition_duration_s * 1000.0),
                        f"{sample.requested_rpm:.3f}",
                        _optional_number(sample.integrated_command_rpm),
                        _optional_number(sample.encoder_feedback_rpm),
                        _optional_number(sample.tracking_error_rpm),
                        _optional_number(sample.platform_speed_rpm),
                        _optional_number(sample.drawstring_voltage_v),
                        _optional_number(sample.drawstring_extension_mm),
                ]
                position_row = [
                    f"{sample.elapsed_s:.3f}",
                    f"{sample.sampled_monotonic_s:.6f}",
                    _optional_number(None if sample.acquisition_duration_s is None else sample.acquisition_duration_s * 1000.0),
                    _optional_number(sample.motor_position_deg),
                    _optional_number(None if sample.motor_position_deg is None else sin(radians(sample.motor_position_deg))),
                    _optional_number(sample.platform_position_deg),
                    _optional_number(None if sample.platform_position_deg is None else sin(radians(sample.platform_position_deg))),
                    _optional_number(sample.drawstring_voltage_v),
                    _optional_number(sample.drawstring_extension_mm),
                ]
                writer.writerow(
                    speed_row if channel_group == "speed"
                    else position_row if channel_group == "position"
                    else [*speed_row[:-2], *position_row[3:]]
                )


@dataclass(frozen=True)
class EventRecord:
    timestamp: datetime
    level: str
    source: str
    event: str
    details: str = ""


class EventHistory:
    def __init__(self, max_records: int = 5000) -> None:
        self.records: deque[EventRecord] = deque(maxlen=max_records)

    def add(self, level: str, source: str, event: str, details: str = "") -> EventRecord:
        record = EventRecord(datetime.now(), level.upper(), source.upper(), event, details)
        self.records.append(record)
        return record

    def export_csv(self, path: str | Path) -> None:
        destination = Path(path)
        with destination.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["timestamp", "level", "source", "event", "details"])
            for record in self.records:
                writer.writerow(
                    [
                        record.timestamp.isoformat(timespec="milliseconds"),
                        record.level,
                        record.source,
                        record.event,
                        record.details,
                    ]
                )


def speed_sweep_values(start: float, end: float, step: float, cycles: int = 1) -> list[float]:
    if step <= 0:
        raise ValueError("Step must be positive")
    if cycles < 1:
        raise ValueError("Cycles must be at least 1")
    direction = 1.0 if end >= start else -1.0
    signed_step = step * direction
    forward: list[float] = []
    value = start
    while (direction > 0 and value <= end + 1e-9) or (direction < 0 and value >= end - 1e-9):
        forward.append(round(value, 6))
        value += signed_step
    if not forward or forward[-1] != end:
        forward.append(end)
    reverse = list(reversed(forward[:-1]))
    one_cycle = forward + reverse
    return one_cycle * cycles


def _optional_number(value: float | None) -> str:
    return "" if value is None else f"{value:.3f}"
