"""CSV motion-profile parsing and drive-oriented compilation helpers.

The parser is pure: it reads CSV and produces immutable motion descriptions,
without opening the drive. ui.py owns staging/execution. Exact ordered headers
select a schema; preview interpolation is not the live speed command policy
(the speed worker sends row setpoints at their deadlines). Position targets
are run-relative platform degrees and speed limits are motor rpm.
Changing a schema requires updating examples, export consumers and tests.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ProfilePoint:
    time_s: float
    value: float


@dataclass(frozen=True)
class CompiledSegment:
    start_s: float
    end_s: float
    start_value: float
    end_value: float
    motor_speed_rpm: float
    platform_speed_rpm: float


@dataclass(frozen=True)
class MotionProfile:
    kind: str
    source: Path
    points: tuple[ProfilePoint, ...]
    segments: tuple[CompiledSegment, ...]
    ratio: float
    segment_target_speeds_rpm: tuple[float, ...] | None = None
    position_path_targets_deg: tuple[float, ...] | None = None
    position_path_speeds_rpm: tuple[float, ...] | None = None
    position_path_holds_s: tuple[float, ...] | None = None

    @property
    def duration_s(self) -> float:
        return self.points[-1].time_s

    @property
    def maximum_motor_speed_rpm(self) -> float:
        if self.position_path_speeds_rpm is not None:
            return max(self.position_path_speeds_rpm, default=0.0)
        if self.segment_target_speeds_rpm is not None:
            return max(self.segment_target_speeds_rpm, default=0.0)
        if self.kind == "speed":
            return max(abs(point.value) for point in self.points)
        return max((abs(segment.motor_speed_rpm) for segment in self.segments), default=0.0)

    @property
    def final_value(self) -> float:
        return self.points[-1].value

    @property
    def pr_path_count(self) -> int:
        if self.position_path_targets_deg is not None:
            return len(self.position_path_targets_deg)
        return len(self.segments)


def validate_live_speed_profile(profile: MotionProfile) -> None:
    """Enforce the live-run terminal stop without requiring a zero first row."""
    if profile.kind != "speed":
        raise ValueError("The profile is not a speed profile")
    if abs(profile.points[-1].value) > 1e-9:
        raise ValueError("Physical Speed CSV profiles must end with a 0 rpm row")


def _validated_rows(reader: csv.DictReader):
    """Reject extra or missing cells instead of silently dropping CSV data."""
    for row in reader:
        line_number = reader.line_num
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f"Line {line_number}: number of fields must match the CSV headers")
        yield line_number, row


def load_motion_profile(path: str | Path, ratio: float = 2.0) -> MotionProfile:
    """Load either a speed timeline or a position/speed/hold waypoint plan."""
    source = Path(path)
    if not math.isfinite(ratio) or ratio <= 0:
        raise ValueError("Motor-to-platform ratio must be finite and greater than zero")
    with source.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        headers = tuple((name or "").strip() for name in (reader.fieldnames or ()))
        # Normalize names after checking their exact order and membership.
        reader.fieldnames = list(headers)
        if headers == ("platform_position_deg", "motor_speed_rpm", "hold_s"):
            return _load_position_speed_profile(source, reader, ratio)
        if headers != ("time_s", "motor_rpm"):
            raise ValueError(
                "Headers must be exactly 'time_s,motor_rpm' or "
                "'platform_position_deg,motor_speed_rpm,hold_s'. "
                "Legacy time-based position headers are no longer supported."
            )
        points: list[ProfilePoint] = []
        for line_number, row in _validated_rows(reader):
            try:
                time_s = float(row["time_s"])
                value = float(row["motor_rpm"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Line {line_number} contains a non-numeric value") from exc
            if not math.isfinite(time_s) or not math.isfinite(value):
                raise ValueError(f"Line {line_number} must contain finite values")
            if time_s < 0:
                raise ValueError(f"Line {line_number}: time cannot be negative")
            if points and time_s <= points[-1].time_s:
                raise ValueError(f"Line {line_number}: time must increase strictly")
            if abs(value) > 1000.0:
                raise ValueError(f"Line {line_number}: motor speed commands must remain within +/-1000 rpm")
            points.append(ProfilePoint(time_s, value))
    if len(points) < 2:
        raise ValueError("A profile requires at least two data rows")
    if abs(points[0].time_s) > 1e-9:
        raise ValueError("The first profile row must start at time_s = 0")
    segments = tuple(
        CompiledSegment(
            first.time_s, second.time_s, first.value, second.value,
            (first.value + second.value) / 2.0,
            (first.value + second.value) / 2.0 / ratio,
        )
        for first, second in zip(points, points[1:])
    )
    return MotionProfile("speed", source, tuple(points), segments, ratio)


def _load_position_speed_profile(
    source: Path,
    reader: csv.DictReader,
    ratio: float,
) -> MotionProfile:
    """Load movement and stationary-hold waypoints without zero-speed division."""
    targets: list[float] = []
    speeds: list[float] = []
    holds: list[float] = []
    for line_number, row in _validated_rows(reader):
        try:
            target = float(row["platform_position_deg"])
            speed = float(row["motor_speed_rpm"])
            hold = float(row["hold_s"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Line {line_number} contains a non-numeric value") from exc
        if not all(math.isfinite(value) for value in (target, speed, hold)):
            raise ValueError(f"Line {line_number} must contain finite values")
        if not 0 <= speed <= 1000.0:
            raise ValueError(f"Line {line_number}: motor speed must be between 0 and 1000 rpm")
        if not 0 <= hold <= 32.767:
            raise ValueError(f"Line {line_number}: hold must be between 0 and 32.767 s")
        targets.append(target)
        speeds.append(speed)
        holds.append(hold)

    if len(targets) < 2:
        raise ValueError("A profile requires at least two data rows")
    if abs(targets[0]) > 1e-9:
        raise ValueError("The first platform position must be 0 degrees")
    if abs(speeds[0]) > 1e-9:
        raise ValueError("The first motor_speed_rpm value must be 0")
    for index in range(1, len(targets)):
        moving = abs(targets[index] - targets[index - 1]) > 1e-12
        if moving != (speeds[index] > 0):
            raise ValueError(
                f"Line {index + 2}: position changes require a positive motor speed; "
                "unchanged positions require 0 rpm"
            )

    # Build a time estimate for preview/timeout purposes. Drive compilation uses
    # the explicit speed and hold fields directly, not these estimated times.
    points: list[ProfilePoint] = [ProfilePoint(0.0, targets[0])]
    elapsed_s = 0.0
    if holds[0] > 0:
        elapsed_s += holds[0]
        points.append(ProfilePoint(elapsed_s, targets[0]))
    for index in range(1, len(targets)):
        delta = targets[index] - targets[index - 1]
        if abs(delta) > 1e-12:
            motor_revolutions = abs(delta) * ratio / 360.0
            duration_s = 60.0 * motor_revolutions / speeds[index]
            if not math.isfinite(duration_s) or duration_s <= 0:
                raise ValueError(f"Line {index + 2}: movement duration is not finite and positive")
            elapsed_s += duration_s
            points.append(ProfilePoint(elapsed_s, targets[index]))
        # Unchanged targets are stationary holds: their required speed is zero.
        # Keep the waypoint for execution, but add no zero-duration preview edge.
        if holds[index] > 0:
            elapsed_s += holds[index]
            points.append(ProfilePoint(elapsed_s, targets[index]))

    if not math.isfinite(elapsed_s) or elapsed_s <= 0:
        raise ValueError("A position profile requires a positive movement or hold duration")
    if any(b.time_s <= a.time_s for a, b in zip(points, points[1:])):
        raise ValueError("Position durations are too small to represent distinct preview times")

    segments = tuple(
        CompiledSegment(
            first.time_s,
            second.time_s,
            first.value,
            second.value,
            0.0 if second.value == first.value else (second.value - first.value) * ratio / 360.0 * 60.0 / (second.time_s - first.time_s),
            0.0 if second.value == first.value else (second.value - first.value) / 360.0 * 60.0 / (second.time_s - first.time_s),
        )
        for first, second in zip(points, points[1:])
    )
    return MotionProfile(
        "position",
        source,
        tuple(points),
        segments,
        ratio,
        position_path_targets_deg=tuple(targets),
        position_path_speeds_rpm=tuple(speeds),
        position_path_holds_s=tuple(holds),
    )


def interpolate_profile(profile: MotionProfile, elapsed_s: float) -> float:
    """Piecewise-linear requested value used by preview and simulation."""
    # This is nominal preview/simulation interpolation. Live speed execution
    # uses row setpoints and the drive ramp, so this curve is not measured motion.
    if elapsed_s <= 0:
        return profile.points[0].value
    if elapsed_s >= profile.duration_s:
        return profile.points[-1].value
    for first, second in zip(profile.points, profile.points[1:]):
        if elapsed_s <= second.time_s:
            fraction = (elapsed_s - first.time_s) / (second.time_s - first.time_s)
            return first.value + fraction * (second.value - first.value)
    return profile.points[-1].value


def step_profile_value(profile: MotionProfile, elapsed_s: float) -> float:
    """Return the most recent CSV value; speed commands change at row times."""
    value = profile.points[0].value
    for point in profile.points[1:]:
        if elapsed_s < point.time_s:
            break
        value = point.value
    return value


def segment_at(profile: MotionProfile, elapsed_s: float) -> CompiledSegment:
    for segment in profile.segments:
        if elapsed_s <= segment.end_s:
            return segment
    return profile.segments[-1]


def position_speed_command(
    current_platform_deg: float,
    target_platform_deg: float,
    maximum_motor_speed_rpm: float,
    ratio: float,
    travel_direction: int,
    *,
    tolerance_deg: float = 0.5,
    proportional_gain: float = 0.15,
) -> tuple[float, bool, float]:
    """Convert position error into a conservative one-direction speed command.

    The command is expressed in motor rpm in the CSV-positive direction. Once
    the target is reached or crossed, zero is returned so a caller can stop
    without reversing across drivetrain slack. The final result is positive
    overshoot in platform degrees.
    """
    if ratio <= 0:
        raise ValueError("Motor-to-platform ratio must be greater than zero")
    if maximum_motor_speed_rpm <= 0:
        raise ValueError("Waypoint motor speed must be greater than zero")
    if travel_direction not in (-1, 1):
        raise ValueError("Travel direction must be -1 or +1")
    if tolerance_deg < 0 or proportional_gain <= 0:
        raise ValueError("Tolerance must be non-negative and gain must be positive")

    directed_error = (target_platform_deg - current_platform_deg) * travel_direction
    overshoot_deg = max(0.0, -directed_error)
    if directed_error <= tolerance_deg:
        return 0.0, True, overshoot_deg

    requested_rpm = directed_error * ratio * proportional_gain
    magnitude = min(maximum_motor_speed_rpm, max(0.1, requested_rpm))
    return travel_direction * magnitude, False, 0.0


def position_waypoint_can_overlap(profile: MotionProfile, index: int) -> bool:
    """Return whether a position waypoint can be crossed without slowing."""
    targets = profile.position_path_targets_deg
    speeds = profile.position_path_speeds_rpm
    holds = profile.position_path_holds_s
    if targets is None or speeds is None or holds is None:
        return False
    if not 0 < index < len(targets) - 1:
        return False
    incoming = targets[index] - targets[index - 1]
    outgoing = targets[index + 1] - targets[index]
    return (
        holds[index] == 0
        and speeds[index] > 0
        and speeds[index + 1] > 0
        and incoming * outgoing > 0
    )
