"""Compile position CSV profiles into ASDA-B3 PR register programs.

The compiler deliberately uses incremental (INC) position paths.  A CSV
position is relative to the start of a run, so INC paths avoid silently
assuming that the drive's absolute PR origin matches the HMI session origin.


Maintenance scope: these are pure register-program compilation helpers.
The current live GUI position-profile workflow uses Speed-mode feedback rather
than these PR writes. Keep the compiler tests if retaining the legacy API.
PUU are command units after electronic gearing, not raw encoder pulses; the
gear ratio and motor/platform ratio must not be interchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import gcd, isfinite, sqrt

from .motion_profile import MotionProfile


ENCODER_PULSES_PER_REVOLUTION = 1 << 24
MAX_PR_PATHS = 99
MAX_SHARED_SLOTS = 16


@dataclass(frozen=True)
class PrRegisterWrite:
    parameter: str
    address: int
    words: int
    value: int
    signed: bool = False


@dataclass(frozen=True)
class PrPath:
    number: int
    definition: int
    delta_puu: int
    speed_slot: int
    delay_slot: int
    acceleration_slot: int
    deceleration_slot: int
    target_speed_tenths_rpm: int
    predicted_duration_s: float
    overlap_with_next: bool = False


@dataclass(frozen=True)
class PrExecutionStatus:
    raw_value: int
    path_number: int | None
    command_complete: bool
    target_reached: bool


@dataclass(frozen=True)
class PrProgram:
    egear_numerator: int
    egear_denominator: int
    motor_to_platform_ratio: float
    command_sign: int
    acceleration_ms: int
    deceleration_ms: int
    ramp_slots_ms: tuple[int, ...]
    speed_slots_tenths_rpm: tuple[int, ...]
    delay_slots_ms: tuple[int, ...]
    paths: tuple[PrPath, ...]

    @property
    def puu_per_motor_revolution(self) -> float:
        return (
            ENCODER_PULSES_PER_REVOLUTION
            * self.egear_denominator
            / self.egear_numerator
        )

    def register_writes(self) -> tuple[PrRegisterWrite, ...]:
        # The write plan is inert data. DriveController owns Servo-Off guarding,
        # backup/readback and rollback before a compiled program may be triggered.
        writes: list[PrRegisterWrite] = [
            PrRegisterWrite(f"P5.{20 + slot:03d}", 0x0528 + slot * 2, 1, value)
            for slot, value in enumerate(self.ramp_slots_ms)
        ]
        writes.extend(
            PrRegisterWrite(f"P5.{60 + slot:03d}", 0x0578 + slot * 2, 2, value)
            for slot, value in enumerate(self.speed_slots_tenths_rpm)
        )
        writes.extend(
            PrRegisterWrite(f"P5.{40 + slot:03d}", 0x0550 + slot * 2, 1, value)
            for slot, value in enumerate(self.delay_slots_ms)
        )
        for path in self.paths:
            definition_address, data_address = pr_path_addresses(path.number)
            group = 6 if path.number <= 49 else 7
            parameter_index = path.number * 2 if group == 6 else (path.number - 50) * 2
            writes.extend((
                PrRegisterWrite(
                    f"P{group}.{parameter_index:03d}", definition_address, 2,
                    path.definition,
                ),
                PrRegisterWrite(
                    f"P{group}.{parameter_index + 1:03d}", data_address, 2,
                    path.delta_puu, signed=True,
                ),
            ))
        return tuple(writes)


def puu_per_motor_revolution(egear_numerator: int, egear_denominator: int) -> float:
    if egear_numerator <= 0 or egear_denominator <= 0:
        raise ValueError("Electronic-gear numerator and denominator must be positive")
    ratio = egear_numerator / egear_denominator
    if not 1.0 <= ratio <= 262144.0:
        raise ValueError("Electronic-gear ratio N/M must be between 1 and 262144")
    return ENCODER_PULSES_PER_REVOLUTION * egear_denominator / egear_numerator


def pr_path_addresses(path_number: int) -> tuple[int, int]:
    if not 1 <= path_number <= MAX_PR_PATHS:
        raise ValueError("PR path number must be between 1 and 99")
    if path_number <= 49:
        definition = 0x0604 + (path_number - 1) * 4
    else:
        definition = 0x0700 + (path_number - 50) * 4
    return definition, definition + 2


def decode_pr_status(raw_value: int) -> PrExecutionStatus:
    if 20001 <= raw_value <= 20099:
        return PrExecutionStatus(raw_value, raw_value - 20000, True, True)
    if 10001 <= raw_value <= 10099:
        return PrExecutionStatus(raw_value, raw_value - 10000, True, False)
    if 1 <= raw_value <= 99:
        return PrExecutionStatus(raw_value, raw_value, False, False)
    return PrExecutionStatus(raw_value, None, False, False)


def _allocate_slot(values: list[int], value: int, label: str) -> int:
    # Equal values reuse a slot because the drive has only sixteen shared
    # entries per setting table; path count and distinct-setting count differ.
    try:
        return values.index(value)
    except ValueError:
        if len(values) >= MAX_SHARED_SLOTS:
            raise ValueError(f"Profile requires more than 16 distinct PR {label} settings")
        values.append(value)
        return len(values) - 1


def _speed_for_timed_move(
    motor_revolutions: float,
    duration_s: float,
    acceleration_ms: int,
    deceleration_ms: int,
) -> float:
    """Solve a stop-to-stop symmetric trapezoid for its target speed.

    P5.020 is the time from 0 to 3000 rpm.  With equal acceleration and
    deceleration, the two half-ramp areas remove one full ramp duration from
    the equivalent constant-speed move.
    """
    if motor_revolutions <= 0:
        return 0.0
    combined_base_s = (acceleration_ms + deceleration_ms) / 1000.0
    linear = duration_s / 60.0
    quadratic = combined_base_s / 360000.0
    discriminant = linear * linear - 4.0 * quadratic * motor_revolutions
    if discriminant < 0:
        raise ValueError(
            "Move cannot reach its target in the requested time with the selected PR ramp"
        )
    return (linear - sqrt(discriminant)) / (2.0 * quadratic)


def compile_position_pr_program(
    profile: MotionProfile,
    *,
    egear_numerator: int,
    egear_denominator: int,
    acceleration_ms: int,
    deceleration_ms: int,
    maximum_motor_speed_rpm: float,
    command_sign: int = 1,
    timing_tolerance_s: float = 0.025,
) -> PrProgram:
    if profile.kind != "position":
        raise ValueError("Only platform-position CSV profiles can compile to a PR program")
    if profile.pr_path_count > MAX_PR_PATHS:
        raise ValueError("Profile requires more than the 99 available PR paths")
    if not 1 <= acceleration_ms <= 65500 or not 1 <= deceleration_ms <= 65500:
        raise ValueError("PR acceleration and deceleration times must be between 1 and 65500 ms")
    if not isfinite(maximum_motor_speed_rpm) or maximum_motor_speed_rpm <= 0:
        raise ValueError("Maximum motor speed must be positive")
    if timing_tolerance_s < 0:
        raise ValueError("Timing tolerance cannot be negative")
    if command_sign not in (-1, 1):
        raise ValueError("PR command sign must be +1 or -1")

    puu_per_revolution = puu_per_motor_revolution(
        egear_numerator, egear_denominator
    )
    speed_slots: list[int] = []
    delay_slots: list[int] = []
    # Keep the operator ACC and DEC values in their established P5.020 and
    # P5.021 slots even when the numeric values are identical.
    ramp_slots: list[int] = [acceleration_ms, deceleration_ms]
    default_acceleration_slot = 0
    default_deceleration_slot = 1
    paths: list[PrPath] = []

    path_targets = profile.position_path_targets_deg
    path_speeds = profile.position_path_speeds_rpm
    path_holds = profile.position_path_holds_s
    if path_targets is not None and path_speeds is not None and path_holds is not None:
        deltas = [path_targets[0]] + [
            current - previous
            for previous, current in zip(path_targets, path_targets[1:])
        ]
        speed_tenths = [round(speed * 10.0) for speed in path_speeds]
        for index, (delta, requested_speed, quantised_speed) in enumerate(
            zip(deltas, path_speeds, speed_tenths), start=1
        ):
            if delta != 0 and quantised_speed <= 0:
                raise ValueError(f"PR path {index} motor speed rounds to zero")
            if requested_speed > maximum_motor_speed_rpm + 1e-9:
                raise ValueError(
                    f"PR path {index} requests {requested_speed:g} rpm, above the "
                    f"{maximum_motor_speed_rpm:g} rpm limit"
                )

        overlap_flags = [False] * len(path_targets)
        for index in range(len(path_targets) - 1):
            current_delta = deltas[index]
            next_delta = deltas[index + 1]
            overlap_flags[index] = (
                current_delta != 0
                and path_holds[index] == 0
                and next_delta != 0
                and (current_delta > 0) == (next_delta > 0)
            )

        acceleration_slots = [default_acceleration_slot] * len(path_targets)
        deceleration_slots = [default_deceleration_slot] * len(path_targets)
        for index, overlaps in enumerate(overlap_flags[:-1]):
            if not overlaps:
                continue
            first_speed = speed_tenths[index]
            second_speed = speed_tenths[index + 1]
            divisor = gcd(first_speed, second_speed)
            deceleration_factor = second_speed // divisor
            acceleration_factor = first_speed // divisor
            multiplier = min(
                deceleration_ms // deceleration_factor,
                acceleration_ms // acceleration_factor,
            )
            if multiplier <= 0:
                raise ValueError(
                    f"No valid matched PR ramp exists between paths {index + 1} and {index + 2}"
                )
            deceleration_slots[index] = _allocate_slot(
                ramp_slots,
                multiplier * deceleration_factor,
                "acceleration/deceleration",
            )
            acceleration_slots[index + 1] = _allocate_slot(
                ramp_slots,
                multiplier * acceleration_factor,
                "acceleration/deceleration",
            )

        for path_index, (target, delta, quantised_speed, hold_s) in enumerate(
            zip(path_targets, deltas, speed_tenths, path_holds), start=1
        ):
            del target  # The incremental delta is the drive command.
            delta_puu = round(
                delta * profile.ratio / 360.0 * puu_per_revolution * command_sign
            )
            if not -(1 << 31) <= delta_puu <= (1 << 31) - 1:
                raise ValueError(
                    f"PR path {path_index} position increment exceeds signed 32-bit PUU"
                )
            delay_ms = round(hold_s * 1000.0)
            if not 0 <= delay_ms <= 32767:
                raise ValueError(
                    f"PR path {path_index} hold time exceeds the P5.040-P5.055 limit of 32767 ms"
                )
            rounded_speed = quantised_speed / 10.0
            if delta_puu == 0:
                predicted_duration = delay_ms / 1000.0
            else:
                motor_revolutions = abs(delta) * profile.ratio / 360.0
                acceleration_time_s = (
                    quantised_speed * ramp_slots[acceleration_slots[path_index - 1]]
                    / 30_000_000.0
                )
                deceleration_time_s = (
                    quantised_speed * ramp_slots[deceleration_slots[path_index - 1]]
                    / 30_000_000.0
                )
                predicted_duration = (
                    60.0 * motor_revolutions / rounded_speed
                    + (acceleration_time_s + deceleration_time_s) / 2.0
                    + delay_ms / 1000.0
                )

            speed_slot = _allocate_slot(speed_slots, quantised_speed, "speed")
            delay_slot = _allocate_slot(delay_slots, delay_ms, "delay")
            option = 0xA if overlap_flags[path_index - 1] else 0x8
            path_type = 2 if path_index == len(path_targets) else 3
            definition = (
                (delay_slot << 20)
                | (speed_slot << 16)
                | (deceleration_slots[path_index - 1] << 12)
                | (acceleration_slots[path_index - 1] << 8)
                | (option << 4)
                | path_type
            )
            paths.append(
                PrPath(
                    path_index,
                    definition,
                    delta_puu,
                    speed_slot,
                    delay_slot,
                    acceleration_slots[path_index - 1],
                    deceleration_slots[path_index - 1],
                    quantised_speed,
                    predicted_duration,
                    overlap_flags[path_index - 1],
                )
            )
        return PrProgram(
            egear_numerator,
            egear_denominator,
            profile.ratio,
            command_sign,
            acceleration_ms,
            deceleration_ms,
            tuple(ramp_slots),
            tuple(speed_slots),
            tuple(delay_slots),
            tuple(paths),
        )

    explicit_member: dict[int, tuple[int, float, bool, int, int]] = {}
    explicit_speeds = profile.segment_target_speeds_rpm
    if explicit_speeds is not None:
        run_start = 0
        while run_start < len(profile.segments):
            first = profile.segments[run_start]
            first_delta = first.end_value - first.start_value
            if first_delta == 0:
                run_start += 1
                continue
            direction = 1 if first_delta > 0 else -1
            run_end = run_start + 1
            while run_end < len(profile.segments):
                candidate = profile.segments[run_end]
                delta = candidate.end_value - candidate.start_value
                if delta == 0 or (1 if delta > 0 else -1) != direction:
                    break
                run_end += 1

            speeds_tenths = [round(explicit_speeds[i] * 10.0) for i in range(run_start, run_end)]
            if any(speed <= 0 for speed in speeds_tenths):
                raise ValueError("Moving intervals require a positive segment motor speed")
            if any(speed / 10.0 > maximum_motor_speed_rpm + 1e-9 for speed in speeds_tenths):
                raise ValueError(
                    f"Explicit segment speed exceeds the {maximum_motor_speed_rpm:g} rpm limit"
                )

            acceleration_slots = [default_acceleration_slot] * len(speeds_tenths)
            deceleration_slots = [default_deceleration_slot] * len(speeds_tenths)
            for offset in range(len(speeds_tenths) - 1):
                first_speed = speeds_tenths[offset]
                second_speed = speeds_tenths[offset + 1]
                divisor = gcd(first_speed, second_speed)
                deceleration_factor = second_speed // divisor
                acceleration_factor = first_speed // divisor
                multiplier = min(
                    deceleration_ms // deceleration_factor,
                    acceleration_ms // acceleration_factor,
                )
                if multiplier <= 0:
                    raise ValueError("No valid matched PR ramp exists for an overlap transition")
                matched_deceleration_ms = multiplier * deceleration_factor
                matched_acceleration_ms = multiplier * acceleration_factor
                deceleration_slots[offset] = _allocate_slot(
                    ramp_slots,
                    matched_deceleration_ms,
                    "acceleration/deceleration",
                )
                acceleration_slots[offset + 1] = _allocate_slot(
                    ramp_slots,
                    matched_acceleration_ms,
                    "acceleration/deceleration",
                )

            command_start_s = 0.0
            previous_waypoint_s = 0.0
            predicted_durations: list[float] = []
            for offset, segment_index in enumerate(range(run_start, run_end)):
                segment = profile.segments[segment_index]
                speed_tenths = speeds_tenths[offset]
                speed_rpm = speed_tenths / 10.0
                motor_revolutions = (
                    abs(segment.end_value - segment.start_value) * profile.ratio / 360.0
                )
                acceleration_time_s = (
                    speed_tenths * ramp_slots[acceleration_slots[offset]] / 30_000_000.0
                )
                deceleration_time_s = (
                    speed_tenths * ramp_slots[deceleration_slots[offset]] / 30_000_000.0
                )
                if offset < len(speeds_tenths) - 1:
                    next_command_start_s = (
                        command_start_s
                        + 60.0 * motor_revolutions / speed_rpm
                        + (acceleration_time_s - deceleration_time_s) / 2.0
                    )
                    next_speed_rpm = speeds_tenths[offset + 1] / 10.0
                    crossing_fraction = sqrt(speed_rpm) / (
                        sqrt(speed_rpm) + sqrt(next_speed_rpm)
                    )
                    waypoint_s = (
                        next_command_start_s + crossing_fraction * deceleration_time_s
                    )
                    command_start_s = next_command_start_s
                else:
                    waypoint_s = (
                        command_start_s
                        + 60.0 * motor_revolutions / speed_rpm
                        + (acceleration_time_s + deceleration_time_s) / 2.0
                    )
                predicted_durations.append(waypoint_s - previous_waypoint_s)
                previous_waypoint_s = waypoint_s

            for offset, segment_index in enumerate(range(run_start, run_end)):
                requested_duration = (
                    profile.segments[segment_index].end_s
                    - profile.segments[segment_index].start_s
                )
                predicted_duration = predicted_durations[offset]
                if abs(predicted_duration - requested_duration) > timing_tolerance_s:
                    raise ValueError(
                        f"PR path {segment_index + 1} variable-overlap timing error is "
                        f"{abs(predicted_duration - requested_duration):.3f} s"
                    )
                explicit_member[segment_index] = (
                    speeds_tenths[offset],
                    predicted_duration,
                    offset < len(speeds_tenths) - 1,
                    acceleration_slots[offset],
                    deceleration_slots[offset],
                )
            run_start = run_end

    # Build conservative overlap groups.  A group may contain adjacent moves in
    # the same direction only when one quantised target speed reproduces every
    # CSV waypoint time.  This is the subset of OVLP that needs no extra shared
    # acceleration/deceleration slots: equal target speed plus equal ACC/DEC
    # time keeps the transition continuous, as specified in ASDA-B3 7.1.6.
    overlap_groups: dict[int, tuple[int, int, tuple[float, ...]]] = {}
    group_start = 0
    while group_start < len(profile.segments):
        if explicit_speeds is not None:
            break
        first_segment = profile.segments[group_start]
        first_delta = first_segment.end_value - first_segment.start_value
        if first_delta == 0:
            group_start += 1
            continue
        direction = 1 if first_delta > 0 else -1
        group_end = group_start + 1
        while group_end < len(profile.segments):
            candidate = profile.segments[group_end]
            candidate_delta = candidate.end_value - candidate.start_value
            if candidate_delta == 0 or (1 if candidate_delta > 0 else -1) != direction:
                break
            group_end += 1

        if group_end - group_start > 1:
            members = profile.segments[group_start:group_end]
            total_duration = sum(item.end_s - item.start_s for item in members)
            total_motor_revolutions = sum(
                abs(item.end_value - item.start_value) * profile.ratio / 360.0
                for item in members
            )
            try:
                group_speed = _speed_for_timed_move(
                    total_motor_revolutions,
                    total_duration,
                    acceleration_ms,
                    deceleration_ms,
                )
            except ValueError:
                group_speed = 0.0
            target_speed_tenths = round(group_speed * 10.0)
            if target_speed_tenths > 0:
                rounded_speed = target_speed_tenths / 10.0
                acceleration_s = acceleration_ms / 1000.0 * rounded_speed / 3000.0
                deceleration_s = deceleration_ms / 1000.0 * rounded_speed / 3000.0
                predicted_durations = []
                for offset, item in enumerate(members):
                    motor_revolutions = (
                        abs(item.end_value - item.start_value) * profile.ratio / 360.0
                    )
                    predicted = 60.0 * motor_revolutions / rounded_speed
                    if offset == 0:
                        predicted += acceleration_s / 2.0
                    if offset == len(members) - 1:
                        predicted += deceleration_s / 2.0
                    predicted_durations.append(predicted)
                if all(
                    abs(predicted - (item.end_s - item.start_s)) <= timing_tolerance_s
                    for predicted, item in zip(predicted_durations, members)
                ):
                    overlap_groups[group_start] = (
                        group_end,
                        target_speed_tenths,
                        tuple(predicted_durations),
                    )
        group_start = group_end

    overlap_member: dict[int, tuple[int, float, bool]] = {}
    for start, (end, speed_tenths, durations) in overlap_groups.items():
        for offset, segment_index in enumerate(range(start, end)):
            overlap_member[segment_index] = (
                speed_tenths,
                durations[offset],
                segment_index < end - 1,
            )

    for segment_index, segment in enumerate(profile.segments):
        index = segment_index + 1
        duration_s = segment.end_s - segment.start_s
        platform_delta_degrees = segment.end_value - segment.start_value
        motor_revolutions = abs(platform_delta_degrees) * profile.ratio / 360.0
        delta_puu = round(
            platform_delta_degrees
            * profile.ratio
            / 360.0
            * puu_per_revolution
            * command_sign
        )
        if not -(1 << 31) <= delta_puu <= (1 << 31) - 1:
            raise ValueError(f"PR path {index} position increment exceeds signed 32-bit PUU")

        overlap_with_next = False
        acceleration_slot = default_acceleration_slot
        deceleration_slot = default_deceleration_slot
        if delta_puu == 0:
            target_speed_tenths = 0
            delay_ms = round(duration_s * 1000.0)
            if not 0 <= delay_ms <= 32767:
                raise ValueError(f"PR path {index} hold time exceeds the P5.040-P5.055 limit of 32767 ms")
            predicted_duration = delay_ms / 1000.0
        elif segment_index in explicit_member:
            (
                target_speed_tenths,
                predicted_duration,
                overlap_with_next,
                acceleration_slot,
                deceleration_slot,
            ) = explicit_member[segment_index]
            delay_ms = 0
        elif segment_index in overlap_member:
            target_speed_tenths, predicted_duration, overlap_with_next = overlap_member[
                segment_index
            ]
            rounded_speed = target_speed_tenths / 10.0
            if rounded_speed > maximum_motor_speed_rpm + 1e-9:
                raise ValueError(
                    f"PR path {index} requires {rounded_speed:g} rpm, above the "
                    f"{maximum_motor_speed_rpm:g} rpm limit"
                )
            delay_ms = 0
        else:
            target_speed = _speed_for_timed_move(
                motor_revolutions, duration_s, acceleration_ms, deceleration_ms
            )
            target_speed_tenths = round(target_speed * 10.0)
            if target_speed_tenths <= 0:
                raise ValueError(f"PR path {index} target speed rounds to zero")
            rounded_speed = target_speed_tenths / 10.0
            if rounded_speed > maximum_motor_speed_rpm + 1e-9:
                raise ValueError(
                    f"PR path {index} requires {rounded_speed:g} rpm, above the "
                    f"{maximum_motor_speed_rpm:g} rpm limit"
                )
            predicted_duration = (
                60.0 * motor_revolutions / rounded_speed
                + (acceleration_ms + deceleration_ms)
                / 1000.0
                * rounded_speed
                / 6000.0
            )
            if abs(predicted_duration - duration_s) > timing_tolerance_s:
                raise ValueError(
                    f"PR path {index} timing error is "
                    f"{abs(predicted_duration - duration_s):.3f} s after 0.1 rpm quantisation"
                )
            delay_ms = 0

        speed_slot = _allocate_slot(speed_slots, target_speed_tenths, "speed")
        delay_slot = _allocate_slot(delay_slots, delay_ms, "delay")

        # Definition nibbles are [D C B A U Z Y X].  U/Z select shared
        # deceleration slot 1 / acceleration slot 0.  Y=8 is INC; bit 1 enables
        # OVLP, producing Y=A.  Intermediate paths are AUTO (X=3), final is
        # SINGLE (2).
        path_type = 2 if index == len(profile.segments) else 3
        option = 0xA if overlap_with_next else 0x8
        definition = (
            (delay_slot << 20)
            | (speed_slot << 16)
            | (deceleration_slot << 12)
            | (acceleration_slot << 8)
            | (option << 4)
            | path_type
        )
        paths.append(
            PrPath(
                index,
                definition,
                delta_puu,
                speed_slot,
                delay_slot,
                acceleration_slot,
                deceleration_slot,
                target_speed_tenths,
                predicted_duration,
                overlap_with_next,
            )
        )

    return PrProgram(
        egear_numerator,
        egear_denominator,
        profile.ratio,
        command_sign,
        acceleration_ms,
        deceleration_ms,
        tuple(ramp_slots),
        tuple(speed_slots),
        tuple(delay_slots),
        tuple(paths),
    )
