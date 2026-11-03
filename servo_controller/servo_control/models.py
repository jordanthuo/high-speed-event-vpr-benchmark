"""Immutable drive snapshots and decoded status/engineering values.

Raw register fields retain their drive meanings; None marks unavailable
monitor data, whereas zero is a real measurement. DI channel arguments are
one-based. Logical DI states already include configured contact polarity,
which is particularly important for normally-closed EMGS.
Snapshot angle helpers convert a single raw count; continuous rollover-aware
angle history belongs to TelemetryBuffer, not these helpers.
"""

from dataclasses import dataclass
from enum import Enum


class ConnectionState(str, Enum):
    DISCONNECTED = "Disconnected"
    CONNECTING = "Connecting"
    CONNECTED = "Connected"
    FAULT = "Fault"


class OperatingMode(str, Enum):
    SIMULATION = "Simulation"
    LIVE = "Live Drive"


@dataclass(frozen=True)
class MotionTelemetry:
    """One timestamped contiguous read of speed and encoder position."""

    sampled_monotonic_s: float
    acquisition_duration_s: float
    integrated_command_rpm: float
    feedback_speed_rpm: float
    encoder_position_pulses: int


@dataclass(frozen=True)
class DriveSnapshot:
    p3_002: int
    p3_006: int
    p4_007: int
    p4_009: int
    drive_status: int
    alarm_code: int
    requested_speed_rpm: float
    integrated_command_rpm: float | None
    feedback_speed_rpm: float | None
    encoder_position_pulses: int | None
    monitor_1_selection: int
    monitor_2_selection: int
    monitor_3_selection: int
    simulated: bool
    hardware_motion_enabled: bool = False
    emgs_channels: tuple[int, ...] = ()
    emgs_normally_closed: bool = False

    @property
    def mode(self) -> OperatingMode:
        return OperatingMode.SIMULATION if self.simulated else OperatingMode.LIVE

    @property
    def di6_software_selected(self) -> bool:
        return bool(self.p3_006 & 0x0020)

    @property
    def di7_software_selected(self) -> bool:
        return bool(self.p3_006 & 0x0040)

    @property
    def di8_software_selected(self) -> bool:
        return bool(self.p3_006 & 0x0080)

    @property
    def di6_on(self) -> bool:
        return bool(self.p4_007 & 0x0020)

    @property
    def di7_on(self) -> bool:
        return bool(self.p4_007 & 0x0040)

    @property
    def di8_on(self) -> bool:
        return bool(self.p4_007 & 0x0080)

    def digital_input_on(self, channel: int) -> bool:
        if not 1 <= channel <= 13:
            raise ValueError("Digital input channel must be between 1 and 13")
        return bool(self.p4_007 & (1 << (channel - 1)))

    def digital_input_software_selected(self, channel: int) -> bool:
        if not 1 <= channel <= 13:
            raise ValueError("Digital input channel must be between 1 and 13")
        return bool(self.p3_006 & (1 << (channel - 1)))

    def digital_output_on(self, channel: int) -> bool:
        if not 1 <= channel <= 6:
            raise ValueError("Digital output channel must be between 1 and 6")
        return bool(self.p4_009 & (1 << (channel - 1)))

    @property
    def digital_inputs_on_count(self) -> int:
        return sum(self.digital_input_on(channel) for channel in range(1, 14))

    @property
    def emgs_hardware_healthy(self) -> bool:
        if len(self.emgs_channels) != 1 or not self.emgs_normally_closed:
            return False
        channel = self.emgs_channels[0]
        # ASDA-Soft/P4.007 reports the logical DI function state after contact
        # polarity. For a normally-closed EMGS assignment, OFF means the
        # circuit is healthy; ON means the emergency-stop function is active.
        return (
            not self.digital_input_software_selected(channel)
            and not self.digital_input_on(channel)
        )

    @property
    def digital_outputs_on_count(self) -> int:
        return sum(self.digital_output_on(channel) for channel in range(1, 7))

    @property
    def servo_ready(self) -> bool:
        return bool(self.drive_status & 0x0001)

    @property
    def servo_on(self) -> bool:
        return bool(self.drive_status & 0x0002)

    @property
    def drive_ready_indication(self) -> bool:
        """Dashboard-ready indication: the drive is ready and Servo On is active."""
        return self.servo_ready and self.servo_on

    @property
    def zero_speed(self) -> bool:
        return bool(self.drive_status & 0x0004)

    @property
    def target_speed_reached(self) -> bool:
        return bool(self.drive_status & 0x0008)

    @property
    def alarm_active(self) -> bool:
        return self.alarm_code != 0 or bool(self.drive_status & 0x0040)

    @property
    def command_speed_rpm(self) -> float:
        """Backward-compatible alias for the requested P1.009 setpoint."""
        return self.requested_speed_rpm

    @property
    def tracking_error_rpm(self) -> float | None:
        if self.integrated_command_rpm is None or self.feedback_speed_rpm is None:
            return None
        return self.integrated_command_rpm - self.feedback_speed_rpm

    def platform_speed_rpm(self, motor_to_platform_ratio: float = 2.0) -> float | None:
        if self.feedback_speed_rpm is None or motor_to_platform_ratio <= 0:
            return None
        return self.feedback_speed_rpm / motor_to_platform_ratio

    def motor_angle_degrees(self, encoder_pulses_per_revolution: int = 1 << 24) -> float | None:
        if self.encoder_position_pulses is None or encoder_pulses_per_revolution <= 0:
            return None
        return self.encoder_position_pulses * 360.0 / encoder_pulses_per_revolution

    def platform_angle_degrees(
        self,
        motor_to_platform_ratio: float = 2.0,
        encoder_pulses_per_revolution: int = 1 << 24,
    ) -> float | None:
        motor_angle = self.motor_angle_degrees(encoder_pulses_per_revolution)
        if motor_angle is None or motor_to_platform_ratio <= 0:
            return None
        return motor_angle / motor_to_platform_ratio

    @property
    def speed_mode_selected(self) -> bool:
        # In PR/S dual mode, S-P OFF selects Speed mode.
        return not self.di7_on

    @property
    def internal_speed_selected(self) -> bool:
        # SPD0 ON with SPD1 OFF selects P1.009.
        return self.di6_on

    @property
    def configuration_issues(self) -> tuple[str, ...]:
        issues: list[str] = []
        if (self.p3_006 & 0x0060) != 0x0060:
            issues.append("P3.006 does not select software control for DI6 and DI7")
        if len(self.emgs_channels) != 1:
            issues.append("Exactly one DI must be assigned EMGS")
        elif not self.emgs_normally_closed:
            issues.append(f"DI{self.emgs_channels[0]}/EMGS is not normally closed")
        elif self.digital_input_software_selected(self.emgs_channels[0]):
            issues.append(f"DI{self.emgs_channels[0]}/EMGS is software-controlled")
        if self.monitor_1_selection != 9:
            issues.append("P0.017 is not 9, so integrated command monitoring is unavailable")
        if self.monitor_2_selection != 7:
            issues.append("P0.018 is not 7, so live speed feedback is unavailable")
        if self.monitor_3_selection != 3:
            issues.append("P0.019 is not 3, so encoder-position monitoring is unavailable")
        return tuple(issues)
