"""Guarded application boundary between GUI workflows and drive registers.

All GUI motion and parameter writes should use this service, not the transport
directly. _serial_io holds a reentrant lock across a whole operation, including
read/modify/write and verification; transport-level locks alone cannot protect
such sequences. Live permission is per connection and is revoked on faults.

Read this file in layers: connection and PR helpers; status/acquisition;
parameter and speed commands; live readiness; digital I/O and mode selection.
PR helpers remain supported infrastructure, but the current GUI CSV position
workflow uses encoder-controlled Speed mode. Mock tests establish software
contracts, not physical stopping distance or independent safety certification.
"""

from collections.abc import Callable
from functools import wraps
from dataclasses import replace
from threading import RLock
from time import monotonic, perf_counter

from .config import SerialConfig
from .models import ConnectionState, DriveSnapshot, MotionTelemetry
from .pr_program import PrExecutionStatus, PrProgram, PrRegisterWrite, decode_pr_status
from .registers import (
    DI_PLANNING_REGISTERS,
    DO_PLANNING_REGISTERS,
    MONITOR_COMMAND_CODE,
    MONITOR_FEEDBACK_CODE,
    PROJECT_PARAMETER_CATALOG,
    REGISTER_BY_PARAMETER,
    P0_001,
    P0_009,
    P0_010,
    P0_011,
    P0_017,
    P0_018,
    P0_019,
    P0_046,
    P1_001,
    P1_009,
    P1_034,
    P1_035,
    P1_044,
    P1_045,
    P1_055,
    P1_111,
    P3_002,
    P3_005,
    P3_006,
    P4_006,
    P4_007,
    P4_009,
    P5_007,
    Register,
    Persistence,
)
from .transport import (
    MinimalModbusTransport,
    MockTransport,
    RegisterTransport,
    value_to_words,
    words_to_value,
)


def _serial_io(method: Callable[..., object]) -> Callable[..., object]:
    """Serialize complete drive operations; serial Modbus clients are not thread-safe."""
    @wraps(method)
    def guarded(self: "DriveController", *args: object, **kwargs: object) -> object:
        with self._io_lock:
            return method(self, *args, **kwargs)

    return guarded


class MotionLockedError(RuntimeError):
    pass


class DriveController:
    """Application service and the single safety gate for all drive access."""

    MIN_SPEED_RPM = -1000.0
    MAX_SPEED_RPM = 1000.0
    DEFAULT_OPERATOR_LIMIT_RPM = 100.0
    DI_CHANNEL_COUNT = 13
    DI_MASK = 0x1FFF
    SON_FUNCTION_CODE = 0x01
    EMGS_FUNCTION_CODE = 0x21

    def __init__(
        self,
        transport_factory: Callable[[SerialConfig, bool], RegisterTransport] | None = None,
    ) -> None:
        self.state = ConnectionState.DISCONNECTED
        self.config = SerialConfig()
        self.simulated = True
        self.transport: RegisterTransport | None = None
        self.last_error: str | None = None
        self._transport_factory = transport_factory or self._default_transport
        self._io_lock = RLock()
        self._di_planning_cache: tuple[int, ...] = ()
        self._speed_mode_selected = False
        self._last_speed_command_word: int | None = None

        # Live motion is session-only and can become true only through the
        # guarded live-test arming workflow. It is cleared by disconnect/fault.
        self.hardware_motion_enabled = False
        self._verified_pr_program: PrProgram | None = None
        self._pr_register_backup: tuple[PrRegisterWrite, ...] = ()

    @staticmethod
    def _default_transport(config: SerialConfig, simulated: bool) -> RegisterTransport:
        return MockTransport() if simulated else MinimalModbusTransport(config)

    @property
    def cached_speed_command_rpm(self) -> float:
        return 0.0 if self._last_speed_command_word is None else self._last_speed_command_word / 10.0

    @_serial_io
    def connect(self, config: SerialConfig, simulated: bool = True) -> None:
        if self.state is ConnectionState.CONNECTED:
            self.disconnect()
        config.validate()
        self.state = ConnectionState.CONNECTING
        self.last_error = None
        transport = self._transport_factory(config, simulated)
        try:
            transport.connect()
            self.transport = transport
            self.config = config
            self.simulated = simulated
            if simulated:
                # Match the rig startup: Speed mode, S-P OFF, zero command.
                self._write_value(P1_009, 0)
                p4_007 = self._read_value(P4_007)
                self._write_value(P4_007, (p4_007 | 0x0020) & ~0x0040)
            self._speed_mode_selected = simulated
            self._last_speed_command_word = 0
            self._refresh_di_planning_cache()
            if not simulated:
                self._ensure_speed_only_di_sources()
        except Exception as exc:
            transport.close()
            self.transport = None
            self.state = ConnectionState.FAULT
            self.last_error = str(exc)
            raise
        self.state = ConnectionState.CONNECTED

    def _ensure_speed_only_di_sources(self) -> None:
        """Software-source SPD0 and hardware-source unwired S-P.

        This connection-time operation is deliberately permitted only while the
        drive reports zero speed. DI7 must be normally-open S-P; its logical
        state is displayed but is not an interlock. Neither mode input may be
        assigned SON/EMGS.
        """
        speed_source = 0x0020  # DI6 / SPD0
        position_source = 0x0040  # DI7 / S-P
        critical_codes = {self.SON_FUNCTION_CODE, self.EMGS_FUNCTION_CODE}
        conflicts = tuple(
            channel
            for channel in (6, 7)
            if (self._di_planning_cache[channel - 1] & 0x00FF) in critical_codes
        )
        if conflicts:
            labels = ", ".join(f"DI{channel}" for channel in conflicts)
            raise MotionLockedError(
                f"Cannot select software control for {labels}: its assigned "
                "function is SON or EMGS"
            )
        if not self._read_value(P0_046) & 0x0004:
            raise MotionLockedError(
                "Zero-speed indication is required before selecting software "
                "control for DI6 and hardware control for DI7"
            )
        if self._di_planning_cache[6] != 0x0118:
            raise MotionLockedError("DI7/S-P must be normally open (P2.016 = 0x0118) for unwired Speed-only operation")
        # A previously selected PR branch may expose a stale nonzero speed
        # command when S-P goes OFF. Zero and verify it before switching sources.
        self._write_value(P1_009, 0)
        if self._read_value(P1_009) != 0:
            raise MotionLockedError("Cannot select Speed-only inputs: P1.009 did not read back zero")
        combined = self._read_value(P4_007)
        self._write_value(P4_007, (combined | speed_source) & ~position_source)
        current = self._read_value(P3_006)
        updated = (current | speed_source) & ~position_source
        if updated != current:
            self._write_value(P3_006, updated)
        readback = self._read_value(P3_006)
        if readback != updated:
            raise RuntimeError(
                f"P3.006 verification failed: wrote 0x{updated:04X}, "
                f"read back 0x{readback:04X}"
            )
        self._speed_mode_selected = True

    @_serial_io
    def disconnect(self) -> None:
        if self.transport is not None:
            if self.simulated or self.hardware_motion_enabled:
                try:
                    if not self.simulated and self._verified_pr_program is not None:
                        self._write_value(P5_007, 1000)
                    self._write_value(P1_009, 0)
                except Exception:
                    pass
            self.transport.close()
        self.transport = None
        self._di_planning_cache = ()
        self._speed_mode_selected = False
        self._last_speed_command_word = None
        self.hardware_motion_enabled = False
        self._verified_pr_program = None
        self._pr_register_backup = ()
        self.state = ConnectionState.DISCONNECTED

    @_serial_io
    def read_pr_scaling(self) -> tuple[int, int]:
        """Return the live electronic-gear numerator and denominator."""
        self._require_connection()
        return self._read_value(P1_044), self._read_value(P1_045)

    @_serial_io
    def read_pr_setup(self) -> tuple[int, int, int]:
        """Return E-gear N/M and P1.001.Z direction control."""
        self._require_connection()
        direction_control = (self._read_value(P1_001) >> 8) & 0xF
        if direction_control not in (0, 1):
            raise RuntimeError(f"P1.001.Z has unsupported direction value {direction_control}")
        return self._read_value(P1_044), self._read_value(P1_045), direction_control

    def _read_pr_write(self, write: PrRegisterWrite) -> int:
        assert self.transport is not None
        words = self.transport.read_registers(write.address, write.words)
        return words_to_value(words, signed=write.signed)

    def _write_pr_write(self, write: PrRegisterWrite) -> None:
        assert self.transport is not None
        self.transport.write_registers(write.address, value_to_words(write.value, write.words))

    def _require_pr_write_safe_state(self) -> None:
        if self._read_value(P1_009) != 0:
            raise MotionLockedError("Set P1.009 to 0 rpm before writing a PR program")
        status = self._read_value(P0_046)
        if not status & 0x0004:
            raise MotionLockedError("Zero-speed indication is required before writing a PR program")
        if not self.simulated and status & 0x0002:
            raise MotionLockedError("Servo Off is required before writing a live PR program")
        if not self.simulated and (self._read_value(P1_001) & 0x000F) != 0x0008:
            raise MotionLockedError("P1.001.X must select the project PR/S control mode (8)")
        if self._read_value(P3_005) != 0:
            raise MotionLockedError("P3.005 must be 0 for verified low-word-first 32-bit PR writes")

    @_serial_io
    def write_and_verify_pr_program(self, program: PrProgram) -> tuple[PrRegisterWrite, ...]:
        """Transactionally stage a compiled PR program without starting motion."""
        # Treat backup, write and readback as one serialized operation. A partial
        # program must not become the verified program eligible for triggering.
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError("Live PR programs are disabled; use Speed-mode position control")
        self._require_pr_write_safe_state()
        if not program.paths:
            raise ValueError("PR program must contain at least one path")
        scaling = self.read_pr_scaling()
        if scaling != (program.egear_numerator, program.egear_denominator):
            raise RuntimeError(
                "Electronic-gear scaling changed after compilation: "
                f"compiled {program.egear_numerator}/{program.egear_denominator}, "
                f"drive reports {scaling[0]}/{scaling[1]}"
            )

        writes = program.register_writes()
        backup = tuple(
            PrRegisterWrite(
                item.parameter,
                item.address,
                item.words,
                self._read_pr_write(item),
                item.signed,
            )
            for item in writes
        )
        try:
            for item in writes:
                self._write_pr_write(item)
                readback = self._read_pr_write(item)
                if readback != item.value:
                    raise RuntimeError(
                        f"{item.parameter} verification failed: wrote {item.value}, "
                        f"read back {readback}"
                    )
        except Exception as write_error:
            restore_errors: list[str] = []
            for item in reversed(backup):
                try:
                    self._write_pr_write(item)
                    if self._read_pr_write(item) != item.value:
                        restore_errors.append(f"{item.parameter} readback mismatch")
                except Exception as restore_error:
                    restore_errors.append(f"{item.parameter}: {restore_error}")
            self._verified_pr_program = None
            if restore_errors:
                raise RuntimeError(
                    f"PR write failed ({write_error}); rollback also failed: "
                    + "; ".join(restore_errors)
                ) from write_error
            raise RuntimeError(f"PR write failed and was rolled back: {write_error}") from write_error

        self._pr_register_backup = backup
        self._verified_pr_program = program
        return tuple(
            PrRegisterWrite(
                item.parameter,
                item.address,
                item.words,
                self._read_pr_write(item),
                item.signed,
            )
            for item in writes
        )

    @_serial_io
    def restore_pr_program_backup(self) -> None:
        """Restore the PR registers captured before the last successful stage."""
        self._require_connection()
        self._require_pr_write_safe_state()
        if not self._pr_register_backup:
            raise RuntimeError("No same-session PR register backup is available")
        for item in reversed(self._pr_register_backup):
            self._write_pr_write(item)
            if self._read_pr_write(item) != item.value:
                raise RuntimeError(f"{item.parameter} backup restore verification failed")
        self._verified_pr_program = None
        self._pr_register_backup = ()

    @_serial_io
    def trigger_verified_pr_program(self) -> None:
        """Start PR#1 only after the existing live-motion arming workflow."""
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError("Live PR starts are disabled; use Speed-mode position control")
        self._require_motion_permission("PR program start")
        if self._verified_pr_program is None:
            raise MotionLockedError("No PR program has been written and verified this session")
        self._write_value(P1_009, 0)
        p4_007 = self._read_value(P4_007)
        self._write_value(P4_007, (p4_007 | 0x0040) & ~0x0020)
        self._write_value(P5_007, 1)

    @_serial_io
    def read_pr_execution_status(self) -> PrExecutionStatus:
        self._require_connection()
        return decode_pr_status(self._read_value(P5_007))

    @_serial_io
    def stop_pr_program(self) -> None:
        """Request the ASDA-B3 PR stop command; always available when connected."""
        self._require_connection()
        self._write_value(P5_007, 1000)
        self.hardware_motion_enabled = False

    @_serial_io
    def read_snapshot(self) -> DriveSnapshot:
        self._require_connection()
        monitor_1_selection = self._read_value(P0_017)
        monitor_2_selection = self._read_value(P0_018)
        monitor_3_selection = self._read_value(P0_019)
        # The three 32-bit monitor values are contiguous. One six-word request
        # keeps command, speed and position coherent and avoids three RTU
        # transactions delaying the dedicated high-rate sampler.
        assert self.transport is not None
        motion_words = self.transport.read_registers(P0_009.address, 6)
        integrated_command = (
            words_to_value(motion_words[0:2], signed=True) / 10.0
            if monitor_1_selection == MONITOR_COMMAND_CODE else None
        )
        feedback_speed = (
            words_to_value(motion_words[2:4], signed=True) / 10.0
            if monitor_2_selection == MONITOR_FEEDBACK_CODE else None
        )
        encoder_position_pulses = (
            words_to_value(motion_words[4:6], signed=True)
            if monitor_3_selection == 3 else None
        )
        snapshot = DriveSnapshot(
            p3_002=self._read_value(P3_002),
            p3_006=self._read_value(P3_006),
            p4_007=self._read_value(P4_007),
            p4_009=self._read_value(P4_009),
            drive_status=self._read_value(P0_046),
            alarm_code=self._read_value(P0_001),
            requested_speed_rpm=self._read_value(P1_009) / 10.0,
            integrated_command_rpm=integrated_command,
            feedback_speed_rpm=feedback_speed,
            encoder_position_pulses=encoder_position_pulses,
            monitor_1_selection=monitor_1_selection,
            monitor_2_selection=monitor_2_selection,
            monitor_3_selection=monitor_3_selection,
            simulated=self.simulated,
            hardware_motion_enabled=self.hardware_motion_enabled,
            emgs_channels=self._emgs_channels(),
            emgs_normally_closed=self._emgs_normally_closed(),
        )
        if (
            not self.simulated
            and self.hardware_motion_enabled
            and (
                snapshot.alarm_active
                or not snapshot.drive_ready_indication
                or not snapshot.emgs_hardware_healthy
                or snapshot.di7_software_selected
            )
        ):
            try:
                if self._verified_pr_program is not None:
                    self._write_value(P5_007, 1000)
                self._write_value(P1_009, 0)
            except Exception:
                pass
            self.hardware_motion_enabled = False
            snapshot = replace(
                snapshot,
                requested_speed_rpm=0.0,
                hardware_motion_enabled=False,
            )
        return snapshot

    @_serial_io
    def read_motion_telemetry(self) -> MotionTelemetry:
        """Read speed and position atomically in one four-word RTU request.

        The requested speed is already cached when the GUI writes P1.009, so
        re-reading the two P0.009 words on every sample only increases serial
        transfer time. The cached request is retained as the command trace.
        """
        # The fast path reads P0.010/P0.011 only. Despite the field name,
        # integrated_command_rpm below is the cached requested setpoint, not a fresh
        # P0.009 measurement. The slow snapshot path does read that monitor.
        # Midpoint timing brackets the RTU call; it is not a hardware latch time.
        self._require_connection()
        assert self.transport is not None
        started = perf_counter()
        words = self.transport.read_registers(P0_010.address, 4)
        finished = perf_counter()
        if len(words) != 4:
            raise RuntimeError(f"Motion telemetry returned {len(words)} words; expected 4")
        return MotionTelemetry(
            sampled_monotonic_s=(started + finished) / 2.0,
            acquisition_duration_s=finished - started,
            integrated_command_rpm=self.cached_speed_command_rpm,
            feedback_speed_rpm=words_to_value(words[0:2], signed=True) / 10.0,
            encoder_position_pulses=words_to_value(words[2:4], signed=True),
        )

    @_serial_io
    def read_speed_ramp_rates(self) -> tuple[float, float]:
        """Return actual drive acceleration/deceleration rates in motor rpm/s."""
        self._require_connection()
        acceleration_ms = self._read_value(P1_034)
        deceleration_ms = self._read_value(P1_035)
        if acceleration_ms <= 0 or deceleration_ms <= 0:
            raise RuntimeError("P1.034/P1.035 must be greater than zero")
        return 3_000_000.0 / acceleration_ms, 3_000_000.0 / deceleration_ms

    @_serial_io
    def read_project_parameters(self) -> dict[str, int]:
        """Read the manual-verified, project-relevant parameter catalog."""
        self._require_connection()
        return {
            register.parameter: self._read_value(register)
            for register in PROJECT_PARAMETER_CATALOG
        }

    @_serial_io
    def write_parameter(self, parameter: str, value: int) -> int:
        """Write and verify one catalog parameter in Simulation or Live Drive mode."""
        self._require_connection()
        return self._write_catalog_parameter(parameter, value, allow_live=True)

    @_serial_io
    def write_simulation_parameter(self, parameter: str, value: int) -> int:
        """Backward-compatible simulator-only parameter write API."""
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError("This API is available only in Simulation mode")
        return self._write_catalog_parameter(parameter, value, allow_live=False)

    def _write_catalog_parameter(self, parameter: str, value: int, *, allow_live: bool) -> int:
        try:
            register = REGISTER_BY_PARAMETER[parameter]
        except KeyError as exc:
            raise ValueError(f"Unknown project parameter: {parameter}") from exc
        if register.persistence is Persistence.READ_ONLY:
            raise MotionLockedError(f"{parameter} is read-only")
        if not isinstance(value, int):
            raise TypeError("Parameter value must be an integer raw register value")
        bits = register.words * 16
        storage_minimum = -(1 << (bits - 1)) if register.signed else 0
        storage_maximum = (1 << (bits - 1)) - 1 if register.signed else (1 << bits) - 1
        minimum = register.minimum if register.minimum is not None else storage_minimum
        maximum = register.maximum if register.maximum is not None else storage_maximum
        if not minimum <= value <= maximum:
            raise ValueError(f"{parameter} must be between {minimum} and {maximum}")

        if not self.simulated:
            if not allow_live:
                raise MotionLockedError("Live Drive parameter writes are not enabled for this operation")
            self._validate_live_parameter_write(register, value)

        self._write_value(register, value)
        readback = self._read_value(register)
        if readback != value:
            raise RuntimeError(
                f"{parameter} verification failed: wrote {value}, read back {readback}"
            )
        if register in DI_PLANNING_REGISTERS:
            self._refresh_di_planning_cache()
        return readback

    def _validate_live_parameter_write(self, register: Register, value: int) -> None:
        """Require a stopped, Servo-Off drive for generic live parameter editing."""
        if register is P1_009:
            if value != 0:
                self._require_motion_permission(f"P1.009 write ({value / 10.0:g} rpm)")
            return
        if register in (P4_006, P4_007):
            raise MotionLockedError(
                f"Use the Motion-page Digital I/O editor for live {register.parameter} changes"
            )
        if self._read_value(P1_009) != 0:
            raise MotionLockedError("Set P1.009 to 0 rpm before writing a live parameter")
        drive_status = self._read_value(P0_046)
        if not drive_status & 0x0004:
            raise MotionLockedError("The live drive must report zero speed before a parameter write")
        if drive_status & 0x0002:
            raise MotionLockedError("Servo Off is required for generic live parameter writes")
        if register is P3_006:
            if value & 0x0040:
                raise MotionLockedError("DI7/S-P must remain hardware-controlled for Speed-only operation")
            self._refresh_di_planning_cache()
            emgs_mask = sum(1 << (channel - 1) for channel in self._emgs_channels())
            if value & emgs_mask:
                channels = ", ".join(f"DI{channel}" for channel in self._emgs_channels())
                raise MotionLockedError(
                    f"{channels} is assigned EMGS and must remain hardware-sourced in Live Drive mode"
                )

    @_serial_io
    def compare_project_profile(self) -> dict[str, bool | None]:
        values = self.read_project_parameters()
        return {
            register.parameter: register.matches_project(values[register.parameter])
            for register in PROJECT_PARAMETER_CATALOG
        }

    @_serial_io
    def command_speed_rpm(self, speed_rpm: float) -> None:
        self._require_connection()
        if not self.MIN_SPEED_RPM <= speed_rpm <= self.MAX_SPEED_RPM:
            raise ValueError(
                f"Speed must be between {self.MIN_SPEED_RPM:g} and {self.MAX_SPEED_RPM:g} rpm"
            )
        # A zero command is always permitted on a connected drive. This keeps
        # the HMI's controlled-stop action available even when live motion has
        # not been enabled (or has just been automatically disarmed).
        if not (not self.simulated and abs(speed_rpm) < 1e-9):
            self._require_motion_permission(f"Speed command ({speed_rpm:g} rpm)")

        # P1.009 supplies the internal speed command selected by SPD0. Live
        # DI7 remains hardware-sourced OFF; clearing its software shadow does
        # not override the physical input. Subsequent speed writes act directly.
        self._write_value(P1_009, round(speed_rpm * 10.0))
        p4_007 = self._read_value(P4_007)
        p4_007 = (p4_007 | 0x0020) & ~0x0040  # SPD0 ON, S-P OFF
        self._write_value(P4_007, p4_007)
        self._speed_mode_selected = True
        self._last_speed_command_word = round(speed_rpm * 10.0)

    def update_speed_command_rpm(self, speed_rpm: float, *, force: bool = False) -> bool:
        """Fast in-motion update: one P1.009 write, with 0.1 rpm de-duplication.

        Speed mode must have been selected once through ``select_speed_mode`` (or
        ``command_speed_rpm``).  This deliberately avoids re-reading and
        re-writing P4.007 on every control tick so encoder telemetry keeps
        priority on the Modbus link.  Returns True when a write was sent.
        """
        # Normal callers deduplicate before taking the bus lock. force=True,
        # used by the CSV timeline, deliberately sends even a repeated setpoint;
        # the caller's pending-command flag supplies priority over telemetry.
        if not self.MIN_SPEED_RPM <= speed_rpm <= self.MAX_SPEED_RPM:
            raise ValueError(
                f"Speed must be between {self.MIN_SPEED_RPM:g} and {self.MAX_SPEED_RPM:g} rpm"
            )
        command_word = round(speed_rpm * 10.0)
        # This fast path intentionally runs before taking the serial lock. A
        # high-rate telemetry reader may occupy the bus continuously when the
        # physical read time exceeds the requested sampling period. Unchanged
        # commands require no I/O and must never block the UI control timer.
        if not force and command_word == self._last_speed_command_word:
            return False
        with self._io_lock:
            self._require_connection()
            if not (not self.simulated and abs(speed_rpm) < 1e-9):
                self._require_motion_permission(f"Speed command ({speed_rpm:g} rpm)")
            if not self._speed_mode_selected:
                raise MotionLockedError("Select Speed mode before using fast speed updates")
            # Another command may have completed while this caller waited.
            if not force and command_word == self._last_speed_command_word:
                return False
            self._write_value(P1_009, command_word)
            self._last_speed_command_word = command_word
            return True

    @_serial_io
    def controlled_stop(self) -> None:
        """Command zero, reassert Speed mode, and verify the drive accepted it."""
        # A verified zero setpoint is only a command acknowledgement. The GUI keeps
        # acquiring feedback to establish stable zero speed before run completion.
        self.command_speed_rpm(0.0)
        command_word = self._read_value(P1_009)
        p4_007 = self._read_value(P4_007)
        if command_word != 0:
            raise RuntimeError(
                f"Controlled stop verification failed: P1.009 read back {command_word / 10.0:+.1f} rpm"
            )
        if not p4_007 & 0x0020 or p4_007 & 0x0040:
            raise RuntimeError(
                "Controlled stop verification failed: Speed mode is not selected after the zero command"
            )

    @_serial_io
    def set_simulation_ramps(
        self,
        acceleration_s: float,
        deceleration_s: float,
        reference_speed_rpm: float = DEFAULT_OPERATOR_LIMIT_RPM,
    ) -> None:
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError("Ramp writes are locked in Live Drive mode")
        acceleration_ms, deceleration_ms = self.ramp_parameter_values(
            acceleration_s, deceleration_s, reference_speed_rpm
        )
        self._write_value(P1_034, acceleration_ms)
        self._write_value(P1_035, deceleration_ms)

    @classmethod
    def ramp_parameter_values(
        cls,
        acceleration_s: float,
        deceleration_s: float,
        reference_speed_rpm: float = DEFAULT_OPERATOR_LIMIT_RPM,
    ) -> tuple[int, int]:
        """Convert operator ramp times to the drive's 0-to-3000-rpm basis."""
        # Operator time is measured at the chosen reference speed; the drive stores
        # time for a 3000 rpm change. Preserve that conversion rather than writing
        # operator seconds directly as register milliseconds.
        if acceleration_s <= 0 or deceleration_s <= 0 or not 0 < reference_speed_rpm <= cls.MAX_SPEED_RPM:
            raise ValueError("Ramp times and reference speed must be positive and within the hard limit")
        # P1.034/P1.035 are defined on a 0-to-3000-rpm basis. Convert the
        # operator's 0-to-reference-speed time into the drive parameter basis.
        acceleration_ms = round(acceleration_s * 3000.0 / reference_speed_rpm * 1000.0)
        deceleration_ms = round(deceleration_s * 3000.0 / reference_speed_rpm * 1000.0)
        if not 1 <= acceleration_ms <= 65500 or not 1 <= deceleration_ms <= 65500:
            raise ValueError("Ramp time is outside the P1.034/P1.035 range for the applied speed range")
        return acceleration_ms, deceleration_ms

    @_serial_io
    def apply_ramps(
        self,
        acceleration_s: float,
        deceleration_s: float,
        reference_speed_rpm: float = DEFAULT_OPERATOR_LIMIT_RPM,
    ) -> tuple[int, int]:
        """Apply and verify P1.034/P1.035 in Simulation or Live Drive mode."""
        self._require_connection()
        acceleration_ms, deceleration_ms = self.ramp_parameter_values(
            acceleration_s, deceleration_s, reference_speed_rpm
        )
        if not self.simulated:
            # Validate both writes before changing either register. Generic
            # live parameter editing requires zero command, zero speed and SON
            # OFF, which is also the safe state for changing motion ramps.
            self._validate_live_parameter_write(P1_034, acceleration_ms)
            self._validate_live_parameter_write(P1_035, deceleration_ms)
        for register, value in (
            (P1_034, acceleration_ms),
            (P1_035, deceleration_ms),
        ):
            self._write_value(register, value)
            readback = self._read_value(register)
            if readback != value:
                raise RuntimeError(
                    f"{register.parameter} verification failed: wrote {value}, read back {readback}"
                )
        return acceleration_ms, deceleration_ms

    @_serial_io
    def set_simulation_feedback_noise(self, amplitude_rpm: float) -> None:
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError("Simulator feedback settings are unavailable in Live Drive mode")
        if amplitude_rpm < 0 or amplitude_rpm > 10:
            raise ValueError("Feedback noise must be between 0 and 10 rpm")
        assert isinstance(self.transport, MockTransport)
        self.transport.feedback_noise_rpm = amplitude_rpm

    @_serial_io
    def configure_di_sources(self, software_mask_di1_to_di13: int) -> int:
        """Apply P3.006 source bits for DI1-DI13."""
        self._require_connection()
        if not 0 <= software_mask_di1_to_di13 <= self.DI_MASK:
            raise ValueError("DI source mask must contain only DI1-DI13 bits")
        if not self.simulated:
            if software_mask_di1_to_di13 & 0x0040:
                raise MotionLockedError("DI7/S-P must remain hardware-controlled for Speed-only operation")
            self._refresh_di_planning_cache()
            emgs_mask = sum(1 << (channel - 1) for channel in self._emgs_channels())
            if software_mask_di1_to_di13 & emgs_mask:
                channels = ", ".join(f"DI{channel}" for channel in self._emgs_channels())
                raise MotionLockedError(
                    f"{channels} is assigned EMGS and must remain hardware-sourced in Live Drive mode"
                )
            if self._read_value(P1_009) != 0:
                raise MotionLockedError("Set P1.009 to 0 rpm before changing live DI sources")
            if not self._read_value(P0_046) & 0x0004:
                raise MotionLockedError("The live drive must report zero speed before changing DI sources")
        current = self._read_value(P3_006)
        updated = (current & ~self.DI_MASK) | software_mask_di1_to_di13
        self._write_value(P3_006, updated)
        return updated

    def _live_motion_readiness_issues(self, max_test_speed_rpm: float) -> tuple[str, ...]:
        issues: list[str] = []
        if self.simulated:
            return ("Live Drive mode is not connected",)
        if not 0 < max_test_speed_rpm <= self.MAX_SPEED_RPM:
            issues.append("Live test speed cap must be greater than 0 and no more than 1000 rpm")
        status = self._read_value(P0_046)
        alarm = self._read_value(P0_001)
        p3_006 = self._read_value(P3_006)
        p4_007 = self._read_value(P4_007)
        self._refresh_di_planning_cache()
        son_channels = self._function_channels_from_planning(
            self._di_planning_cache, self.SON_FUNCTION_CODE
        )
        emgs_channels = self._emgs_channels()
        if not status & 0x0001:
            issues.append("SRDY is OFF")
        if not status & 0x0002:
            issues.append("SON is OFF")
        if not status & 0x0004:
            issues.append("ZSPD is OFF")
        if status & 0x0040 or alarm:
            issues.append("A drive alarm is active")
        if self._read_value(P1_009) != 0:
            issues.append("P1.009 is not 0 rpm")
        if (self._read_value(P1_001) & 0x000F) != 0x0008:
            issues.append("P1.001.X is not the project PR/S control mode (8)")
        if self._read_value(P3_005) != 0:
            issues.append("P3.005 is not low-word-first (0)")
        if not p3_006 & 0x0020:
            issues.append("P3.006 does not select software control for DI6/SPD0")
        if p3_006 & 0x0040:
            issues.append("DI7/S-P must remain hardware-controlled")
        if self._di_planning_cache[6] != 0x0118:
            issues.append("DI7/S-P must be normally open (P2.016 = 0x0118)")
        if len(son_channels) != 1:
            issues.append("Exactly one digital input must be assigned SON")
        elif p3_006 & (1 << (son_channels[0] - 1)):
            issues.append(f"DI{son_channels[0]}/SON must remain hardware-controlled")
        if len(emgs_channels) != 1:
            issues.append("Exactly one digital input must be assigned EMGS")
        elif not self._emgs_normally_closed():
            issues.append(f"DI{emgs_channels[0]}/EMGS is not configured as normally closed")
        elif p3_006 & (1 << (emgs_channels[0] - 1)):
            issues.append(f"DI{emgs_channels[0]}/EMGS is software-controlled")
        elif p4_007 & (1 << (emgs_channels[0] - 1)):
            issues.append(f"DI{emgs_channels[0]}/EMGS is active")
        if self._read_value(P0_017) != MONITOR_COMMAND_CODE:
            issues.append("P0.017 is not configured for integrated-command monitoring")
        if self._read_value(P0_018) != MONITOR_FEEDBACK_CODE:
            issues.append("P0.018 is not configured for encoder-speed monitoring")
        drive_limit = self._read_value(P1_055)
        overspeed_level = self._read_value(P1_111)
        if max_test_speed_rpm > drive_limit:
            issues.append(f"Test cap exceeds P1.055 ({drive_limit} rpm)")
        if max_test_speed_rpm >= overspeed_level:
            issues.append(f"Test cap must remain below P1.111 ({overspeed_level} rpm)")
        if not 1 <= self._read_value(P1_034) <= 65500 or not 1 <= self._read_value(P1_035) <= 65500:
            issues.append("P1.034/P1.035 ramp values are invalid")
        return tuple(issues)

    @_serial_io
    def live_motion_readiness_issues(self, max_test_speed_rpm: float) -> tuple[str, ...]:
        self._require_connection()
        return self._live_motion_readiness_issues(max_test_speed_rpm)

    @_serial_io
    def arm_live_motion(
        self,
        max_test_speed_rpm: float,
        physical_confirmations: tuple[bool, ...],
    ) -> None:
        # Permission is session state, not a saved setting. Recheck live readiness
        # here even if the GUI displayed green checks earlier.
        self._require_connection()
        if self.simulated:
            raise MotionLockedError("Live-test arming is unavailable in Simulation mode")
        if not physical_confirmations or not all(physical_confirmations):
            raise MotionLockedError("Every physical commissioning confirmation is required")
        issues = self._live_motion_readiness_issues(max_test_speed_rpm)
        if issues:
            raise MotionLockedError("; ".join(issues))
        self.hardware_motion_enabled = True

    @_serial_io
    def disarm_live_motion(self, request_stop: bool = True) -> None:
        if self.transport is not None and not self.simulated and request_stop:
            try:
                if self._verified_pr_program is not None:
                    self._write_value(P5_007, 1000)
                self._write_value(P1_009, 0)
            except Exception:
                pass
        self.hardware_motion_enabled = False

    @_serial_io
    def read_io_configuration(self) -> dict[str, object]:
        """Read the non-polling I/O setup used by the Motion-page editor."""
        self._require_connection()
        di_planning = self._refresh_di_planning_cache()
        return {
            "di_planning": di_planning,
            "do_planning": tuple(self._read_value(register) for register in DO_PLANNING_REGISTERS),
            "di_sources": self._read_value(P3_006) & self.DI_MASK,
            "di_software_states": self._read_value(P4_007) & self.DI_MASK,
            "do_software_states": self._read_value(P4_006),
        }

    @_serial_io
    def apply_io_configuration(
        self,
        di_planning: tuple[int, ...],
        do_planning: tuple[int, ...],
        di_software_sources: int,
        di_software_states: int,
        do_software_states: int,
    ) -> dict[str, object]:
        """Apply a staged I/O setup with live-drive zero-speed safeguards."""
        # Source bits, function planning and logical states are separate registers.
        # Do not simplify this to one DI bitmap or software-source the EMGS chain.
        self._require_connection()
        if len(di_planning) != self.DI_CHANNEL_COUNT or len(do_planning) != 6:
            raise ValueError("The I/O plan must contain 13 DI and 6 DO values")
        if di_software_sources & ~self.DI_MASK or di_software_states & ~self.DI_MASK:
            raise ValueError("DI source and state masks may contain only DI1-DI13 bits")
        if do_software_states & ~0xFFFF:
            raise ValueError("P4.006 may contain only 16 software-output bits")
        for value in (*di_planning, *do_planning):
            if value & ~0x01FF:
                raise ValueError("I/O planning values may contain only the contact bit and function byte")

        current_di = tuple(self._read_value(register) for register in DI_PLANNING_REGISTERS)
        current_do = tuple(self._read_value(register) for register in DO_PLANNING_REGISTERS)
        planning_changes = current_di != di_planning or current_do != do_planning
        emgs_channels = self._emgs_channels_from_planning(di_planning)
        if not self.simulated:
            if di_software_sources & 0x0040 or di_planning[6] != 0x0118:
                raise MotionLockedError("Speed-only operation requires DI7/S-P normally open and hardware-controlled")
            if len(emgs_channels) != 1:
                raise MotionLockedError(
                    "Live Drive I/O configuration requires exactly one DI assigned to EMGS"
                )
            emgs_channel = emgs_channels[0]
            if di_planning[emgs_channel - 1] & 0x0100:
                raise MotionLockedError(
                    f"DI{emgs_channel}/EMGS must be configured as normally closed in Live Drive mode"
                )
            if di_software_sources & (1 << (emgs_channel - 1)):
                raise MotionLockedError(
                    f"DI{emgs_channel}/EMGS must use the Hardware source in Live Drive mode"
                )
            if self._read_value(P1_009) != 0:
                raise MotionLockedError("Set P1.009 to 0 rpm before changing live I/O configuration")
            drive_status = self._read_value(P0_046)
            if not drive_status & 0x0004:
                raise MotionLockedError("The live drive must report zero speed before changing I/O configuration")
            if planning_changes and drive_status & 0x0002:
                raise MotionLockedError(
                    "Servo Off is required before changing P2 DI/DO function or contact planning"
                )

        current_sources = self._read_value(P3_006)
        current_emgs_mask = sum(
            1 << (channel - 1) for channel in self._emgs_channels_from_planning(current_di)
        )
        target_emgs_mask = sum(1 << (channel - 1) for channel in emgs_channels)
        staged_sources = current_sources & ~(current_emgs_mask | target_emgs_mask)
        if staged_sources != current_sources:
            self._write_value(P3_006, staged_sources)

        for register, old, new in zip(DI_PLANNING_REGISTERS, current_di, di_planning):
            if old != new:
                self._write_value(register, new)
        for register, old, new in zip(DO_PLANNING_REGISTERS, current_do, do_planning):
            if old != new:
                self._write_value(register, new)

        # Set software values before selecting their software source, avoiding a
        # transient caused by switching a channel to an undefined software bit.
        current_combined_di = self._read_value(P4_007)
        staged_di = (current_combined_di & ~di_software_sources) | (
            di_software_states & di_software_sources
        )
        self._write_value(P4_007, staged_di)
        self._write_value(P4_006, do_software_states)
        self._write_value(P3_006, (staged_sources & ~self.DI_MASK) | di_software_sources)
        self._di_planning_cache = di_planning
        return {
            "di_planning": di_planning,
            "do_planning": do_planning,
            "di_sources": di_software_sources,
            "di_software_states": di_software_states,
            "do_software_states": do_software_states,
        }

    @classmethod
    def _function_channels_from_planning(
        cls, planning: tuple[int, ...], function_code: int
    ) -> tuple[int, ...]:
        return tuple(
            channel
            for channel, value in enumerate(planning, start=1)
            if value & 0x00FF == function_code
        )

    @classmethod
    def _emgs_channels_from_planning(cls, planning: tuple[int, ...]) -> tuple[int, ...]:
        return cls._function_channels_from_planning(planning, cls.EMGS_FUNCTION_CODE)

    def _refresh_di_planning_cache(self) -> tuple[int, ...]:
        self._di_planning_cache = tuple(
            self._read_value(register) for register in DI_PLANNING_REGISTERS
        )
        return self._di_planning_cache

    def _emgs_channels(self) -> tuple[int, ...]:
        return self._emgs_channels_from_planning(self._di_planning_cache)

    def _emgs_normally_closed(self) -> bool:
        channels = self._emgs_channels()
        return len(channels) == 1 and not bool(self._di_planning_cache[channels[0] - 1] & 0x0100)

    @_serial_io
    def select_safe_position_mode(self) -> None:
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError("Live PR mode is disabled; DI7/S-P remains hardware-controlled OFF")
        self._require_motion_permission("Mode selection")
        self._write_value(P1_009, 0)
        p4_007 = self._read_value(P4_007)
        p4_007 = (p4_007 | 0x0040) & ~0x0020  # S-P ON, SPD0 OFF
        self._write_value(P4_007, p4_007)
        self._speed_mode_selected = False
        self._last_speed_command_word = 0

    @_serial_io
    def select_speed_mode(self) -> None:
        """Select internal Speed mode at a forced zero speed command."""
        self._require_connection()
        self._require_motion_permission("Mode selection")
        self._write_value(P1_009, 0)
        p4_007 = self._read_value(P4_007)
        p4_007 = (p4_007 | 0x0020) & ~0x0040  # SPD0 ON, S-P OFF
        self._write_value(P4_007, p4_007)
        self._speed_mode_selected = True
        self._last_speed_command_word = 0

    @_serial_io
    def reset_alarm(self) -> None:
        self._require_connection()
        if not self.simulated:
            raise MotionLockedError(
                "Live alarm reset is locked; use the wired DI5/ARST during commissioning"
            )
        assert isinstance(self.transport, MockTransport)
        self.transport.write_register(P0_001.address, 0)

    def command_position_degrees(self, position_degrees: float) -> None:
        raise MotionLockedError(
            f"Position command ({position_degrees:g} deg) is reserved for the next commissioning stage"
        )

    def _require_motion_permission(self, command: str) -> None:
        if self.simulated:
            return
        if not self.hardware_motion_enabled:
            raise MotionLockedError(
                f"{command} is locked in Live Drive mode until motor/encoder, STO, EMGS, "
                "direction and stop behaviour are commissioned"
            )

    def _require_connection(self) -> None:
        if self.state is not ConnectionState.CONNECTED or self.transport is None:
            raise ConnectionError("Drive is not connected")

    def _read_value(self, register: Register) -> int:
        self._require_transport()
        assert self.transport is not None
        words = self.transport.read_registers(register.address, register.words)
        return words_to_value(words, signed=register.signed)

    def _write_value(self, register: Register, value: int) -> None:
        self._require_transport()
        assert self.transport is not None
        self.transport.write_registers(register.address, value_to_words(value, register.words))

    def _require_transport(self) -> None:
        if self.transport is None:
            raise ConnectionError("Drive transport is not available")
