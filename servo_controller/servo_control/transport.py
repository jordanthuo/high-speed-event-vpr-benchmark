"""Low-level register transport, word encoding and in-memory drive simulator.

RegisterTransport keeps the controller independent of serial hardware.
MinimalModbusTransport serializes individual bus transactions; the controller
additionally locks complete multi-transaction operations. Words are ordered
low first for this project's drive configuration. Do not change that ordering
without checking P3.005 and the signed conversion tests.
MockTransport models ramps, feedback lag, counters and status for development;
its elapsed-time model is not a validated mechanical plant model.
"""

from abc import ABC, abstractmethod
from math import exp, sin
from threading import Lock
from time import monotonic

from .config import SerialConfig
from .registers import (
    PROJECT_PARAMETER_CATALOG,
    P0_001,
    P0_009,
    P0_010,
    P0_011,
    P0_013,
    P0_017,
    P0_018,
    P0_019,
    P0_021,
    P0_046,
    P1_009,
    P1_034,
    P1_035,
    P1_044,
    P1_045,
    P3_001,
    P3_002,
    P4_007,
    P4_009,
)


def words_to_value(words: list[int], signed: bool = False) -> int:
    # Assemble low-word-first data before applying two's-complement sign.
    # Sign-extending each individual word would corrupt 32-bit encoder values.
    if not words:
        raise ValueError("At least one register word is required")
    value = 0
    for index, word in enumerate(words):
        value |= (word & 0xFFFF) << (16 * index)
    bits = 16 * len(words)
    if signed and value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


def value_to_words(value: int, count: int) -> list[int]:
    # Masking also encodes negative integers as fixed-width two's complement.
    # The returned order must stay paired with words_to_value.
    if count < 1:
        raise ValueError("Word count must be positive")
    mask = (1 << (count * 16)) - 1
    raw = value & mask
    return [(raw >> (16 * index)) & 0xFFFF for index in range(count)]


class RegisterTransport(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def read_registers(self, address: int, count: int = 1) -> list[int]:
        raise NotImplementedError

    @abstractmethod
    def write_registers(self, address: int, values: list[int]) -> None:
        raise NotImplementedError

    def read_register(self, address: int) -> int:
        return self.read_registers(address, 1)[0]

    def write_register(self, address: int, value: int) -> None:
        self.write_registers(address, [value & 0xFFFF])


class MockTransport(RegisterTransport):
    """Small deterministic ASDA-B3 simulator used by the desktop HMI."""

    MAX_RPM = 1000.0
    FEEDBACK_TIME_CONSTANT_S = 0.12

    def __init__(self) -> None:
        self.connected = False
        self.values: dict[int, int] = {}
        for register in PROJECT_PARAMETER_CATALOG:
            if register.project_value is not None:
                for offset, word in enumerate(value_to_words(register.project_value, register.words)):
                    self.values[register.address + offset] = word
        # Preserve the -L default upper nibbles while selecting 38400 bps (X=3).
        self.values[P3_001.address] = 0x0203
        self.values[P0_001.address] = 0
        self.values[P0_046.address] = 0x0007  # SRDY, SON and ZSPD
        self.values[P1_009.address] = 0
        self.values[P1_009.address + 1] = 0
        for offset, word in enumerate(value_to_words(1 << 24, 2)):
            self.values[P1_044.address + offset] = word
        for offset, word in enumerate(value_to_words(100000, 2)):
            self.values[P1_045.address + offset] = word
        # Logical DI status: DI3/SON and DI6/SPD0 are active, S-P is OFF, and the
        # normally-closed DI8/EMGS function is OFF (healthy).
        self.values[P4_007.address] = 0x0024
        # Simulated default -L outputs: DO1/SRDY and DO2/ZSPD are active.
        self.values[P4_009.address] = 0x0003
        self._feedback_rpm = 0.0
        self.feedback_noise_rpm = 0.0
        self._target_rpm = 0.0
        self._integrated_rpm = 0.0
        self._encoder_position_pulses = 0.0
        self._last_update = monotonic()
        self._lock = Lock()

    def connect(self) -> None:
        with self._lock:
            self.connected = True
            self._last_update = monotonic()

    def close(self) -> None:
        with self._lock:
            self._update_motion()
            self._target_rpm = 0.0
            self.connected = False

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        with self._lock:
            self._require_connection()
            self._update_motion()
            if address == P0_009.address and count == 6:
                integrated = round(self._integrated_rpm * 10.0) if self.values[P0_017.address] == 9 else 0
                reported_feedback = self._feedback_rpm + self.feedback_noise_rpm * sin(monotonic() * 17.0)
                feedback = round(reported_feedback * 10.0) if self.values[P0_018.address] == 7 else 0
                position = round(self._encoder_position_pulses) if self.values[P0_019.address] == 3 else 0
                return [
                    *value_to_words(integrated, 2),
                    *value_to_words(feedback, 2),
                    *value_to_words(position, 2),
                ]
            if address == P0_010.address and count == 4:
                reported_feedback = self._feedback_rpm + self.feedback_noise_rpm * sin(monotonic() * 17.0)
                feedback = round(reported_feedback * 10.0) if self.values[P0_018.address] == 7 else 0
                position = round(self._encoder_position_pulses) if self.values[P0_019.address] == 3 else 0
                return [
                    *value_to_words(feedback, 2),
                    *value_to_words(position, 2),
                ]
            if address == P0_009.address:
                value = round(self._integrated_rpm * 10.0) if self.values[P0_017.address] == 9 else 0
                return value_to_words(value, count)
            if address == P0_010.address:
                reported_feedback = self._feedback_rpm + self.feedback_noise_rpm * sin(monotonic() * 17.0)
                value = round(reported_feedback * 10.0) if self.values[P0_018.address] == 7 else 0
                return value_to_words(value, count)
            if address == P0_011.address:
                value = round(self._encoder_position_pulses) if self.values[P0_019.address] == 3 else 0
                return value_to_words(value, count)
            if address == P0_013.address:
                torque = round((self._integrated_rpm - self._feedback_rpm) * 5.0)
                value = torque if self.values[P0_021.address] == 54 else 0
                return value_to_words(value, count)
            return [self.values.get(address + offset, 0) for offset in range(count)]

    def write_registers(self, address: int, values: list[int]) -> None:
        with self._lock:
            self._require_connection()
            self._update_motion()
            for offset, value in enumerate(values):
                self.values[address + offset] = value & 0xFFFF
            if address <= P1_009.address < address + len(values):
                command = words_to_value(
                    [self.values[P1_009.address], self.values[P1_009.address + 1]],
                    signed=True,
                )
                self._target_rpm = max(-self.MAX_RPM, min(self.MAX_RPM, command / 10.0))
            self._update_status()

    def _require_connection(self) -> None:
        if not self.connected:
            raise ConnectionError("Simulated drive is disconnected")

    def _update_motion(self) -> None:
        # This lazy model advances when registers are accessed. Its deterministic
        # lag/noise makes regression tests repeatable, not mechanically predictive.
        now = monotonic()
        elapsed = max(0.0, now - self._last_update)
        self._last_update = now
        previous_integrated = self._integrated_rpm
        previous_feedback = self._feedback_rpm
        ramp_parameter = P1_034 if abs(self._target_rpm) > abs(self._integrated_rpm) else P1_035
        ramp_ms = max(1, self.values.get(ramp_parameter.address, 60000))
        ramp_rate = 3000.0 / (ramp_ms / 1000.0)
        command_step = ramp_rate * elapsed
        command_difference = self._target_rpm - self._integrated_rpm
        if abs(command_difference) <= command_step:
            self._integrated_rpm = self._target_rpm
            ramp_duration = abs(command_difference) / ramp_rate if ramp_rate else 0.0
        elif command_difference > 0:
            self._integrated_rpm += command_step
            ramp_duration = elapsed
        else:
            self._integrated_rpm -= command_step
            ramp_duration = elapsed

        # Exact first-order response for a command that changes linearly during
        # the ramp, followed by an optional constant-command settling period.
        if ramp_duration > 0:
            command_rate = (self._integrated_rpm - previous_integrated) / ramp_duration
            decay = exp(-ramp_duration / self.FEEDBACK_TIME_CONSTANT_S)
            self._feedback_rpm = (
                self._integrated_rpm
                - command_rate * self.FEEDBACK_TIME_CONSTANT_S
                + (
                    self._feedback_rpm
                    - previous_integrated
                    + command_rate * self.FEEDBACK_TIME_CONSTANT_S
                )
                * decay
            )
        settling_duration = max(0.0, elapsed - ramp_duration)
        if settling_duration > 0:
            decay = exp(-settling_duration / self.FEEDBACK_TIME_CONSTANT_S)
            self._feedback_rpm = self._integrated_rpm + (
                self._feedback_rpm - self._integrated_rpm
            ) * decay
        if abs(self._integrated_rpm - self._feedback_rpm) < 0.005:
            self._feedback_rpm = self._integrated_rpm
        average_feedback_rpm = (previous_feedback + self._feedback_rpm) / 2.0
        self._encoder_position_pulses += average_feedback_rpm / 60.0 * (1 << 24) * elapsed
        self._update_status()

    def _update_status(self) -> None:
        status = 0x0003  # SRDY and SON
        if abs(self._feedback_rpm) < 0.05:
            status |= 0x0004
        if abs(self._feedback_rpm - self._integrated_rpm) < 0.5:
            status |= 0x0008
        if self.values[P0_001.address]:
            status |= 0x0040
        self.values[P0_046.address] = status
        # Model the common PR/S default outputs used for operator feedback.
        do_status = 0x0001  # DO1 / SRDY
        if status & 0x0004:
            do_status |= 0x0002  # DO2 / ZSPD
        if status & 0x0008:
            do_status |= 0x0004  # DO3 / TSPD
        if not (status & 0x0040):
            do_status |= 0x0010  # DO5 / ALRM (healthy, normally active)
        self.values[P4_009.address] = do_status


class MinimalModbusTransport(RegisterTransport):
    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        self.instrument = None
        self._lock = Lock()

    def connect(self) -> None:
        # Import the serial dependency only for live operation so the simulator
        # can remain usable without a hardware Python environment.
        self.config.validate()
        try:
            import minimalmodbus
            import serial
        except ImportError as exc:
            raise RuntimeError("Live Drive mode requires minimalmodbus and pyserial") from exc

        parity_map = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
        }
        instrument = minimalmodbus.Instrument(self.config.port, self.config.slave_address)
        instrument.mode = minimalmodbus.MODE_RTU
        instrument.serial.baudrate = self.config.baudrate
        instrument.serial.bytesize = self.config.bytesize
        instrument.serial.parity = parity_map[self.config.parity]
        instrument.serial.stopbits = self.config.stopbits
        instrument.serial.timeout = self.config.timeout_s
        instrument.clear_buffers_before_each_transaction = True
        self.instrument = instrument

        # A successful request to the already-proven P3.002 address proves the link.
        self.read_register(P3_002.address)

    def close(self) -> None:
        if self.instrument is not None:
            self.instrument.serial.close()
            self.instrument = None

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        if self.instrument is None:
            raise ConnectionError("Drive is disconnected")
        with self._lock:
            if count == 1:
                return [
                    self.instrument.read_register(
                        registeraddress=address,
                        number_of_decimals=0,
                        functioncode=3,
                        signed=False,
                    )
                ]
            return self.instrument.read_registers(
                registeraddress=address,
                number_of_registers=count,
                functioncode=3,
            )

    def write_registers(self, address: int, values: list[int]) -> None:
        if self.instrument is None:
            raise ConnectionError("Drive is disconnected")
        with self._lock:
            if len(values) == 1:
                self.instrument.write_register(
                    registeraddress=address,
                    value=values[0],
                    number_of_decimals=0,
                    functioncode=6,
                    signed=False,
                )
            else:
                self.instrument.write_registers(registeraddress=address, values=values)
