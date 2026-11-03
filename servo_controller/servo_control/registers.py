"""Catalog of the project-relevant ASDA-B3 registers and configuration targets.

Addresses, widths, signedness, scaling and persistence feed both the GUI editor
and controller. Treat changes as hardware-facing behavior, not display labels.
project_value is a desired audit value, not live readback; compare_mask limits
comparison to relevant bits. This is a subset, not a complete drive manual.
"""

from dataclasses import dataclass
from enum import Enum


class Persistence(str, Enum):
    READ_ONLY = "Read-only"
    NON_VOLATILE = "Non-volatile"
    POWER_CYCLE = "Power-cycle to apply"
    VOLATILE = "Resets after power cycle"
    RUNTIME = "Runtime command"


@dataclass(frozen=True)
class Register:
    parameter: str
    address: int
    description: str
    words: int = 1
    signed: bool = False
    unit: str = ""
    value_format: str = "DEC"
    minimum: int | None = None
    maximum: int | None = None
    persistence: Persistence = Persistence.NON_VOLATILE
    project_value: int | None = None
    compare_mask: int | None = None
    note: str = ""

    def matches_project(self, value: int) -> bool | None:
        if self.project_value is None:
            return None
        if self.compare_mask is None:
            return value == self.project_value
        return (value & self.compare_mask) == (self.project_value & self.compare_mask)


# The ASDA-B3 address formula is (group << 8) + (parameter number * 2).
# With project/default P3.005.Z = 0, 32-bit values are low word then high word.

# Monitoring registers. P0.017/P0.018 select the content of P0.009/P0.010.
P0_001 = Register(
    "P0.001", 0x0002, "Current drive alarm code", value_format="HEX", persistence=Persistence.READ_ONLY
)
P0_009 = Register(
    "P0.009", 0x0012, "Status monitor 1 - integrated speed command", words=2,
    signed=True, unit="0.1 rpm", persistence=Persistence.READ_ONLY,
)
P0_010 = Register(
    "P0.010", 0x0014, "Status monitor 2 - filtered encoder speed", words=2,
    signed=True, unit="0.1 rpm", persistence=Persistence.READ_ONLY,
)
P0_011 = Register(
    "P0.011", 0x0016, "Status monitor 3 - motor encoder position", words=2,
    signed=True, unit="pulse", persistence=Persistence.READ_ONLY,
)
P0_013 = Register(
    "P0.013", 0x001A, "Status monitor 5", words=2, signed=True,
    persistence=Persistence.READ_ONLY,
)
P0_017 = Register(
    "P0.017", 0x0022, "Monitor 1 selection", signed=True,
    minimum=-300, maximum=127, project_value=9,
    note="Monitoring code 9: integrated Speed command, 0.1 rpm.",
)
P0_018 = Register(
    "P0.018", 0x0024, "Monitor 2 selection", signed=True,
    minimum=-300, maximum=127, project_value=7,
    note="Monitoring code 7: filtered motor/encoder speed, 0.1 rpm.",
)
P0_019 = Register(
    "P0.019", 0x0026, "Monitor 3 selection", signed=True,
    minimum=-300, maximum=127, project_value=3,
    note="Monitoring code 3: motor feedback position in encoder pulses.",
)
P0_021 = Register(
    "P0.021", 0x002A, "Monitor 5 selection", signed=True,
    minimum=-300, maximum=127, project_value=54,
    note="Monitoring code 54: torque feedback, 0.1%.",
)
P0_046 = Register(
    "P0.046", 0x005C, "Servo drive digital-output status", value_format="HEX",
    persistence=Persistence.READ_ONLY,
)

# Project motion parameters.
P1_001 = Register(
    "P1.001", 0x0102, "Control mode and command input", value_format="HEX",
    minimum=0x0000, maximum=0x112F, persistence=Persistence.POWER_CYCLE,
    project_value=0x0008, note="YX = 08 selects PR/S dual mode.",
)
P1_009 = Register(
    "P1.009", 0x0112, "Internal Speed command 1", words=2, signed=True,
    unit="0.1 rpm", minimum=-75000, maximum=75000,
    persistence=Persistence.RUNTIME, project_value=0,
)
P1_034 = Register(
    "P1.034", 0x0144, "S-curve acceleration constant", unit="ms",
    minimum=1, maximum=65500, project_value=60000,
    note="Time from 0 to 3000 rpm; 60000 ms gives 2 s from 0 to 100 rpm.",
)
P1_035 = Register(
    "P1.035", 0x0146, "S-curve deceleration constant", unit="ms",
    minimum=1, maximum=65500, project_value=60000,
    note="Time from 3000 rpm to 0; 60000 ms gives 2 s from 100 to 0 rpm.",
)
P1_036 = Register(
    "P1.036", 0x0148, "S-curve smoothing constant", unit="ms",
    minimum=0, maximum=65500, project_value=0,
    note="0 keeps the internal-command profile trapezoidal.",
)
P1_040 = Register(
    "P1.040", 0x0150, "Maximum speed for analog Speed command 1", words=2,
    unit="rpm", minimum=0, maximum=50000, project_value=0,
    note="Zero scaling prevents an unwired V_REF from commanding speed.",
)
P1_044 = Register(
    "P1.044", 0x0158, "Electronic gear numerator N1", words=2,
    unit="pulse", minimum=1, maximum=(1 << 29) - 1,
    note="Read for PR degree-to-PUU scaling; never assumed by the CSV compiler.",
)
P1_045 = Register(
    "P1.045", 0x015A, "Electronic gear denominator M", words=2,
    unit="PUU", minimum=1, maximum=(1 << 31) - 1,
    note="Read for PR degree-to-PUU scaling; never assumed by the CSV compiler.",
)
P1_055 = Register(
    "P1.055", 0x016E, "Maximum motor speed limit", unit="rpm",
    minimum=0, maximum=5000, project_value=100,
)
P1_081 = Register(
    "P1.081", 0x01A2, "Maximum speed for analog Speed command 2", words=2,
    unit="rpm", minimum=0, maximum=50000, project_value=0,
)
P1_111 = Register(
    "P1.111", 0x01DE, "Overspeed protection level", words=2,
    unit="rpm", minimum=0, maximum=66000, project_value=150,
)

# Digital I/O planning. The low byte is the function code; bit 8 selects
# contact A (normally open) when set and contact B (normally closed) when clear.
P2_010 = Register("P2.010", 0x0214, "DI1 planning - disabled", value_format="HEX", project_value=0x0100)
P2_011 = Register("P2.011", 0x0216, "DI2 planning - disabled", value_format="HEX", project_value=0x0100)
P2_012 = Register("P2.012", 0x0218, "DI3 planning - SON selector, normally open", value_format="HEX", project_value=0x0101)
P2_013 = Register("P2.013", 0x021A, "DI4 planning - disabled", value_format="HEX", project_value=0x0100)
P2_014 = Register("P2.014", 0x021C, "DI5 planning - ARST, normally open", value_format="HEX", project_value=0x0102)
P2_015 = Register("P2.015", 0x021E, "DI6 planning - SPD0, normally open", value_format="HEX", project_value=0x0114)
P2_016 = Register("P2.016", 0x0220, "DI7 planning - S-P, normally open", value_format="HEX", project_value=0x0118)
P2_017 = Register("P2.017", 0x0222, "DI8 planning - EMGS, normally closed", value_format="HEX", project_value=0x0021)
P2_018 = Register("P2.018", 0x0224, "DO1 planning - SRDY, normally open", value_format="HEX", project_value=0x0101)
P2_019 = Register("P2.019", 0x0226, "DO2 planning - ZSPD, normally open", value_format="HEX", project_value=0x0103)
P2_020 = Register("P2.020", 0x0228, "DO3 planning - TSPD, normally open", value_format="HEX", project_value=0x0104)
P2_021 = Register("P2.021", 0x022A, "DO4 planning - TPOS, normally open", value_format="HEX", project_value=0x0105)
P2_022 = Register("P2.022", 0x022C, "DO5 planning - ALRM, normally closed", value_format="HEX", project_value=0x0007)
P2_036 = Register("P2.036", 0x0248, "DI9 planning - disabled", value_format="HEX", project_value=0x0100)
P2_037 = Register("P2.037", 0x024A, "DI10 planning - disabled", value_format="HEX", project_value=0x0100)
P2_038 = Register("P2.038", 0x024C, "DI11 planning - disabled", value_format="HEX", project_value=0x0100)
P2_039 = Register("P2.039", 0x024E, "DI12 planning - disabled", value_format="HEX", project_value=0x0100)
P2_040 = Register("P2.040", 0x0250, "DI13 planning - disabled", value_format="HEX", project_value=0x0100)
P2_041 = Register("P2.041", 0x0252, "DO6 planning - disabled", value_format="HEX", project_value=0x0100)
P2_030 = Register(
    "P2.030", 0x023C, "Auxiliary function", minimum=-8, maximum=8,
    persistence=Persistence.VOLATILE, project_value=5,
    note="Manual requires value 5 for communication control to avoid EEPROM wear.",
)

# Communication parameters.
P3_000 = Register(
    "P3.000", 0x0300, "Modbus slave address", value_format="HEX", minimum=1,
    maximum=0x7F, persistence=Persistence.POWER_CYCLE, project_value=1,
)
P3_001 = Register(
    "P3.001", 0x0302, "Transmission speed", value_format="HEX",
    persistence=Persistence.POWER_CYCLE, project_value=0x0003, compare_mask=0x000F,
    note="Low nibble X = 3 selects 38400 bps; preserve unrelated nibbles.",
)
P3_002 = Register(
    "P3.002", 0x0304, "Modbus communication protocol", value_format="HEX",
    minimum=0, maximum=8, project_value=6, note="6 = Modbus RTU, 8-N-2.",
)
P3_003 = Register(
    "P3.003", 0x0306, "Communication error handling", value_format="HEX",
    minimum=0, maximum=1, project_value=1,
    note="AL020 and decelerate to stop using P5.003.B.",
)
P3_004 = Register(
    "P3.004", 0x0308, "Communication timeout", unit="s", minimum=0,
    maximum=20, project_value=2,
)
P3_005 = Register(
    "P3.005", 0x030A, "Modbus word order and role", value_format="HEX",
    minimum=0, maximum=0x0112, project_value=0,
    note="Z = 0 selects low-word-first for 32-bit transfers.",
)
P3_006 = Register(
    "P3.006", 0x030C, "Digital-input source selection", value_format="HEX",
    minimum=0, maximum=0x1FFF, persistence=Persistence.VOLATILE,
    project_value=0x0020,
    note="Speed-only live operation: DI6/SPD0 software-sourced; unwired DI7/S-P hardware-sourced OFF.",
)
P3_007 = Register(
    "P3.007", 0x030E, "Modbus response delay", unit="0.5 ms",
    minimum=0, maximum=1000, project_value=1,
)

# Diagnosis/runtime registers.
P4_000 = Register("P4.000", 0x0400, "Latest fault record", words=2, value_format="HEX", persistence=Persistence.READ_ONLY)
P4_006 = Register(
    "P4.006", 0x040C, "Software digital-output bits", value_format="HEX",
    minimum=0, maximum=0xFFFF, persistence=Persistence.VOLATILE,
    note="Bits drive DO functions SPO_0 through SPO_F (function codes 0x30-0x3F).",
)
P4_007 = Register(
    "P4.007", 0x040E, "Combined/software digital-input state", value_format="HEX",
    minimum=0, maximum=0x3FFF, persistence=Persistence.VOLATILE,
    note="Read returns combined DI state; write sets software DI state.",
)
P4_009 = Register(
    "P4.009", 0x0412, "Digital output hardware status", value_format="HEX",
    minimum=0, maximum=0x003F, persistence=Persistence.READ_ONLY,
    note="Read-only actual DO1-DO6 hardware state; one bit per output channel.",
)

# Communication-loss deceleration profile. Included for audit only.
P5_003 = Register(
    "P5.003", 0x0506, "Auto-protection deceleration selectors", words=2,
    value_format="HEX", minimum=0, maximum=0xFFFFFFFF, project_value=0xEEDFEEFF,
    note="B/CTO selects profile D (P5.033) for AL020 communication timeout.",
)
P5_033 = Register(
    "P5.033", 0x0542, "PR acceleration/deceleration time 13", unit="ms",
    minimum=1, maximum=65500, project_value=60000,
)
P5_007 = Register(
    "P5.007", 0x050E, "PR command trigger", minimum=0, maximum=1000,
    persistence=Persistence.RUNTIME,
    note="1-99 triggers a PR procedure; 1000 requests the PR stop command.",
)

MONITOR_COMMAND_CODE = 9
MONITOR_FEEDBACK_CODE = 7
MONITOR_ENCODER_POSITION_CODE = 3
MONITOR_TORQUE_CODE = 54

SNAPSHOT_REGISTERS = (
    P0_001, P0_009, P0_010, P0_011, P0_017, P0_018, P0_019, P0_046,
    P1_009, P3_002, P3_006, P4_007, P4_009,
)

PROJECT_PARAMETER_CATALOG = (
    P0_017, P0_018, P0_019, P0_021,
    P1_001, P1_009, P1_034, P1_035, P1_036, P1_040, P1_044, P1_045,
    P1_055, P1_081, P1_111,
    P2_010, P2_011, P2_012, P2_013, P2_014, P2_015, P2_016, P2_017,
    P2_018, P2_019, P2_020, P2_021, P2_022, P2_030,
    P2_036, P2_037, P2_038, P2_039, P2_040, P2_041,
    P3_000, P3_001, P3_002, P3_003, P3_004, P3_005, P3_006, P3_007,
    P4_006, P4_007, P4_009, P5_003, P5_033,
)

DI_PLANNING_REGISTERS = (
    P2_010, P2_011, P2_012, P2_013, P2_014, P2_015, P2_016, P2_017,
    P2_036, P2_037, P2_038, P2_039, P2_040,
)
DO_PLANNING_REGISTERS = (P2_018, P2_019, P2_020, P2_021, P2_022, P2_041)

REGISTER_BY_PARAMETER = {register.parameter: register for register in PROJECT_PARAMETER_CATALOG}
