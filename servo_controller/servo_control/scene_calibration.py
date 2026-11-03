"""Scene height calibration: 3.620 V = 0 mm; 1.671 V = +125 mm."""

from math import isclose

DEFAULT_ZERO_VOLTAGE_V = 3.62
DEFAULT_MM_PER_VOLT = -125.0 / (3.62 - 1.671)


def displacement_mm(voltage_v, zero_voltage_v, mm_per_volt):
    """Signed slope: decreasing voltage means increasing height by default."""
    return None if mm_per_volt is None else (voltage_v - zero_voltage_v) * mm_per_volt


def load_calibration(settings):
    """Migrate the obsolete project slope; preserve other custom calibrations."""
    zero = settings.get("zero_voltage_v", DEFAULT_ZERO_VOLTAGE_V)
    scale = settings.get("mm_per_volt", settings.get("degrees_per_volt", DEFAULT_MM_PER_VOLT))
    if scale is not None and isclose(float(scale), 70.680628, rel_tol=0, abs_tol=1e-6):
        return DEFAULT_ZERO_VOLTAGE_V, DEFAULT_MM_PER_VOLT
    return zero, scale
