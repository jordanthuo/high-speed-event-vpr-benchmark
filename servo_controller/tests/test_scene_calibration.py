import unittest
from servo_control.scene_calibration import (
    DEFAULT_MM_PER_VOLT, DEFAULT_ZERO_VOLTAGE_V, displacement_mm, load_calibration,
)


class SceneCalibrationTests(unittest.TestCase):
    def test_reference_points_and_direction(self):
        convert = lambda v: displacement_mm(v, DEFAULT_ZERO_VOLTAGE_V, DEFAULT_MM_PER_VOLT)
        self.assertAlmostEqual(convert(3.62), 0)
        self.assertAlmostEqual(convert(1.671), 125)
        self.assertAlmostEqual(convert((3.62 + 1.671) / 2), 62.5)
        self.assertLess(convert(3.7), 0)

    def test_old_saved_settings_migrate(self):
        for zero in (0.18, 3.62):
            self.assertEqual(load_calibration({'zero_voltage_v': zero, 'mm_per_volt': 70.680628}),
                             (DEFAULT_ZERO_VOLTAGE_V, DEFAULT_MM_PER_VOLT))
        self.assertEqual(load_calibration({'degrees_per_volt': 70.680628}),
                         (DEFAULT_ZERO_VOLTAGE_V, DEFAULT_MM_PER_VOLT))

    def test_custom_and_voltage_only_settings_preserved(self):
        self.assertEqual(load_calibration({'zero_voltage_v': 1, 'mm_per_volt': -50}), (1, -50))
        self.assertIsNone(displacement_mm(2, 1, None))
        self.assertEqual(load_calibration({'mm_per_volt': None}), (3.62, None))
