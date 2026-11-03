"""Regression tests: Playback delay calculation for reconstructed GIFs. Output
display duration is separate from event accumulation and sensor timestamps.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import unittest

from make_angle_comparison_gif import gif_durations_ms


class AngleComparisonGifTests(unittest.TestCase):
    def test_tenth_speed_at_100_source_fps_uses_100_ms_frames(self):
        durations = gif_durations_ms(30, 100.0, 0.1)
        self.assertEqual(set(durations), {100})
        self.assertEqual(sum(durations), 3000)


if __name__ == "__main__":
    unittest.main()
