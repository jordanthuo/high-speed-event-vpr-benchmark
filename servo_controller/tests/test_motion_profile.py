"""Regression tests: Strict CSV schema selection, numeric validation and motion/PR
compilation. Temporary profiles isolate parser behavior from the GUI and
hardware.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import tempfile
import csv
import io
import unittest
from pathlib import Path

from servo_control.motion_profile import (
    MotionProfile,
    ProfilePoint,
    CompiledSegment,
    interpolate_profile,
    load_motion_profile,
    position_speed_command,
    position_waypoint_can_overlap,
    step_profile_value,
    validate_live_speed_profile,
)
from servo_control.pr_program import compile_position_pr_program, decode_pr_status, pr_path_addresses


class MotionProfileTests(unittest.TestCase):
    def _profile(self, contents: str):
        temporary = tempfile.TemporaryDirectory()
        path = Path(temporary.name) / "profile.csv"
        path.write_text(contents, encoding="utf-8")
        self.addCleanup(temporary.cleanup)
        return load_motion_profile(path, ratio=2.0)

    def _timed_compiler_profile(self, contents: str):
        """Build historical timed plans directly for offline compiler regression.

        These tests do not import legacy CSVs through the application parser.
        The PR compiler remains a separate API with existing structured inputs.
        """
        reader = csv.DictReader(io.StringIO(contents))
        rows = list(reader)
        points = tuple(ProfilePoint(float(row["time_s"]), float(row["platform_position_deg"])) for row in rows)
        segments = tuple(
            CompiledSegment(
                a.time_s, b.time_s, a.value, b.value,
                (b.value-a.value)*2.0/360.0*60.0/(b.time_s-a.time_s),
                (b.value-a.value)/360.0*60.0/(b.time_s-a.time_s),
            )
            for a,b in zip(points, points[1:])
        )
        speeds = tuple(float(row["segment_motor_speed_rpm"]) for row in rows[1:]) if "segment_motor_speed_rpm" in rows[0] else None
        return MotionProfile("position", Path("offline_compiler_fixture.csv"), points, segments, 2.0, speeds)

    def test_speed_profile_is_detected_and_interpolated(self) -> None:
        profile = self._profile("time_s,motor_rpm\n0,0\n2,40\n4,0\n")
        self.assertEqual(profile.kind, "speed")
        self.assertEqual(profile.maximum_motor_speed_rpm, 40.0)
        self.assertEqual(interpolate_profile(profile, 1.0), 20.0)
        self.assertEqual(step_profile_value(profile, 1.999), 0.0)
        self.assertEqual(step_profile_value(profile, 2.0), 40.0)
        self.assertEqual(profile.segments[0].platform_speed_rpm, 10.0)

    def test_live_speed_profile_may_start_nonzero_but_must_finish_zero(self) -> None:
        accepted = self._profile("time_s,motor_rpm\n0,25\n2,25\n3,0\n")
        validate_live_speed_profile(accepted)
        rejected = self._profile("time_s,motor_rpm\n0,25\n2,10\n")
        with self.assertRaisesRegex(ValueError, "end with a 0 rpm"):
            validate_live_speed_profile(rejected)

    def test_position_profile_compiles_motor_speed_from_total_platform_angle(self) -> None:
        profile = self._profile("platform_position_deg,motor_speed_rpm,hold_s\n0,0,0\n180,30,0\n")
        self.assertEqual(profile.kind, "position")
        self.assertEqual(profile.segments[0].motor_speed_rpm, 30.0)
        self.assertEqual(profile.segments[0].platform_speed_rpm, 15.0)

    def test_position_speed_command_slows_near_target(self) -> None:
        far, reached, overshoot = position_speed_command(0, 20, 10, 2, 1)
        near, near_reached, _ = position_speed_command(19, 20, 10, 2, 1)
        self.assertEqual(far, 6.0)
        self.assertEqual(near, 0.3)
        self.assertFalse(reached)
        self.assertFalse(near_reached)
        self.assertEqual(overshoot, 0.0)

    def test_position_speed_command_stops_without_reverse_after_crossing(self) -> None:
        command, reached, overshoot = position_speed_command(21.25, 20, 10, 2, 1)
        self.assertEqual(command, 0.0)
        self.assertTrue(reached)
        self.assertEqual(overshoot, 1.25)

    def test_position_speed_command_supports_negative_travel(self) -> None:
        command, reached, overshoot = position_speed_command(10, -10, 4, 2, -1)
        self.assertEqual(command, -4.0)
        self.assertFalse(reached)
        self.assertEqual(overshoot, 0.0)

    def test_zero_hold_same_direction_waypoint_can_overlap(self) -> None:
        profile = self._profile(
            "platform_position_deg,motor_speed_rpm,hold_s\n"
            "0,0,2\n5,0.5,0\n10,1,0\n15,1.5,2\n"
        )
        self.assertTrue(position_waypoint_can_overlap(profile, 1))
        self.assertTrue(position_waypoint_can_overlap(profile, 2))
        self.assertFalse(position_waypoint_can_overlap(profile, 3))

    def test_hold_or_reversal_disables_speed_mode_overlap(self) -> None:
        profile = self._profile(
            "platform_position_deg,motor_speed_rpm,hold_s\n"
            "0,0,0\n5,1,1\n10,1,0\n0,1,0\n"
        )
        self.assertFalse(position_waypoint_can_overlap(profile, 1))
        self.assertFalse(position_waypoint_can_overlap(profile, 2))

    def test_rejects_ambiguous_headers_and_non_increasing_time(self) -> None:
        with self.assertRaises(ValueError):
            self._profile("time,speed\n0,0\n1,10\n")
        with self.assertRaises(ValueError):
            self._profile("time_s,motor_rpm\n0,0\n0,10\n")

    def test_rejects_legacy_position_headers(self) -> None:
        for contents in (
            "time_s,platform_position_deg\n0,0\n2,90\n",
            "time_s,platform_position_deg,segment_motor_speed_rpm\n0,0,0\n2,90,5\n",
        ):
            with self.subTest(contents=contents), self.assertRaisesRegex(ValueError, "no longer supported"):
                self._profile(contents)

    def test_rejects_extra_or_missing_cells_in_both_schemas(self) -> None:
        for contents in (
            "time_s,motor_rpm\n0,0,unexpected\n2,0\n",
            "time_s,motor_rpm\n0,0\n2\n",
            "platform_position_deg,motor_speed_rpm,hold_s\n0,0,0,unexpected\n90,5,0\n",
            "platform_position_deg,motor_speed_rpm,hold_s\n0,0,0\n90,5\n",
        ):
            with self.subTest(contents=contents), self.assertRaisesRegex(ValueError, "number of fields"):
                self._profile(contents)

    def test_stationary_waypoints_preserve_holds_without_division_by_zero(self) -> None:
        profile = self._profile(
            "platform_position_deg,motor_speed_rpm,hold_s\n"
            "0,0,1\n0,0,2\n90,5,0\n90,0,3\n"
        )
        self.assertEqual(profile.position_path_targets_deg, (0.0, 0.0, 90.0, 90.0))
        self.assertEqual(profile.position_path_holds_s, (1.0, 2.0, 0.0, 3.0))
        self.assertEqual([point.time_s for point in profile.points], [0.0, 1.0, 3.0, 9.0, 12.0])
        self.assertEqual(profile.duration_s, 12.0)
        self.assertEqual(profile.segments[0].motor_speed_rpm, 0.0)

    def test_zero_duration_stationary_waypoint_adds_no_duplicate_preview_time(self) -> None:
        profile = self._profile(
            "platform_position_deg,motor_speed_rpm,hold_s\n"
            "0,0,0\n90,5,0\n90,0,0\n180,5,0\n"
        )
        self.assertEqual(profile.pr_path_count, 4)
        self.assertEqual([point.time_s for point in profile.points], [0.0, 6.0, 12.0])

    def test_stationary_only_profile_requires_positive_duration(self) -> None:
        profile = self._profile("platform_position_deg,motor_speed_rpm,hold_s\n0,0,0\n0,0,1\n")
        self.assertEqual(profile.duration_s, 1.0)
        with self.assertRaisesRegex(ValueError, "positive movement or hold"):
            self._profile("platform_position_deg,motor_speed_rpm,hold_s\n0,0,0\n0,0,0\n")

    def test_nonfinite_ratio_is_rejected(self) -> None:
        for ratio in (float("nan"), float("inf")):
            with self.subTest(ratio=ratio), self.assertRaisesRegex(ValueError, "ratio must be finite"):
                load_motion_profile("unused.csv", ratio=ratio)

    def test_position_profile_compiles_to_incremental_pr_paths(self) -> None:
        profile = self._timed_compiler_profile(
            "time_s,platform_position_deg\n0,0\n2,0\n5,90\n7,180\n"
        )
        program = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=600,
            deceleration_ms=600,
            maximum_motor_speed_rpm=100.0,
        )
        self.assertEqual(program.puu_per_motor_revolution, 100000.0)
        self.assertEqual([path.delta_puu for path in program.paths], [0, 50000, 50000])
        self.assertEqual(program.paths[0].definition & 0xF, 3)
        self.assertEqual(program.paths[-1].definition & 0xF, 2)
        self.assertEqual((program.paths[-1].definition >> 4) & 0xF, 0x8)
        self.assertIn(2000, program.delay_slots_ms)

    def test_equal_speed_waypoints_compile_with_overlap(self) -> None:
        profile = self._timed_compiler_profile(
            "time_s,platform_position_deg\n"
            "0,0\n"
            "2,0\n"
            "5.338708,5\n"
            "8.672041,10\n"
            "12.005374,15\n"
            "15.344082,20\n"
            "17.344082,20\n"
        )
        program = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=64500,
            deceleration_ms=64500,
            maximum_motor_speed_rpm=100.0,
        )
        self.assertEqual(
            [path.overlap_with_next for path in program.paths],
            [False, True, True, True, False, False],
        )
        self.assertEqual(
            [path.target_speed_tenths_rpm for path in program.paths[1:5]],
            [5, 5, 5, 5],
        )
        self.assertEqual(
            [(path.definition >> 4) & 0xF for path in program.paths],
            [0x8, 0xA, 0xA, 0xA, 0x8, 0x8],
        )
        requested = [
            segment.end_s - segment.start_s for segment in profile.segments
        ]
        for path, duration_s in zip(program.paths, requested):
            self.assertAlmostEqual(path.predicted_duration_s, duration_s, delta=0.001)

    def test_overlap_is_not_used_across_a_reversal(self) -> None:
        profile = self._timed_compiler_profile(
            "time_s,platform_position_deg\n0,0\n3.338708,5\n6.677416,0\n"
        )
        program = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=64500,
            deceleration_ms=64500,
            maximum_motor_speed_rpm=100.0,
        )
        self.assertEqual(
            [path.overlap_with_next for path in program.paths], [False, False]
        )
        self.assertEqual(
            [(path.definition >> 4) & 0xF for path in program.paths], [0x8, 0x8]
        )

    def test_explicit_variable_speeds_use_matched_overlap_ramps(self) -> None:
        profile = self._timed_compiler_profile(
            "time_s,platform_position_deg,segment_motor_speed_rpm\n"
            "0,0,0\n"
            "2,0,0\n"
            "5.337786,5,0.5\n"
            "7.004289,10,1.0\n"
            "8.132611,15,1.5\n"
            "10.132611,15,0\n"
        )
        self.assertEqual(profile.segment_target_speeds_rpm, (0.0, 0.5, 1.0, 1.5, 0.0))
        self.assertEqual(profile.maximum_motor_speed_rpm, 1.5)
        program = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=64500,
            deceleration_ms=64500,
            maximum_motor_speed_rpm=100.0,
        )
        self.assertEqual(program.ramp_slots_ms, (64500, 64500, 32250, 43000))
        self.assertEqual(
            [path.target_speed_tenths_rpm for path in program.paths],
            [0, 5, 10, 15, 0],
        )
        self.assertEqual(
            [path.overlap_with_next for path in program.paths],
            [False, True, True, False, False],
        )
        self.assertEqual(
            [(path.acceleration_slot, path.deceleration_slot) for path in program.paths[1:4]],
            [(0, 0), (2, 0), (3, 1)],
        )
        writes = {item.parameter: item.value for item in program.register_writes()}
        self.assertEqual(writes["P5.022"], 32250)
        self.assertEqual(writes["P5.023"], 43000)

    def test_position_speed_hold_format_compiles_overlap_holds_and_reversal(self) -> None:
        profile = self._profile(
            "platform_position_deg,motor_speed_rpm,hold_s\n"
            "0,0,2\n"
            "5,0.5,0\n"
            "10,1.0,1.25\n"
            "0,1.5,2\n"
        )
        self.assertEqual(profile.pr_path_count, 4)
        self.assertEqual(profile.maximum_motor_speed_rpm, 1.5)
        program = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=64500,
            deceleration_ms=64500,
            maximum_motor_speed_rpm=100.0,
        )
        self.assertEqual([path.delta_puu for path in program.paths], [0, 2778, 2778, -5556])
        self.assertEqual(
            [path.target_speed_tenths_rpm for path in program.paths], [0, 5, 10, 15]
        )
        self.assertEqual(
            [path.overlap_with_next for path in program.paths], [False, True, False, False]
        )
        self.assertEqual(program.delay_slots_ms, (2000, 0, 1250))
        self.assertEqual(program.ramp_slots_ms, (64500, 64500, 32250))

    def test_position_speed_hold_format_rejects_invalid_rows(self) -> None:
        with self.assertRaisesRegex(ValueError, "first motor_speed_rpm"):
            self._profile(
                "platform_position_deg,motor_speed_rpm,hold_s\n0,1,0\n5,1,0\n"
            )
        with self.assertRaisesRegex(ValueError, "position changes require"):
            self._profile(
                "platform_position_deg,motor_speed_rpm,hold_s\n0,0,0\n5,0,0\n"
            )

    def test_pr_address_boundary_matches_manual(self) -> None:
        self.assertEqual(pr_path_addresses(1), (0x0604, 0x0606))
        self.assertEqual(pr_path_addresses(49), (0x06C4, 0x06C6))
        self.assertEqual(pr_path_addresses(50), (0x0700, 0x0702))
        self.assertEqual(pr_path_addresses(99), (0x07C4, 0x07C6))

    def test_pr_compile_rejects_infeasible_timing(self) -> None:
        profile = self._timed_compiler_profile("time_s,platform_position_deg\n0,0\n1,180\n")
        with self.assertRaisesRegex(ValueError, "cannot reach"):
            compile_position_pr_program(
                profile,
                egear_numerator=1 << 24,
                egear_denominator=100000,
                acceleration_ms=60000,
                deceleration_ms=60000,
                maximum_motor_speed_rpm=100.0,
            )

    def test_pr_status_decoding_distinguishes_command_and_target_completion(self) -> None:
        self.assertFalse(decode_pr_status(3).command_complete)
        self.assertTrue(decode_pr_status(10003).command_complete)
        self.assertFalse(decode_pr_status(10003).target_reached)
        self.assertTrue(decode_pr_status(20003).target_reached)
        self.assertEqual(decode_pr_status(20003).path_number, 3)

    def test_pr_command_sign_can_reverse_motor_puu_without_changing_timing(self) -> None:
        profile = self._timed_compiler_profile("time_s,platform_position_deg\n0,0\n2,90\n")
        positive = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=600,
            deceleration_ms=600,
            maximum_motor_speed_rpm=100.0,
            command_sign=1,
        )
        negative = compile_position_pr_program(
            profile,
            egear_numerator=1 << 24,
            egear_denominator=100000,
            acceleration_ms=600,
            deceleration_ms=600,
            maximum_motor_speed_rpm=100.0,
            command_sign=-1,
        )
        self.assertEqual(positive.paths[0].delta_puu, -negative.paths[0].delta_puu)
        self.assertEqual(
            positive.paths[0].target_speed_tenths_rpm,
            negative.paths[0].target_speed_tenths_rpm,
        )


if __name__ == "__main__":
    unittest.main()
