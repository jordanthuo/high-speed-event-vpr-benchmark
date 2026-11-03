"""Regression tests: Controller guards, register writes/readback, ramp conversion
and simulated motion. Mock transports deliberately inject unsafe states;
passing these tests is not physical commissioning.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import time
import threading
import unittest
from dataclasses import replace

from servo_control.config import SerialConfig
from servo_control.controller import DriveController, MotionLockedError
from servo_control.models import ConnectionState
from servo_control.motion_profile import load_motion_profile
from servo_control.pr_program import compile_position_pr_program
from servo_control.registers import P0_001, P0_009, P0_010, P0_046, P1_009, P1_034, P1_035, P1_055, P2_037, P2_038, P2_039, P2_040, P3_001, P3_003, P3_006, P4_007, P4_009
from servo_control.transport import MockTransport, value_to_words, words_to_value


class SerialConfigTests(unittest.TestCase):
    def test_project_defaults_are_valid(self) -> None:
        SerialConfig().validate()

    def test_invalid_slave_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            SerialConfig(slave_address=0).validate()


class RegisterEncodingTests(unittest.TestCase):
    def test_low_word_first_signed_32_bit_round_trip(self) -> None:
        words = value_to_words(-1000, 2)
        self.assertEqual(words, [0xFC18, 0xFFFF])
        self.assertEqual(words_to_value(words, signed=True), -1000)


class DriveControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = DriveController()
        self.controller.connect(SerialConfig(), simulated=True)

    def tearDown(self) -> None:
        self.controller.disconnect()

    def test_simulation_starts_in_safe_pr_state(self) -> None:
        snapshot = self.controller.read_snapshot()
        self.assertTrue(snapshot.simulated)
        self.assertEqual(snapshot.p3_002, 0x0006)
        self.assertEqual(snapshot.p3_006, 0x0020)
        self.assertTrue(snapshot.di6_software_selected)
        self.assertFalse(snapshot.di7_software_selected)
        self.assertFalse(snapshot.di8_software_selected)
        self.assertTrue(snapshot.di6_on)
        self.assertFalse(snapshot.di7_on)
        self.assertFalse(snapshot.di8_on)
        self.assertTrue(snapshot.emgs_hardware_healthy)
        self.assertTrue(snapshot.drive_ready_indication)
        self.assertTrue(snapshot.speed_mode_selected)
        self.assertEqual(snapshot.command_speed_rpm, 0.0)

    def test_drive_ready_requires_both_srdy_and_son(self) -> None:
        snapshot = self.controller.read_snapshot()
        self.assertFalse(replace(snapshot, drive_status=0x0001).drive_ready_indication)
        self.assertFalse(replace(snapshot, drive_status=0x0002).drive_ready_indication)
        self.assertTrue(replace(snapshot, drive_status=0x0003).drive_ready_indication)

    def test_snapshot_exposes_individual_digital_io_states(self) -> None:
        snapshot = self.controller.read_snapshot()
        self.assertEqual(snapshot.digital_inputs_on_count, 2)
        self.assertFalse(snapshot.digital_input_on(1))
        self.assertTrue(snapshot.digital_input_on(3))
        self.assertFalse(snapshot.digital_input_on(7))
        self.assertFalse(snapshot.digital_input_on(8))
        self.assertFalse(snapshot.digital_input_on(9))
        self.assertEqual(snapshot.digital_outputs_on_count, 4)
        self.assertTrue(snapshot.digital_output_on(1))
        self.assertFalse(snapshot.digital_output_on(4))

    def test_simulation_di_source_mask_supports_all_thirteen_channels(self) -> None:
        updated = self.controller.configure_di_sources(0x1FFF)
        self.assertEqual(updated & 0x1FFF, 0x1FFF)
        snapshot = self.controller.read_snapshot()
        self.assertTrue(snapshot.digital_input_software_selected(1))
        self.assertTrue(snapshot.digital_input_software_selected(6))
        self.assertTrue(snapshot.digital_input_software_selected(7))
        self.assertTrue(snapshot.digital_input_software_selected(8))
        self.assertTrue(snapshot.digital_input_software_selected(13))

    def test_io_editor_round_trip_applies_planning_sources_and_software_states(self) -> None:
        configuration = self.controller.read_io_configuration()
        di_planning = list(configuration["di_planning"])
        do_planning = list(configuration["do_planning"])
        di_planning[1] = 0x0023  # DI2 = PL / CCWL, normally closed
        do_planning[5] = 0x0130  # DO6 = SPO_0, normally open
        applied = self.controller.apply_io_configuration(
            tuple(di_planning),
            tuple(do_planning),
            0x0062,
            0x0042,
            0x0001,
        )
        self.assertEqual(applied["di_planning"][1], 0x0023)
        self.assertEqual(applied["do_planning"][5], 0x0130)
        reread = self.controller.read_io_configuration()
        self.assertEqual(reread["di_sources"], 0x0062)
        self.assertTrue(reread["di_software_states"] & 0x0002)
        self.assertEqual(reread["do_software_states"], 0x0001)

    def test_speed_command_selects_internal_speed_and_ramps(self) -> None:
        self.controller.command_speed_rpm(100.0)
        time.sleep(0.06)
        snapshot = self.controller.read_snapshot()
        self.assertEqual(snapshot.command_speed_rpm, 100.0)
        self.assertTrue(snapshot.di6_on)
        self.assertFalse(snapshot.di7_on)
        self.assertTrue(snapshot.speed_mode_selected)
        self.assertEqual(snapshot.monitor_1_selection, 9)
        self.assertEqual(snapshot.monitor_2_selection, 7)
        self.assertGreater(snapshot.integrated_command_rpm or 0.0, snapshot.feedback_speed_rpm or 0.0)
        self.assertGreater(snapshot.feedback_speed_rpm or 0.0, 0.0)
        self.assertLess(snapshot.feedback_speed_rpm or 0.0, 100.0)

    def test_feedback_does_not_snap_to_integrated_command_after_one_time_constant(self) -> None:
        self.controller.command_speed_rpm(100.0)
        time.sleep(0.14)
        snapshot = self.controller.read_snapshot()
        self.assertGreater(snapshot.integrated_command_rpm or 0.0, snapshot.feedback_speed_rpm or 0.0)
        self.assertGreater(snapshot.tracking_error_rpm or 0.0, 1.0)

    def test_operator_ramp_time_converts_to_manual_3000_rpm_basis(self) -> None:
        self.controller.set_simulation_ramps(2.0, 2.0)
        values = self.controller.read_project_parameters()
        self.assertEqual(values[P1_034.parameter], 60000)
        self.assertEqual(values[P1_035.parameter], 60000)

    def test_speed_ramp_rates_are_read_from_actual_drive_parameters(self) -> None:
        self.controller.set_simulation_ramps(2.0, 2.0, 100.0)
        acceleration, deceleration = self.controller.read_speed_ramp_rates()
        self.assertAlmostEqual(acceleration, 50.0)
        self.assertAlmostEqual(deceleration, 50.0)

    def test_motion_telemetry_is_one_contiguous_timestamped_read(self) -> None:
        class TrackingMockTransport(MockTransport):
            def __init__(self) -> None:
                super().__init__()
                self.read_requests: list[tuple[int, int]] = []

            def read_registers(self, address: int, count: int = 1) -> list[int]:
                self.read_requests.append((address, count))
                return super().read_registers(address, count)

        transport = TrackingMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=True)
        try:
            transport.read_requests.clear()
            sample = controller.read_motion_telemetry()
            self.assertEqual(transport.read_requests, [(P0_010.address, 4)])
            self.assertGreater(sample.sampled_monotonic_s, 0.0)
            self.assertGreaterEqual(sample.acquisition_duration_s, 0.0)
            self.assertEqual(sample.integrated_command_rpm, 0.0)
            self.assertEqual(sample.feedback_speed_rpm, 0.0)
        finally:
            controller.disconnect()

    def test_project_profile_masks_unrelated_baud_nibbles(self) -> None:
        values = self.controller.read_project_parameters()
        self.assertEqual(values[P3_001.parameter], 0x0203)
        comparison = self.controller.compare_project_profile()
        self.assertTrue(comparison[P3_001.parameter])

    def test_simulation_parameter_write_is_range_checked_and_verified(self) -> None:
        readback = self.controller.write_simulation_parameter(P1_055.parameter, 123)
        self.assertEqual(readback, 123)
        values = self.controller.read_project_parameters()
        self.assertEqual(values[P1_055.parameter], 123)
        with self.assertRaises(ValueError):
            self.controller.write_simulation_parameter(P3_003.parameter, 2)
        with self.assertRaises(MotionLockedError):
            self.controller.write_simulation_parameter(P4_009.parameter, 0)

    def test_simulation_feedback_noise_setting_is_bounded(self) -> None:
        self.controller.set_simulation_feedback_noise(0.2)
        self.assertEqual(getattr(self.controller.transport, "feedback_noise_rpm"), 0.2)
        with self.assertRaises(ValueError):
            self.controller.set_simulation_feedback_noise(10.1)

    def test_reverse_command_is_signed(self) -> None:
        self.controller.command_speed_rpm(-42.5)
        snapshot = self.controller.read_snapshot()
        self.assertEqual(snapshot.command_speed_rpm, -42.5)

    def test_out_of_range_speed_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.controller.command_speed_rpm(1000.1)

    def test_safe_position_mode_zeros_command(self) -> None:
        self.controller.command_speed_rpm(25.0)
        self.controller.select_safe_position_mode()
        snapshot = self.controller.read_snapshot()
        self.assertEqual(snapshot.command_speed_rpm, 0.0)
        self.assertFalse(snapshot.di6_on)
        self.assertTrue(snapshot.di7_on)

    def test_dashboard_speed_mode_selection_starts_at_zero(self) -> None:
        self.controller.select_speed_mode()
        snapshot = self.controller.read_snapshot()
        self.assertEqual(snapshot.command_speed_rpm, 0.0)
        self.assertTrue(snapshot.di6_on)
        self.assertFalse(snapshot.di7_on)
        self.assertTrue(snapshot.speed_mode_selected)

    def test_fast_speed_updates_deduplicate_and_do_not_touch_mode_bits(self) -> None:
        class TrackingMockTransport(MockTransport):
            def __init__(self) -> None:
                super().__init__()
                self.writes: list[int] = []

            def write_registers(self, address: int, values: list[int]) -> None:
                self.writes.append(address)
                super().write_registers(address, values)

        transport = TrackingMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=True)
        try:
            controller.select_speed_mode()
            transport.writes.clear()
            self.assertTrue(controller.update_speed_command_rpm(12.34))
            self.assertFalse(controller.update_speed_command_rpm(12.31))
            self.assertEqual(transport.writes, [P1_009.address])
            self.assertEqual(controller.cached_speed_command_rpm, 12.3)

            # An unchanged command must return without waiting for a busy
            # telemetry/serial lock; otherwise the UI profile timer can freeze.
            result: list[bool] = []
            with controller._io_lock:
                thread = threading.Thread(
                    target=lambda: result.append(controller.update_speed_command_rpm(12.3))
                )
                thread.start()
                thread.join(timeout=0.1)
                self.assertFalse(thread.is_alive())
            self.assertEqual(result, [False])
        finally:
            controller.disconnect()

    def test_disconnect_changes_state(self) -> None:
        self.controller.disconnect()
        self.assertEqual(self.controller.state, ConnectionState.DISCONNECTED)

    def test_pr_program_write_is_verified_and_backup_restores(self) -> None:
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "position.csv"
            path.write_text(
                "platform_position_deg,motor_speed_rpm,hold_s\n0,0,2\n90,10,0\n",
                encoding="utf-8",
            )
            profile = load_motion_profile(path, ratio=2.0)
            numerator, denominator = self.controller.read_pr_scaling()
            program = compile_position_pr_program(
                profile,
                egear_numerator=numerator,
                egear_denominator=denominator,
                acceleration_ms=600,
                deceleration_ms=600,
                maximum_motor_speed_rpm=100.0,
            )
            writes = self.controller.write_and_verify_pr_program(program)
            self.assertTrue(writes)
            self.assertEqual(
                self.controller.transport.read_registers(0x0606, 2),
                value_to_words(program.paths[0].delta_puu, 2),
            )
            self.controller.restore_pr_program_backup()
            self.assertEqual(self.controller.transport.read_registers(0x0604, 2), [0, 0])
            self.assertEqual(self.controller.transport.read_registers(0x0606, 2), [0, 0])

    def test_pr_program_readback_failure_rolls_back_all_touched_registers(self) -> None:
        import tempfile
        from pathlib import Path

        class OneShotBadReadbackTransport(MockTransport):
            def __init__(self) -> None:
                super().__init__()
                self.injected = False
                self.bad_read_pending = False

            def write_registers(self, address: int, values: list[int]) -> None:
                super().write_registers(address, values)
                if address == 0x0604 and not self.injected:
                    self.injected = True
                    self.bad_read_pending = True

            def read_registers(self, address: int, count: int = 1) -> list[int]:
                values = super().read_registers(address, count)
                if address == 0x0604 and self.bad_read_pending:
                    self.bad_read_pending = False
                    values[0] ^= 1
                return values

        transport = OneShotBadReadbackTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "position.csv"
                path.write_text("platform_position_deg,motor_speed_rpm,hold_s\n0,0,0\n90,6,0\n", encoding="utf-8")
                profile = load_motion_profile(path, ratio=2.0)
                numerator, denominator = controller.read_pr_scaling()
                program = compile_position_pr_program(
                    profile,
                    egear_numerator=numerator,
                    egear_denominator=denominator,
                    acceleration_ms=600,
                    deceleration_ms=600,
                    maximum_motor_speed_rpm=100.0,
                )
                with self.assertRaisesRegex(RuntimeError, "rolled back"):
                    controller.write_and_verify_pr_program(program)
                self.assertEqual(transport.read_registers(0x0528, 1), [0])
                self.assertEqual(transport.read_registers(0x052A, 1), [0])
                self.assertEqual(transport.read_registers(0x0604, 2), [0, 0])
                self.assertEqual(transport.read_registers(0x0606, 2), [0, 0])
        finally:
            controller.disconnect()


class LiveDriveSafetyTests(unittest.TestCase):
    class ServoOffMockTransport(MockTransport):
        def _update_status(self) -> None:
            super()._update_status()
            self.values[P0_046.address] &= ~0x0002

    class SelectableSonMockTransport(MockTransport):
        def __init__(self) -> None:
            self.son_enabled = False
            super().__init__()

        def _update_status(self) -> None:
            super()._update_status()
            if not self.son_enabled:
                self.values[P0_046.address] &= ~0x0002

    def test_live_motion_is_locked_even_with_responding_transport(self) -> None:
        controller = DriveController(transport_factory=lambda _config, _simulated: MockTransport())
        controller.connect(SerialConfig(), simulated=False)
        try:
            snapshot = controller.read_snapshot()
            self.assertFalse(snapshot.simulated)
            with self.assertRaises(MotionLockedError):
                controller.command_speed_rpm(10.0)
            with self.assertRaises(MotionLockedError):
                controller.set_simulation_ramps(2.0, 2.0)
            with self.assertRaises(MotionLockedError):
                controller.write_simulation_parameter(P1_055.parameter, 100)
            with self.assertRaises(MotionLockedError):
                controller.write_parameter(P1_055.parameter, 100)
        finally:
            controller.disconnect()

    def test_live_zero_speed_request_is_available_before_arming(self) -> None:
        transport = MockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        try:
            controller.connect(SerialConfig(), simulated=False)
            controller.command_speed_rpm(0.0)
            self.assertEqual(controller.read_snapshot().requested_speed_rpm, 0.0)
            self.assertFalse(controller.hardware_motion_enabled)
        finally:
            controller.disconnect()

    def test_live_connect_software_sources_spd0_but_hardware_sources_sp_and_preserves_other_bits(self) -> None:
        transport = MockTransport()
        transport.values[P3_006.address] = 0x1080  # Preserve upper bits and DI8.
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        try:
            controller.connect(SerialConfig(), simulated=False)
            snapshot = controller.read_snapshot()
            self.assertEqual(snapshot.p3_006, 0x10A0)
            self.assertTrue(snapshot.di6_software_selected)
            self.assertFalse(snapshot.di7_software_selected)
            self.assertFalse(snapshot.di7_on)
            self.assertTrue(snapshot.di8_software_selected)
        finally:
            controller.disconnect()

    def test_live_connect_zeros_stale_speed_before_switching_sources(self) -> None:
        class TrackingTransport(MockTransport):
            def __init__(self):
                super().__init__()
                self.writes = []

            def write_registers(self, address, values):
                self.writes.append((address, list(values)))
                super().write_registers(address, values)

        transport = TrackingTransport()
        transport.values[P1_009.address] = 1000
        controller = DriveController(transport_factory=lambda *_: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            self.assertEqual(transport.writes[0], (P1_009.address, [0, 0]))
            self.assertEqual(controller.read_snapshot().requested_speed_rpm, 0.0)
            self.assertEqual(controller.read_snapshot().p3_006 & 0x0060, 0x0020)
        finally:
            controller.disconnect()

    def test_active_hardware_sp_does_not_block_connection_arming_or_snapshot(self) -> None:
        class ActiveSpTransport(MockTransport):
            def read_registers(self, address, count=1):
                result = super().read_registers(address, count)
                if address == P4_007.address and not self.values[P3_006.address] & 0x0040:
                    result[0] |= 0x0040  # Physical input remains active despite shadow writes.
                return result

        controller = DriveController(transport_factory=lambda *_: ActiveSpTransport())
        controller.connect(SerialConfig(), simulated=False)
        try:
            self.assertEqual(controller.state, ConnectionState.CONNECTED)
            controller.arm_live_motion(100.0, (True, True, True, True))
            snapshot = controller.read_snapshot()
            self.assertTrue(snapshot.di7_on)
            self.assertFalse(snapshot.di7_software_selected)
            self.assertTrue(snapshot.hardware_motion_enabled)
        finally:
            controller.disconnect()

    def test_live_connect_rejects_normally_closed_unwired_sp(self) -> None:
        transport = MockTransport()
        transport.values[0x0220] = 0x0018  # P2.016: NC S-P would invert an open input.
        controller = DriveController(transport_factory=lambda *_: transport)
        with self.assertRaisesRegex(MotionLockedError, "normally open"):
            controller.connect(SerialConfig(), simulated=False)

    def test_live_source_editors_cannot_software_source_sp(self) -> None:
        transport = self.ServoOffMockTransport()
        controller = DriveController(transport_factory=lambda *_: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            with self.assertRaisesRegex(MotionLockedError, "DI7/S-P"):
                controller.configure_di_sources(0x0060)
            with self.assertRaisesRegex(MotionLockedError, "DI7/S-P"):
                controller.write_parameter(P3_006.parameter, 0x0060)
            config = controller.read_io_configuration()
            with self.assertRaisesRegex(MotionLockedError, "DI7/S-P"):
                controller.apply_io_configuration(
                    tuple(config["di_planning"]), tuple(config["do_planning"]),
                    0x0060, int(config["di_software_states"]), int(config["do_software_states"]),
                )
        finally:
            controller.disconnect()

    def test_live_motion_disarms_if_sp_source_changes(self) -> None:
        transport = MockTransport()
        controller = DriveController(transport_factory=lambda *_: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            controller.arm_live_motion(100.0, (True, True, True, True))
            transport.values[P3_006.address] |= 0x0040
            snapshot = controller.read_snapshot()
            self.assertFalse(snapshot.hardware_motion_enabled)
            self.assertEqual(snapshot.requested_speed_rpm, 0.0)
        finally:
            controller.disconnect()

    def test_live_p2_planning_change_requires_servo_off(self) -> None:
        controller = DriveController(transport_factory=lambda _config, _simulated: MockTransport())
        controller.connect(SerialConfig(), simulated=False)
        try:
            configuration = controller.read_io_configuration()
            di_planning = list(configuration["di_planning"])
            di_planning[1] = 0x0123
            with self.assertRaises(MotionLockedError):
                controller.apply_io_configuration(
                    tuple(di_planning),
                    tuple(configuration["do_planning"]),
                    int(configuration["di_sources"]),
                    int(configuration["di_software_states"]),
                    int(configuration["do_software_states"]),
                )
        finally:
            controller.disconnect()

    def test_live_emgs_hardware_requirement_follows_assigned_channel(self) -> None:
        transport = self.ServoOffMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            with self.assertRaises(MotionLockedError):
                controller.configure_di_sources(0x0080)

            configuration = controller.read_io_configuration()
            di_planning = list(configuration["di_planning"])
            di_planning[7] = 0x0100   # DI8 disabled, normally open
            di_planning[9] = 0x0021   # DI10 EMGS, normally closed
            applied = controller.apply_io_configuration(
                tuple(di_planning),
                tuple(configuration["do_planning"]),
                0x0080,  # DI8 may now be software-sourced
                int(configuration["di_software_states"]),
                int(configuration["do_software_states"]),
            )
            self.assertEqual(applied["di_sources"], 0x0080)
            self.assertEqual(controller.read_snapshot().emgs_channels, (10,))
            self.assertEqual(controller.configure_di_sources(0x0080) & 0x1FFF, 0x0080)
            with self.assertRaises(MotionLockedError):
                controller.configure_di_sources(0x0200)  # DI10 now carries EMGS
        finally:
            controller.disconnect()

    def test_guarded_live_parameter_write_requires_servo_off_and_verifies(self) -> None:
        transport = self.ServoOffMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            self.assertEqual(controller.write_parameter(P1_055.parameter, 123), 123)
            self.assertEqual(controller.read_project_parameters()[P1_055.parameter], 123)
            with self.assertRaises(MotionLockedError):
                controller.write_parameter(P4_007.parameter, 0)
            with self.assertRaises(MotionLockedError):
                controller.write_parameter(P1_009.parameter, 10)
        finally:
            controller.disconnect()

    def test_live_ramp_write_requires_servo_off_and_verifies_both_values(self) -> None:
        transport = self.ServoOffMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            self.assertEqual(controller.apply_ramps(2.0, 1.0, 100.0), (60000, 30000))
            values = controller.read_project_parameters()
            self.assertEqual(values[P1_034.parameter], 60000)
            self.assertEqual(values[P1_035.parameter], 30000)
        finally:
            controller.disconnect()

    def test_live_ramp_write_is_rejected_while_servo_on(self) -> None:
        controller = DriveController(transport_factory=lambda _config, _simulated: MockTransport())
        controller.connect(SerialConfig(), simulated=False)
        try:
            with self.assertRaises(MotionLockedError):
                controller.apply_ramps(2.0, 1.0, 100.0)
        finally:
            controller.disconnect()

    def test_live_motion_requires_all_confirmations_and_disarms_to_zero(self) -> None:
        controller = DriveController(transport_factory=lambda _config, _simulated: MockTransport())
        controller.connect(SerialConfig(), simulated=False)
        try:
            with self.assertRaises(MotionLockedError):
                controller.arm_live_motion(10.0, (True, True, False, True))
            controller.arm_live_motion(10.0, (True, True, True, True))
            self.assertTrue(controller.hardware_motion_enabled)
            controller.command_speed_rpm(10.0)
            self.assertEqual(controller.read_snapshot().requested_speed_rpm, 10.0)
            controller.disarm_live_motion(request_stop=True)
            self.assertFalse(controller.hardware_motion_enabled)
            self.assertEqual(controller.read_snapshot().requested_speed_rpm, 0.0)
            with self.assertRaises(MotionLockedError):
                controller.command_speed_rpm(1.0)
        finally:
            controller.disconnect()

    def test_live_alarm_automatically_removes_motion_permission(self) -> None:
        transport = MockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            controller.arm_live_motion(10.0, (True, True, True, True))
            controller.command_speed_rpm(10.0)
            transport.write_register(P0_001.address, 0x0020)
            snapshot = controller.read_snapshot()
            self.assertTrue(snapshot.alarm_active)
            self.assertEqual(snapshot.requested_speed_rpm, 0.0)
            self.assertFalse(snapshot.hardware_motion_enabled)
            self.assertFalse(controller.hardware_motion_enabled)
        finally:
            controller.disconnect()

    def test_live_emgs_activation_automatically_removes_motion_permission(self) -> None:
        transport = MockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            controller.arm_live_motion(10.0, (True, True, True, True))
            controller.command_speed_rpm(10.0)
            transport.write_register(P4_007.address, transport.values[P4_007.address] | 0x0080)
            snapshot = controller.read_snapshot()
            self.assertFalse(snapshot.emgs_hardware_healthy)
            self.assertEqual(snapshot.requested_speed_rpm, 0.0)
            self.assertFalse(controller.hardware_motion_enabled)
        finally:
            controller.disconnect()

    def test_live_speed_cap_must_fit_drive_protection_limits(self) -> None:
        controller = DriveController(transport_factory=lambda _config, _simulated: MockTransport())
        controller.connect(SerialConfig(), simulated=False)
        try:
            issues = controller.live_motion_readiness_issues(151.0)
            self.assertTrue(any("P1.055" in issue for issue in issues))
            self.assertTrue(any("P1.111" in issue for issue in issues))
            with self.assertRaises(MotionLockedError):
                controller.arm_live_motion(151.0, (True, True, True, True))
        finally:
            controller.disconnect()

    def test_live_pr_write_select_and_start_are_disabled(self) -> None:
        import tempfile
        from pathlib import Path

        transport = self.SelectableSonMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        controller.connect(SerialConfig(), simulated=False)
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "position.csv"
                path.write_text(
                    "platform_position_deg,motor_speed_rpm,hold_s\n0,0,2\n90,10,0\n",
                    encoding="utf-8",
                )
                profile = load_motion_profile(path, ratio=2.0)
                numerator, denominator, direction = controller.read_pr_setup()
                self.assertEqual(direction, 0)
                program = compile_position_pr_program(
                    profile,
                    egear_numerator=numerator,
                    egear_denominator=denominator,
                    acceleration_ms=600,
                    deceleration_ms=600,
                    maximum_motor_speed_rpm=100.0,
                )
                with self.assertRaisesRegex(MotionLockedError, "Live PR programs are disabled"):
                    controller.write_and_verify_pr_program(program)
                with self.assertRaisesRegex(MotionLockedError, "Live PR mode is disabled"):
                    controller.select_safe_position_mode()
                with self.assertRaisesRegex(MotionLockedError, "Live PR starts are disabled"):
                    controller.trigger_verified_pr_program()
                self.assertFalse(controller.read_snapshot().di7_on)
                self.assertFalse(controller.hardware_motion_enabled)
        finally:
            controller.disconnect()


class ManualRegisterDefinitionTests(unittest.TestCase):
    def test_manual_address_formula_and_word_widths(self) -> None:
        self.assertEqual(P0_009.address, 0x0012)
        self.assertEqual(P0_010.address, 0x0014)
        self.assertEqual(P1_009.address, 0x0112)
        self.assertEqual(P1_009.address, (1 << 8) + 9 * 2)
        self.assertEqual(P0_009.words, 2)
        self.assertEqual(P0_010.words, 2)
        self.assertTrue(P1_009.signed)
        self.assertEqual(
            (P2_037.address, P2_038.address, P2_039.address, P2_040.address),
            (0x024A, 0x024C, 0x024E, 0x0250),
        )


if __name__ == "__main__":
    unittest.main()
