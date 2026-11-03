"""Regression tests: GUI orchestration and regressions for deadlines,
cancellation, stop priority and presentation. Some cases construct Tk and skip
when Tcl/Tk is unavailable; other cases use lightweight application doubles.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import time
import threading
import tkinter as tk
import unittest
import queue
from types import SimpleNamespace
from unittest.mock import Mock, patch

from servo_control.config import SerialConfig
from servo_control.controller import DriveController, MotionLockedError
from servo_control.models import ConnectionState
from servo_control.registers import P0_046, P1_034, P1_035, P1_055
from servo_control.transport import MockTransport
from servo_control.ui import ServoControllerApp


class TestPageIntegrationTests(unittest.TestCase):
    def test_pending_live_heartbeat_does_not_read_during_test(self) -> None:
        for preparing, running in ((True, False), (False, True)):
            with self.subTest(preparing=preparing, running=running):
                app = SimpleNamespace(
                    polling=True, poll_inflight=False, root=Mock(),
                    controller=SimpleNamespace(simulated=False, read_snapshot=Mock()),
                    test_preparing=False, test_running=False, snapshot_poll_suspended=False,
                )
                ServoControllerApp._schedule_poll(app, 2000)
                app.test_preparing, app.test_running = preparing, running
                with patch('servo_control.ui.threading.Thread') as worker:
                    app.root.after.call_args.args[1]()
                worker.assert_not_called()
                app.controller.read_snapshot.assert_not_called()
                self.assertTrue(app.snapshot_poll_suspended)
                self.assertFalse(app.poll_inflight)

    def test_successful_idle_snapshot_schedules_two_second_heartbeat(self) -> None:
        app = SimpleNamespace(
            events=queue.Queue(), MAX_EVENTS_PER_PUMP=100,
            controller=SimpleNamespace(state=ConnectionState.CONNECTED, simulated=False),
            polling=True, poll_inflight=True, test_preparing=False, test_running=False,
            LIVE_STATUS_HEARTBEAT_MS=ServoControllerApp.LIVE_STATUS_HEARTBEAT_MS,
            _show_connection_heartbeat=Mock(), _show_snapshot=Mock(), _schedule_poll=Mock(),
            _refresh_labjack_display=Mock(), root=Mock(), _event_pump_interval_ms=lambda: 20,
            _process_events=Mock(), _log=Mock(),
        )
        snapshot = object()
        app.events.put(('snapshot', snapshot))
        ServoControllerApp._process_events(app)
        app._show_connection_heartbeat.assert_called_once_with(True)
        app._show_snapshot.assert_called_once_with(snapshot)
        app._schedule_poll.assert_called_once_with(2000)
        self.assertFalse(app.snapshot_poll_suspended)
        app._log.assert_not_called()

    def test_connection_heartbeat_recovers_after_failed_read(self) -> None:
        app = SimpleNamespace(
            controller=SimpleNamespace(simulated=False, config=SerialConfig()),
            connection_text=Mock(), header_badge=Mock(), header_badge_label=Mock(),
            connection_detail=Mock(),
        )
        ServoControllerApp._show_connection_heartbeat(app, False, 'timeout')
        app.connection_text.set.assert_called_with('NO RESPONSE')
        self.assertIn('timeout', app.connection_detail.set.call_args.args[0])
        ServoControllerApp._show_connection_heartbeat(app, True)
        app.connection_text.set.assert_called_with('CONNECTED')
        app.header_badge.set.assert_called_with('LIVE DRIVE • CONNECTED')

    def test_successful_home_preserves_live_permission_for_next_command(self) -> None:
        for permission_revoked in (False, True):
            with self.subTest(permission_revoked=permission_revoked):
                controller = DriveController(transport_factory=lambda *_: MockTransport())
                controller.connect(SerialConfig(), simulated=False)
                try:
                    controller.arm_live_motion(100.0, (True, True, True, True))
                    if permission_revoked:
                        controller.disarm_live_motion()
                    app = SimpleNamespace(
                        controller=controller, profile_manual_home=True, test_preparing=True,
                        profile_homing_started_at=time.monotonic(),
                        _session_angles=lambda: (0.0, 0.0),
                        last_snapshot=SimpleNamespace(feedback_speed_rpm=0.0),
                        _nearest_home_error=ServoControllerApp._nearest_home_error,
                        events=queue.Queue(),
                    )
                    with patch('servo_control.ui.threading.Thread') as worker:
                        ServoControllerApp._run_manual_home_tick(app)
                    worker.call_args.kwargs['target']()
                    self.assertEqual(app.events.get_nowait(), ('profile_home_complete', 0.0))
                    self.assertEqual(controller.cached_speed_command_rpm, 0.0)
                    if permission_revoked:
                        # Completion must never restore permission revoked by a fault/operator.
                        with self.assertRaises(MotionLockedError):
                            controller.command_speed_rpm(5.0)
                    else:
                        controller.command_speed_rpm(5.0)
                        self.assertEqual(controller.cached_speed_command_rpm, 5.0)
                finally:
                    controller.disconnect()

    def test_dashboard_position_start_preserves_measurements_and_logs_marker(self) -> None:
        # Exercise the real startup handler without Tk or a motion worker.
        app = SimpleNamespace(
            controller=SimpleNamespace(state=ConnectionState.CONNECTED, simulated=True),
            test_running=False, test_preparing=False, dashboard_position_active=False,
            session_origin_pulses=0,
            telemetry=SimpleNamespace(samples=["earlier motion"], clear=Mock()),
            labjack=SimpleNamespace(samples=["earlier voltage"], clear=Mock()),
            _automatic_preflight_ready=lambda: True,
            _session_angles=lambda: (0.0, 0.0), _platform_command_sign=lambda: 1,
            dashboard_position_target=SimpleNamespace(get=lambda: "90"),
            dashboard_position_units=SimpleNamespace(get=lambda: "degrees"),
            dashboard_position_direction_choice=SimpleNamespace(get=lambda: "Automatic (target sign)"),
            dashboard_position_speed=SimpleNamespace(get=lambda: "5"),
            platform_ratio=SimpleNamespace(get=lambda: "2"), applied_speed_limit_rpm=100.0,
            _resolve_dashboard_position_target=ServoControllerApp._resolve_dashboard_position_target,
            dashboard_position_command_status=Mock(), dashboard_position_go=Mock(), _log=Mock(),
        )
        with patch("servo_control.ui.threading.Thread") as worker:
            ServoControllerApp._start_dashboard_position_command(app)
        app.telemetry.clear.assert_not_called()
        app.labjack.clear.assert_not_called()
        self.assertEqual(app.telemetry.samples, ["earlier motion"])
        self.assertEqual(app.labjack.samples, ["earlier voltage"])
        self.assertTrue(app.dashboard_position_active)
        self.assertEqual(app.dashboard_position_goal_deg, 90.0)
        self.assertEqual(app._log.call_args.args[0], "Dashboard position started")
        self.assertIn("monotonic start", app._log.call_args.args[3])
        worker.return_value.start.assert_called_once()

    class ServoOffMockTransport(MockTransport):
        def _update_status(self) -> None:
            super()._update_status()
            self.values[P0_046.address] &= ~0x0002

    def test_speed_profile_timeline_runs_without_gui_test_running_flag(self) -> None:
        actions: list[str] = []
        app = SimpleNamespace(
            profile_execution_cancel=threading.Event(),
            profile_execution_thread=None,
            profile_run_clock=SimpleNamespace(run_id="timeline-run"),
            profile_running_started_at=time.perf_counter(),
            profile_first_motion_command_recorded=False,
            events=queue.Queue(),
            controller=SimpleNamespace(
                update_speed_command_rpm=lambda value, force=False: actions.append(
                    f"command:{value:g}:{force}"
                )
            ),
            _record_profile_event=lambda event, details="": actions.append(event),
            _issue_verified_speed_profile_stop=lambda source: (
                actions.append(f"stop:{source}") or time.monotonic()
            ),
        )
        profile = SimpleNamespace(points=(
            SimpleNamespace(time_s=0.0, value=50.0),
            SimpleNamespace(time_s=0.02, value=100.0),
            SimpleNamespace(time_s=0.04, value=0.0),
        ))

        ServoControllerApp._start_speed_profile_execution_worker(app, profile)
        app.profile_execution_thread.join(timeout=0.5)

        self.assertFalse(app.profile_execution_thread.is_alive())
        self.assertIn("command:50:True", actions)
        self.assertIn("command:100:True", actions)
        self.assertIn("stop:csv_timeline", actions)

    def test_speed_profile_timeline_uses_run_clock_timebase(self) -> None:
        """A different monotonic epoch must not delay perf-counter T0 commands."""
        actions: list[str] = []
        app = SimpleNamespace(
            profile_execution_cancel=threading.Event(),
            profile_execution_thread=None,
            profile_run_clock=SimpleNamespace(run_id="clock-domain-run"),
            profile_running_started_at=time.perf_counter(),
            profile_first_motion_command_recorded=False,
            events=queue.Queue(),
            controller=SimpleNamespace(
                update_speed_command_rpm=lambda value, force=False: actions.append(
                    f"command:{value:g}:{force}"
                )
            ),
            _record_profile_event=lambda event, details="": actions.append(event),
            _issue_verified_speed_profile_stop=lambda source: (
                actions.append(f"stop:{source}") or time.monotonic()
            ),
        )
        profile = SimpleNamespace(points=(
            SimpleNamespace(time_s=0.0, value=5.0),
            SimpleNamespace(time_s=0.03, value=0.0),
        ))

        # This reproduces the Windows machine observed in the physical test,
        # where monotonic() and perf_counter() had epochs about 105 s apart.
        with patch("servo_control.ui.monotonic", return_value=time.perf_counter() - 105.0):
            ServoControllerApp._start_speed_profile_execution_worker(app, profile)
            app.profile_execution_thread.join(timeout=0.5)

        if app.profile_execution_thread.is_alive():
            app.profile_execution_cancel.set()
            app.profile_execution_thread.join(timeout=0.5)
        self.assertFalse(app.profile_execution_thread.is_alive())
        self.assertIn("command:5:True", actions)
        self.assertIn("stop:csv_timeline", actions)

    def test_final_speed_profile_stop_preempts_telemetry_and_verifies_zero(self) -> None:
        order: list[str] = []
        app = SimpleNamespace(
            profile_stopping_started_at=None,
            profile_zero_speed_since=None,
            profile_last_zero_sample_time=None,
            profile_cursor_s=None,
            PROFILE_ZERO_SPEED_RPM=0.5,
            PROFILE_ZERO_STABLE_S=0.25,
            PROFILE_STOP_TIMEOUT_S=15.0,
            motion_polling=True,
            motion_poll_thread=None,
            profile_safety_stop_lock=threading.Lock(),
            profile_safety_stop_complete=threading.Event(),
            profile_safety_stop_monotonic_s=None,
            controller=SimpleNamespace(
                state=ConnectionState.CONNECTED,
                controlled_stop=lambda: order.append("controlled_stop"),
            ),
            telemetry=SimpleNamespace(samples=[]),
            written_profile=SimpleNamespace(duration_s=30.0),
            test_status=SimpleNamespace(set=lambda _value: None),
            profile_validation_text=SimpleNamespace(set=lambda _value: None),
            root=SimpleNamespace(after=lambda *_args: None),
            _position_control_interval_ms=lambda: 5,
            _run_motion_profile_tick=lambda: None,
            _stop_motion_polling=lambda *, wait=False: None,
            _start_motion_polling=lambda: order.append("resume"),
            _record_profile_event=lambda event, details="": order.append(event),
            _abort_motion_profile=lambda reason: self.fail(reason),
            _log=lambda *_args: None,
        )
        app._issue_verified_speed_profile_stop = (
            lambda source: ServoControllerApp._issue_verified_speed_profile_stop(app, source)
        )
        def stop_polling(*, wait: bool = False) -> None:
            order.append(f"pause:{wait}")
            app.motion_polling = False
        app._stop_motion_polling = stop_polling

        with patch("servo_control.ui.monotonic", return_value=100.0):
            ServoControllerApp._run_speed_profile_stop_tick(app)

        self.assertEqual(
            order,
            [
                "pause:False",
                "motor_stop_write_started",
                "controlled_stop",
                "motor_stop_write_completed",
                "resume",
            ],
        )
        self.assertEqual(app.profile_stopping_started_at, 100.0)

    def test_home_error_uses_shortest_physical_rotation(self) -> None:
        self.assertAlmostEqual(ServoControllerApp._nearest_home_error(20.0), 20.0)
        self.assertAlmostEqual(ServoControllerApp._nearest_home_error(350.0), -10.0)
        self.assertAlmostEqual(ServoControllerApp._nearest_home_error(360.0), 0.0)
        self.assertAlmostEqual(ServoControllerApp._nearest_home_error(720.0), 0.0)
        self.assertAlmostEqual(ServoControllerApp._nearest_home_error(-200.0), 160.0)

    def test_dashboard_revolutions_are_relative_to_current_position(self) -> None:
        resolve = ServoControllerApp._resolve_dashboard_position_target
        self.assertEqual(resolve(30.0, 1.0, "revolutions", "Positive / anticlockwise"), 390.0)
        self.assertEqual(resolve(30.0, 1.5, "revolutions", "Negative / clockwise"), -510.0)
        self.assertEqual(resolve(30.0, -2.0, "revolutions", "Automatic (target sign)"), -690.0)

    def test_dashboard_degree_target_keeps_orientation_and_selects_route(self) -> None:
        resolve = ServoControllerApp._resolve_dashboard_position_target
        self.assertEqual(resolve(0.0, 90.0, "degrees", "Positive / anticlockwise"), 90.0)
        self.assertEqual(resolve(0.0, 90.0, "degrees", "Negative / clockwise"), -270.0)
        self.assertEqual(resolve(350.0, 90.0, "degrees", "Positive / anticlockwise"), 450.0)
        self.assertEqual(resolve(350.0, 90.0, "degrees", "Negative / clockwise"), 90.0)

    def test_parameter_editor_writes_and_verifies_simulator_value(self) -> None:
        controller = DriveController()
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=True)
            app.selected_parameter = P1_055
            app.parameter_edit_value.set("125")
            app._refresh_parameter_write_state()
            self.assertEqual(str(app.parameter_write_button.cget("state")), "normal")
            app._write_selected_parameter()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and app.parameter_values.get(P1_055.parameter) != 125:
                app.root.update()
                time.sleep(0.02)
            self.assertEqual(app.parameter_values.get(P1_055.parameter), 125)
            self.assertEqual(controller.read_project_parameters()[P1_055.parameter], 125)
        finally:
            controller.disconnect()
            app.root.destroy()

    def test_simulation_sequence_uses_encoder_feedback_and_passes(self) -> None:
        controller = DriveController()
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=True)
            app.polling = True
            app.applied_speed_limit_rpm = 100.0
            app.applied_min_speed_rpm = -100.0
            app.applied_max_speed_rpm = 100.0
            app.acceleration_s.set("0.1")
            app.deceleration_s.set("0.1")
            app.test_type.set("Step response")
            app.test_start.set("0")
            app.test_end.set("5")
            app.test_step.set("1")
            app.test_hold.set("0.5")
            app.test_cycles.set("1")
            app.test_tolerance.set("2.0")
            app._show_snapshot(controller.read_snapshot())
            self.assertEqual(str(app.test_start_button.cget("state")), "normal")

            app._start_test()
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not app.test_status.get().startswith("PASS"):
                app._show_snapshot(controller.read_snapshot())
                app.root.update()
                time.sleep(0.03)

            self.assertTrue(app.test_status.get().startswith("PASS"), app.test_status.get())
            self.assertEqual(app.test_completed.get(), "2 / 2")
            self.assertTrue(app.test_step_result.get().startswith("PASS"))
        finally:
            app.polling = False
            controller.disconnect()
            app.root.destroy()

    def test_trends_tabs_show_simulated_speed_and_position(self) -> None:
        controller = DriveController()
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=True)
            app.channel_motor_position.set(True)
            app.channel_platform_position.set(True)
            app._update_position_channels()
            tab_labels = [app.trends_notebook.tab(tab_id, "text") for tab_id in app.trends_notebook.tabs()]
            self.assertEqual(tab_labels, ["SPEED TRACKING", "ANGULAR POSITION"])
            controller.command_speed_rpm(60.0)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                app._show_snapshot(controller.read_snapshot())
                app.root.update()
                if app.telemetry.samples[-1].motor_position_deg not in (None, 0.0):
                    break
                time.sleep(0.03)
            latest = app.telemetry.samples[-1]
            self.assertIsNotNone(latest.motor_position_deg)
            self.assertNotEqual(latest.motor_position_deg, 0.0)
            self.assertIsNotNone(latest.platform_position_deg)
            self.assertNotEqual(app.motor_position_text.get(), "--")
            self.assertNotEqual(app.platform_position_text.get(), "--")
            self.assertNotEqual(app.motor_sine_text.get(), "--")
            self.assertNotEqual(app.platform_sine_text.get(), "--")
            self.assertLessEqual(abs(float(app.motor_sine_text.get())), 1.0)
            self.assertLessEqual(abs(float(app.platform_sine_text.get())), 1.0)
            self.assertTrue(app.trends_chart.samples)
            self.assertTrue(app.position_chart.samples)
            chart_text = [
                app.position_chart.itemcget(item, "text")
                for item in app.position_chart.find_all()
                if app.position_chart.type(item) == "text"
            ]
            self.assertIn("Motor sin(θ)", chart_text)
            self.assertIn("Platform sin(θ) 2:1", chart_text)

            app.channel_motor_position.set(False)
            app._update_position_channels()
            self.assertFalse(app.position_chart.show_motor)
            self.assertTrue(app.position_chart.show_platform)
            app._toggle_trend_pause("speed")
            self.assertTrue(app.speed_trends_paused)
            self.assertFalse(app.position_trends_paused)
            app._clear_telemetry()
            self.assertFalse(app.telemetry.samples)
        finally:
            controller.disconnect()
            app.root.destroy()

    def test_high_rate_motion_loop_collects_timestamped_samples(self) -> None:
        controller = DriveController()
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=True)
            app.polling = True
            app._start_motion_polling()
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline and len(app.telemetry.samples) < 8:
                app.root.update()
                time.sleep(0.01)
            self.assertGreaterEqual(len(app.telemetry.samples), 8)
            self.assertTrue(all(sample.sampled_monotonic_s > 0 for sample in app.telemetry.samples))
            self.assertTrue(all(sample.acquisition_duration_s is not None for sample in app.telemetry.samples))
            self.assertIn("Hz", app.motion_quality_text.get())
        finally:
            app._stop_motion_polling()
            app.polling = False
            controller.disconnect()
            app.root.destroy()

    def test_parameter_editor_confirms_and_writes_live_drive_value(self) -> None:
        transport = self.ServoOffMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=False)
            app.selected_parameter = P1_055
            app.parameter_values[P1_055.parameter] = 100
            app.parameter_edit_value.set("125")
            app._refresh_parameter_write_state()
            self.assertEqual(str(app.parameter_write_button.cget("state")), "normal")
            self.assertEqual(str(app.parameter_write_button.cget("text")), "WRITE TO LIVE DRIVE")
            with patch("servo_control.ui.messagebox.askyesno", return_value=True):
                app._write_selected_parameter()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and app.parameter_values.get(P1_055.parameter) != 125:
                app.root.update()
                time.sleep(0.02)
            self.assertEqual(app.parameter_values.get(P1_055.parameter), 125)
            self.assertEqual(controller.read_project_parameters()[P1_055.parameter], 125)
        finally:
            controller.disconnect()
            app.root.destroy()

    def test_motion_page_confirms_and_applies_live_drive_ramps(self) -> None:
        transport = self.ServoOffMockTransport()
        controller = DriveController(transport_factory=lambda _config, _simulated: transport)
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=False)
            app._set_connection_controls(True)
            app.applied_speed_limit_rpm = 100.0
            app.acceleration_s.set("2.0")
            app.deceleration_s.set("1.0")
            self.assertEqual(str(app.ramp_apply_button.cget("text")), "APPLY LIVE DRIVE RAMPS")
            with patch("servo_control.ui.messagebox.askyesno", return_value=True):
                app._apply_ramps()
            deadline = time.monotonic() + 2.0
            values = controller.read_project_parameters()
            while time.monotonic() < deadline and values[P1_035.parameter] != 30000:
                app.root.update()
                time.sleep(0.02)
                values = controller.read_project_parameters()
            self.assertEqual(values[P1_034.parameter], 60000)
            self.assertEqual(values[P1_035.parameter], 30000)
        finally:
            controller.disconnect()
            app.root.destroy()

    def test_motion_page_applies_simulator_ramps_without_confirmation(self) -> None:
        controller = DriveController()
        try:
            app = ServoControllerApp(controller=controller)
        except tk.TclError as exc:
            self.skipTest(f"Tk is unavailable: {exc}")
        app.root.withdraw()
        try:
            controller.connect(SerialConfig(), simulated=True)
            app._set_connection_controls(True)
            app.applied_speed_limit_rpm = 100.0
            app.acceleration_s.set("2.0")
            app.deceleration_s.set("1.0")
            self.assertEqual(str(app.ramp_apply_button.cget("text")), "UPDATE SIMULATOR RAMPS")
            with patch("servo_control.ui.messagebox.askyesno") as confirm:
                app._apply_ramps()
            deadline = time.monotonic() + 2.0
            values = controller.read_project_parameters()
            while time.monotonic() < deadline and values[P1_035.parameter] != 30000:
                app.root.update()
                time.sleep(0.02)
                values = controller.read_project_parameters()
            confirm.assert_not_called()
            self.assertEqual(values[P1_034.parameter], 60000)
            self.assertEqual(values[P1_035.parameter], 30000)
        finally:
            controller.disconnect()
            app.root.destroy()


if __name__ == "__main__":
    unittest.main()
