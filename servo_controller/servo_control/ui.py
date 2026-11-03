"""Tk operator interface and orchestration of servo/camera experiments.

Maintenance map (search method names rather than relying on line numbers):
  _build_* constructs the seven pages and binds operator actions.
  _import_motion_profile / _write_motion_profile stage a validated run.
  _arm_lattepanda_profile chooses remote T0 and freezes the clock mapping.
  _start_speed_profile_execution_worker sends speed rows independently of Tk.
  _run_speed_controlled_position_tick closes the position loop via Tk ticks.
  _start_motion_polling acquires feedback; _process_events consumes its queue.
  _export_profile_run and _export_profile_event_feed define run file schemas.

Thread ownership matters: Tk widgets are updated by the UI event loop; bus,
clock and coordinator work use workers and queue results back to that loop.
Run IDs, cancellation events and acquisition generations reject stale work.
Keep the independent final-stop deadline and control-priority mechanism when
refactoring; a busy chart or saturated bus must not own the speed timeline.

Three origins coexist: raw/continuous encoder angle, connection-time compass
zero and position-run origin. Likewise, perf_counter is the acquisition/T0
clock while monotonic is used for some local intervals. Never subtract values
from different clock domains just because both are measured in seconds.
See README_DETAILED.md for operator steps and the output-field dictionary.
"""

import csv
import json
import queue
import sys
import threading
import tkinter as tk
import uuid
from collections import deque
from collections.abc import Callable
from dataclasses import replace
from datetime import datetime
from math import radians, sin
from pathlib import Path
from statistics import pstdev
from time import monotonic, perf_counter, perf_counter_ns
from tkinter import filedialog, messagebox, ttk

from .clock_sync import DEFAULT_PORT, ClockSyncSample, TestRunClock, UdpClockSyncClient
from .config import SerialConfig
from .controller import DriveController
from .labjack_input import LabJackAin0
from .scene_calibration import DEFAULT_MM_PER_VOLT, DEFAULT_ZERO_VOLTAGE_V, load_calibration
from .models import ConnectionState, DriveSnapshot, MotionTelemetry, OperatingMode
from .motion_profile import (
    MotionProfile,
    interpolate_profile,
    load_motion_profile,
    position_speed_command,
    position_waypoint_can_overlap,
    step_profile_value,
    validate_live_speed_profile,
)
from .pr_program import PrExecutionStatus, PrProgram, compile_position_pr_program
from .registers import P1_034, P1_035, PROJECT_PARAMETER_CATALOG, Persistence, Register
from .run_coordinator import UdpRunCoordinatorClient
from .settings_store import default_settings_path, migrate_legacy_settings, read_settings, write_settings
from .telemetry import EventHistory, TelemetryBuffer, TelemetrySample, speed_sweep_values
from .widgets import BLUE, GREEN, INK, MUTED, NAVY, ORANGE, RED, SURFACE, MotionCompass, PositionChart, ProfilePreview, SceneDisplacementChart, SpeedGauge, TrendChart


class ServoControllerApp:
    APP_BUILD_ID = "2026-10-09-bus-saturated-telemetry-v6"
    SIM_POLL_MS = 100
    LIVE_POLL_MS = 500
    LIVE_STATUS_HEARTBEAT_MS = 2000
    MOTION_SAMPLE_HZ = 50.0
    # A zero rate means that live test acquisition is bus-saturated: begin the
    # next minimum-size telemetry transaction immediately after the preceding
    # response.  The achieved rate is therefore set by the drive, USB/serial
    # adapter and RTU turnaround time, not by an artificial application timer.
    TEST_SAMPLE_HZ_BY_BAUD = ((38_400, 0.0), (57_600, 0.0), (115_200, 0.0))
    CHART_REFRESH_HZ = 10.0
    UI_TELEMETRY_REFRESH_HZ = 20.0
    MAX_CHART_DISPLAY_SAMPLES = 1_200
    MAX_EVENTS_PER_PUMP = 100
    PROFILE_ZERO_SPEED_RPM = 0.5
    PROFILE_ZERO_STABLE_S = 0.25
    PROFILE_STOP_TIMEOUT_S = 15.0
    SPEED_NUDGE_RPM = 1.0
    NAV_ITEMS = ("Dashboard", "Motion", "Tests", "Trends", "Parameters", "Logs", "Settings")
    DI_FUNCTION_CATALOG = {
        0x00: "Disabled", 0x01: "SON", 0x02: "ARST", 0x03: "GAINUP", 0x04: "CCLR",
        0x05: "ZCLAMP", 0x06: "CMDINV", 0x08: "CTRG", 0x09: "TRQLM", 0x0C: "VPL",
        0x0D: "VPRS", 0x0F: "SPDKVC", 0x10: "SPDLM", 0x11: "POS0", 0x12: "POS1",
        0x13: "POS2", 0x14: "SPD0", 0x15: "SPD1", 0x16: "TCM0", 0x17: "TCM1",
        0x18: "S-P", 0x19: "S-T", 0x1A: "POS3", 0x1B: "POS4", 0x1C: "POS5",
        0x1D: "ABSE", 0x1E: "POS6", 0x1F: "ABSC", 0x20: "T-P", 0x21: "EMGS",
        0x22: "NL / CWL", 0x23: "PL / CCWL", 0x24: "ORGP", 0x27: "SHOM",
        0x2B: "PT-PR", 0x37: "JOGU", 0x38: "JOGD", 0x39: "EV1", 0x3A: "EV2",
        0x3B: "EV3", 0x3C: "EV4", 0x43: "GNUM0", 0x44: "GNUM1", 0x45: "INHP",
        0x46: "STP", 0x47: "PFQS",
    }
    DO_FUNCTION_CATALOG = {
        0x00: "Disabled", 0x01: "SRDY", 0x02: "SON", 0x03: "ZSPD", 0x04: "TSPD",
        0x05: "TPOS", 0x06: "TQL", 0x07: "ALRM", 0x08: "BRKR", 0x09: "HOME",
        0x0D: "ABSW", 0x0E: "IDXD", 0x10: "OLW", 0x11: "WARN", 0x12: "OVF",
        0x13: "SNL / SCWL", 0x14: "SPL / SCCWL", 0x15: "Cmd_OK", 0x16: "CAP_OK",
        0x17: "MC_OK", 0x19: "SP_OK", 0x2C: "Zon1", 0x2D: "Zon2", 0x2E: "Zon3",
        0x2F: "Zon4", **{code: f"SPO_{code - 0x30:X}" for code in range(0x30, 0x40)},
    }
    DI_FUNCTIONS = (
        "Disabled", "Disabled", "SON", "Disabled", "ARST", "SPD0", "S-P",
        "EMGS", "Disabled", "Disabled", "Disabled", "Disabled", "Disabled",
    )
    DO_FUNCTIONS = ("SRDY", "ZSPD", "TSPD", "TPOS", "ALRM", "Disabled")

    def __init__(
        self,
        controller: DriveController | None = None,
        auto_connect_simulation: bool = False,
        initial_page: str = "Dashboard",
    ) -> None:
        self.controller = controller or DriveController()
        self.labjack = LabJackAin0()
        self.clock_sync = UdpClockSyncClient(
            on_sample=lambda sample: self.events.put(("clock_sync_sample", sample)),
            on_error=lambda details: self.events.put(("clock_sync_error", details)),
        )
        self.auto_connect_simulation = auto_connect_simulation
        self.root = tk.Tk()
        self.root.title("Servo Test Rig - Delta ASDA-B3")
        self.root.geometry("1280x820")
        self.root.minsize(1080, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.report_callback_exception = self._report_callback_exception

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.polling = False
        self.poll_inflight = False
        self.snapshot_poll_suspended = False
        self.motion_polling = False
        self.motion_poll_generation = 0
        self.motion_stop_event = threading.Event()
        self.motion_command_pending = threading.Event()
        self.motion_poll_thread: threading.Thread | None = None
        self.motion_sample_times: deque[float] = deque(maxlen=101)
        self.motion_late_samples = 0
        self.motion_target_hz = self.MOTION_SAMPLE_HZ
        self.configured_baudrate = 115_200
        self.sim_motion_sample_hz = 50.0
        self.last_motion_chart_redraw = 0.0
        self.last_profile_preview_redraw = 0.0
        self.last_motion_ui_refresh = 0.0
        self.speed_after_id: str | None = None
        self.test_after_id: str | None = None
        self.sim_poll_ms = self.SIM_POLL_MS
        self.test_running = False
        self.test_preparing = False
        self.live_test_arming = False
        self.test_values: list[float] = []
        self.test_index = 0
        self.test_hold_seconds = 0.0
        self.test_tolerance_rpm = 0.0
        self.test_step_sample_count = 0
        self.speed_trends_paused = False
        self.position_trends_paused = False
        self.last_snapshot: DriveSnapshot | None = None
        self.last_sent_setpoint: float | None = None
        self.pending_setpoint_rpm: float | None = None
        self._suppress_slider = False

        self.telemetry = TelemetryBuffer(platform_ratio=2.0)
        self.history = EventHistory()
        self.parameter_values: dict[str, int] = {}
        self.selected_parameter: Register | None = None
        self.parameter_edit_value = tk.StringVar(value="")
        self.parameter_edit_unit = tk.StringVar(value="raw value")
        self.settings_path = default_settings_path()
        self.settings_load_error: str | None = None

        self.mode = tk.StringVar(value=OperatingMode.SIMULATION.value)
        self.port = tk.StringVar(value="COM6")
        self.slave = tk.StringVar(value="1")
        self.baud = tk.StringVar(value="115200")
        self.parity = tk.StringVar(value="N")
        self.stopbits = tk.StringVar(value="2")
        self.timeout = tk.StringVar(value="1.0")
        self.connection_text = tk.StringVar(value="DISCONNECTED")
        self.header_badge = tk.StringVar(value="SIMULATION • DISCONNECTED")
        self.page_title = tk.StringVar(value="Servo Test Rig")

        self.speed_entry = tk.StringVar(value="0.0")
        self.speed_scale = tk.DoubleVar(value=0.0)
        self.speed_limit_magnitude = tk.StringVar(value="100.0")
        self.applied_speed_limit_rpm = self.controller.DEFAULT_OPERATOR_LIMIT_RPM
        self.applied_min_speed_rpm = -self.applied_speed_limit_rpm
        self.applied_max_speed_rpm = self.applied_speed_limit_rpm
        self.minimum_scale_text = tk.StringVar(value="-100")
        self.midpoint_scale_text = tk.StringVar(value="0")
        self.maximum_scale_text = tk.StringVar(value="+100 rpm")
        self.feedback_text = tk.StringVar(value="--")
        self.integrated_text = tk.StringVar(value="--")
        self.direction_text = tk.StringVar(value="STOPPED")
        self.platform_speed_text = tk.StringVar(value="--")
        self.motor_position_text = tk.StringVar(value="--")
        self.platform_position_text = tk.StringVar(value="--")
        self.dashboard_platform_position_text = tk.StringVar(value="--")
        self.drawstring_position_text = tk.StringVar(value="--")
        self.dashboard_displacement_mm_text = tk.StringVar(value="-- mm")
        self.dashboard_displacement_voltage_text = tk.StringVar(value="-- V")
        self.motor_sine_text = tk.StringVar(value="--")
        self.platform_sine_text = tk.StringVar(value="--")
        self.encoder_position_text = tk.StringVar(value="--")
        self.tracking_error_text = tk.StringVar(value="--")
        self.rms_error_text = tk.StringVar(value="--")
        self.peak_error_text = tk.StringVar(value="--")
        self.motion_quality_text = tk.StringVar(value="Target 50 Hz • waiting for data")

        self.status_drive = tk.StringVar(value="DISCONNECTED")
        self.status_drive_detail = tk.StringVar(value="SRDY -- • SON -- • Source: P0.046")
        self.status_io = tk.StringVar(value="DISCONNECTED")
        self.status_io_detail = tk.StringVar(value="DI ---- (P4.007) • DO ---- (P4.009)")
        self.status_alarm = tk.StringVar(value="UNKNOWN")
        self.status_alarm_detail = tk.StringVar(value="Raw: ---- • Source: P0.001")
        self.control_mode_text = tk.StringVar(value="UNKNOWN")
        self.control_mode_detail = tk.StringVar(value="S-P / DI7 readback unavailable")
        self.status_lamps: dict[str, tk.Label] = {}
        self.io_detail_summary = tk.StringVar(value="No current readback")
        self.io_di_values = [tk.StringVar(value="-- • unavailable") for _ in range(13)]
        self.io_do_values = [tk.StringVar(value="-- • unavailable") for _ in range(6)]
        self.io_di_source_choices = [tk.StringVar(value="Hardware") for _ in range(13)]
        default_di_codes = (
            0x00, 0x00, 0x01, 0x00, 0x02, 0x14, 0x18,
            0x21, 0x00, 0x00, 0x00, 0x00, 0x00,
        )
        default_do_codes = (0x01, 0x03, 0x04, 0x05, 0x07, 0x00)
        self.io_di_function_choices = [tk.StringVar(value=self._function_label(code, self.DI_FUNCTION_CATALOG)) for code in default_di_codes]
        self.io_do_function_choices = [tk.StringVar(value=self._function_label(code, self.DO_FUNCTION_CATALOG)) for code in default_do_codes]
        self.io_di_contact_choices = [tk.StringVar(value="Normally closed" if index == 8 else "Normally open") for index in range(1, 14)]
        self.io_do_contact_choices = [tk.StringVar(value="Normally closed" if index == 5 else "Normally open") for index in range(1, 7)]
        self.io_di_software_states = [tk.BooleanVar(value=index in (6, 7)) for index in range(1, 14)]
        self.io_do_software_states = [tk.BooleanVar(value=False) for _ in range(6)]
        self.io_di_state_button_texts = [tk.StringVar(value="TRUE" if index in (6, 7) else "FALSE") for index in range(1, 14)]
        self.io_do_state_button_texts = [tk.StringVar(value="FALSE") for _ in range(6)]
        self.io_do_source_texts = [tk.StringVar(value="Drive function") for _ in range(6)]
        self.io_di_source_controls: list[ttk.Button] = []
        self.io_di_function_controls: list[ttk.Button] = []
        self.io_di_contact_controls: list[ttk.Button] = []
        self.io_di_state_controls: list[ttk.Button] = []
        self.io_do_function_controls: list[ttk.Button] = []
        self.io_do_contact_controls: list[ttk.Button] = []
        self.io_do_state_controls: list[ttk.Button] = []
        self.io_source_dirty = False
        self.io_edit_mode = False
        self.io_configuration_loaded = False
        self.io_configuration: dict[str, object] | None = None
        self.io_di_state_labels: list[tk.Label] = []
        self.io_do_state_labels: list[tk.Label] = []
        self.io_di_lamps: list[tk.Label] = []
        self.io_do_lamps: list[tk.Label] = []

        self.acceleration_s = tk.StringVar(value="2.0")
        self.deceleration_s = tk.StringVar(value="2.0")
        self.ramp_limits_text = tk.StringVar(value="")
        self.platform_ratio = tk.StringVar(value="2.0")
        self.platform_command_direction = tk.StringVar(value="Motor positive (+PUU)")
        self.sample_rate = tk.StringVar(value="10")
        self.feedback_noise = tk.StringVar(value="0.2")
        self.labjack_rate = tk.StringVar(value="100")
        self.labjack_zero_voltage = tk.StringVar(value=str(DEFAULT_ZERO_VOLTAGE_V))
        self.labjack_mm_per_volt = tk.StringVar(value=str(DEFAULT_MM_PER_VOLT))
        self.labjack_status = tk.StringVar(value="DISCONNECTED")
        self.clock_sync_host = tk.StringVar(value="")
        self.clock_sync_port = tk.StringVar(value=str(DEFAULT_PORT))
        self.clock_sync_rate = tk.StringVar(value="2")
        self.clock_sync_status = tk.StringVar(value="DISCONNECTED • configure the LattePanda IP address")
        self.clock_sync_connection = tk.StringVar(value="● DISCONNECTED")
        self.clock_sync_metrics = tk.StringVar(value="No clock samples received")
        self.clock_sync_button_text = tk.StringVar(value="START CLOCK SYNC")
        self.dashboard_position_target = tk.StringVar(value="0")
        self.dashboard_position_units = tk.StringVar(value="degrees")
        self.dashboard_position_direction_choice = tk.StringVar(value="Automatic (target sign)")
        self.dashboard_position_speed = tk.StringVar(value="5")
        self.dashboard_position_command_status = tk.StringVar(value="Ready")
        self.dashboard_position_active = False
        self.dashboard_position_goal_deg: float | None = None
        self.dashboard_position_goal_label = ""
        self.dashboard_position_direction: int | None = None
        self.dashboard_position_started_at: float | None = None
        self.dashboard_position_timeout_s = 0.0
        self.dashboard_position_deceleration_rpm_s: float | None = None
        self.dashboard_position_braking = False
        self.dashboard_position_creep = False
        self.dashboard_position_after_id: str | None = None

        self.test_type = tk.StringVar(value="Speed sweep")
        self.test_start = tk.StringVar(value="-100")
        self.test_end = tk.StringVar(value="100")
        self.test_step = tk.StringVar(value="20")
        self.test_hold = tk.StringVar(value="5.0")
        self.test_cycles = tk.StringVar(value="1")
        self.test_tolerance = tk.StringVar(value="2.0")
        self.live_test_speed_cap = tk.StringVar(value="10.0")
        self.test_status = tk.StringVar(value="Ready")
        self.test_completed = tk.StringVar(value="0 / 0")
        self.test_step_result = tk.StringVar(value="--")
        self.live_test_arm_status = tk.StringVar(value="Live sequence not armed")
        self.live_test_confirmations = [tk.BooleanVar(value=False) for _ in range(4)]
        self.live_test_armed = False
        self.armed_live_speed_cap: float | None = None
        self.live_manual_arming = False
        self.preflight_state: dict[str, bool] = {}
        self.test_current_target: float | None = None
        self.test_step_started_at: float | None = None
        self.test_failures: list[str] = []
        self.motion_profile: MotionProfile | None = None
        self.compiled_profile: MotionProfile | None = None
        self.written_profile: MotionProfile | None = None
        self.compiled_pr_program: PrProgram | None = None
        self.pr_direction_control: int | None = None
        self.profile_path = tk.StringVar(value="No CSV selected")
        self.profile_type_text = tk.StringVar(value="AWAITING PROFILE")
        self.profile_detail_text = tk.StringVar(
            value=(
                "Speed: time_s,motor_rpm; "
                "Position: platform_position_deg,motor_speed_rpm,hold_s"
            )
        )
        self.profile_duration_text = tk.StringVar(value="--")
        self.profile_points_text = tk.StringVar(value="--")
        self.profile_speed_text = tk.StringVar(value="--")
        self.profile_ramp_text = tk.StringVar(value="--")
        self.profile_workflow_text = tk.StringVar(value="1  IMPORT   →   2  COMPILE   →   3  WRITE / VERIFY   →   4  START")
        self.profile_validation_text = tk.StringVar(value="Choose a CSV profile to begin")
        self.profile_running_started_at: float | None = None
        self.profile_stopping_started_at: float | None = None
        self.profile_zero_speed_since: float | None = None
        self.profile_last_zero_sample_time: float | None = None
        self.profile_run_after_id: str | None = None
        self.profile_cursor_s: float | None = None
        self.session_origin_pulses: int | None = None
        self.profile_run_origin_motor_deg: float | None = None
        self.session_origin_text = tk.StringVar(value="Captured automatically on connection")
        self.profile_stats_summary = tk.StringVar(value="• No profile imported")
        self.profile_channel_command = tk.BooleanVar(value=True)
        self.profile_channel_estimated_motor = tk.BooleanVar(value=True)
        self.profile_channel_estimated_platform = tk.BooleanVar(value=True)
        self.profile_channel_recorded_motor = tk.BooleanVar(value=True)
        self.profile_channel_recorded_platform = tk.BooleanVar(value=True)
        self.profile_channel_labels = [
            tk.StringVar(value="Motor Speed Command"),
            tk.StringVar(value="Estimated Motor Speed Profile"),
            tk.StringVar(value="Estimated Platform Speed Profile"),
            tk.StringVar(value="Recorded Motor Speed Feedback"),
            tk.StringVar(value="Recorded Platform Speed Feedback"),
        ]
        self.profile_homing_started_at: float | None = None
        self.profile_manual_home = False
        self.profile_home_previous_error: float | None = None
        self.profile_home_error_growth_count = 0
        self.profile_waypoint_index = 0
        self.profile_waypoint_direction: int | None = None
        self.profile_waypoint_started_at: float | None = None
        self.profile_waypoint_hold_started_at: float | None = None
        self.profile_waypoint_stop_latched = False
        self.profile_position_target_history: list[tuple[float, float]] = []
        self.profile_recording_active = False
        self.profile_recorded_samples: list[TelemetrySample] = []
        self.profile_clock_start_ns: int | None = None
        self.profile_clock_end_ns: int | None = None
        self.profile_run_clock: TestRunClock | None = None
        self.profile_run_coordinator: UdpRunCoordinatorClient | None = None
        self.profile_coordinator_monitor_stop = threading.Event()
        self.profile_local_events: list[dict[str, object]] = []
        self.profile_remote_events: list[dict[str, object]] = []
        self.profile_remote_metadata: dict[str, object] = {}
        self.profile_first_motion_command_recorded = False
        self.profile_safety_stop_timer: threading.Timer | None = None
        self.profile_safety_stop_lock = threading.Lock()
        self.profile_safety_stop_complete = threading.Event()
        self.profile_safety_stop_monotonic_s: float | None = None
        self.profile_execution_cancel = threading.Event()
        self.profile_execution_thread: threading.Thread | None = None

        self.channel_command = tk.BooleanVar(value=True)
        self.channel_feedback = tk.BooleanVar(value=True)
        self.channel_error = tk.BooleanVar(value=True)
        self.channel_platform = tk.BooleanVar(value=False)
        self.channel_platform_label = tk.StringVar(value="Platform speed 2:1 (calculated)")
        self.channel_motor_position = tk.BooleanVar(value=True)
        self.channel_platform_position = tk.BooleanVar(value=True)
        self.channel_platform_position_label = tk.StringVar(value="Platform sin(θ) 2:1 (calculated)")

        try:
            migrate_legacy_settings(self.settings_path)
            if self.settings_path.exists():
                self._apply_settings_payload(read_settings(self.settings_path))
        except (OSError, ValueError, TypeError) as exc:
            self.settings_load_error = str(exc)
        if self.auto_connect_simulation:
            self.mode.set(OperatingMode.SIMULATION.value)

        self._configure_styles()
        self._build_shell()
        self._build_pages()
        self._refresh_settings_dependent_widgets()
        self._set_connection_controls(False)
        self._set_disconnected_status()
        self._show_page(initial_page if initial_page in self.NAV_ITEMS else "Dashboard")
        self.root.after(20, self._process_events)
        if self.auto_connect_simulation:
            self.root.after(250, self._toggle_connection)

    @staticmethod
    def _report_callback_exception(exc_type, exc_value, exc_traceback) -> None:
        """Route Tk callback failures to app.py's persistent exception log."""
        sys.excepthook(exc_type, exc_value, exc_traceback)

    def run(self) -> None:
        self._log("Application ready", "INFO", "SYSTEM", f"Restored mode: {self.mode.get()}")
        if self.settings_load_error:
            self._log("Saved settings warning", "WARN", "SETTINGS", self.settings_load_error)
        self.root.mainloop()

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        self.root.configure(background="#F3F6F9")
        style.configure("App.TFrame", background="#F3F6F9")
        style.configure("Card.TFrame", background=SURFACE, relief="flat")
        style.configure("Card.TLabel", background=SURFACE, foreground=INK, font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background=SURFACE, foreground=MUTED, font=("Segoe UI", 9))
        style.configure("Section.TLabel", background=SURFACE, foreground=INK, font=("Segoe UI", 11, "bold"))
        style.configure("Page.TLabel", background=SURFACE, foreground=INK, font=("Segoe UI", 20, "bold"))
        style.configure("Value.TLabel", background=SURFACE, foreground=INK, font=("Segoe UI", 20, "bold"))
        style.configure("Speed.TEntry", fieldbackground="#FFFFFF", foreground=INK, bordercolor=BLUE, padding=8, font=("Segoe UI", 22, "bold"))
        style.configure("Primary.TButton", background=BLUE, foreground="#FFFFFF", font=("Segoe UI", 9, "bold"), padding=(12, 8))
        style.map("Primary.TButton", background=[("active", "#0759CA"), ("disabled", "#AABFDC")])
        style.configure("Outline.TButton", background="#FFFFFF", foreground=BLUE, font=("Segoe UI", 9, "bold"), padding=(12, 8))
        style.configure("IoField.TButton", background="#F7F9FC", foreground=INK, font=("Segoe UI", 8), padding=(7, 5), anchor="w")
        style.map("IoField.TButton", background=[("active", "#E8F1FD"), ("disabled", "#F1F3F6")], foreground=[("disabled", "#65758A")])
        style.configure("IoState.TButton", background="#EEF2F6", foreground=MUTED, font=("Segoe UI", 8, "bold"), padding=(7, 5))
        style.map("IoState.TButton", background=[("active", "#DCE9F9"), ("disabled", "#F1F3F6")], foreground=[("disabled", "#9AA5B2")])
        style.configure("Nudge.TButton", background="#EDF4FE", foreground=BLUE, font=("Segoe UI", 14, "bold"), padding=(8, 7))
        style.map("Nudge.TButton", background=[("active", "#D8E8FC"), ("disabled", "#E4E8ED")])
        style.configure("Stop.TButton", background=RED, foreground="#FFFFFF", font=("Segoe UI", 10, "bold"), padding=(12, 10))
        style.map("Stop.TButton", background=[("active", "#AE2330"), ("disabled", "#D9A8AD")])
        style.configure("Treeview", rowheight=27, background="#FFFFFF", fieldbackground="#FFFFFF", foreground=INK, borderwidth=0, font=("Segoe UI", 9))
        style.configure("Treeview.Heading", background="#F5F7FA", foreground=INK, font=("Segoe UI", 9, "bold"), relief="flat")
        style.map("Treeview", background=[("selected", "#DDEBFF")], foreground=[("selected", INK)])
        style.configure("TEntry", padding=5)
        style.configure("TCombobox", padding=4)
        style.configure("TCheckbutton", background=SURFACE, foreground=INK)
        style.configure("TRadiobutton", background=SURFACE, foreground=INK)

    def _build_shell(self) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        sidebar = tk.Frame(self.root, background=NAVY, width=170)
        sidebar.grid(row=0, column=0, sticky="ns")
        sidebar.grid_propagate(False)
        tk.Label(sidebar, text="SERVO\nTEST RIG", background=NAVY, foreground="#FFFFFF", font=("Segoe UI", 15, "bold"), justify="left").pack(anchor="w", padx=20, pady=(22, 26))
        self.nav_buttons: dict[str, tk.Button] = {}
        for name in self.NAV_ITEMS:
            button = tk.Button(
                sidebar,
                text=name,
                command=lambda page=name: self._show_page(page),
                anchor="w",
                padx=20,
                pady=11,
                borderwidth=0,
                relief="flat",
                background=NAVY,
                foreground="#E8F0F8",
                activebackground="#12436F",
                activeforeground="#FFFFFF",
                font=("Segoe UI", 10),
                cursor="hand2",
            )
            button.pack(fill="x", padx=7, pady=2)
            self.nav_buttons[name] = button

        main = ttk.Frame(self.root, style="App.TFrame")
        main.grid(row=0, column=1, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.rowconfigure(1, weight=1)
        header = ttk.Frame(main, style="Card.TFrame", padding=(22, 12))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, textvariable=self.page_title, style="Page.TLabel").grid(row=0, column=0, sticky="w")
        self.header_badge_label = tk.Label(header, textvariable=self.header_badge, background="#E9EEF4", foreground=INK, padx=14, pady=7, font=("Segoe UI", 10, "bold"))
        self.header_badge_label.grid(row=0, column=1, sticky="e")

        self.page_container = ttk.Frame(main, style="App.TFrame", padding=(18, 14))
        self.page_container.grid(row=1, column=0, sticky="nsew")
        self.page_container.columnconfigure(0, weight=1)
        self.page_container.rowconfigure(0, weight=1)
        self.pages: dict[str, ttk.Frame] = {}

    def _build_pages(self) -> None:
        builders = {
            "Dashboard": self._build_dashboard,
            "Motion": self._build_motion,
            "Tests": self._build_tests,
            "Trends": self._build_trends,
            "Parameters": self._build_parameters,
            "Logs": self._build_logs,
            "Settings": self._build_settings,
        }
        for name, builder in builders.items():
            frame = ttk.Frame(self.page_container, style="App.TFrame")
            frame.grid(row=0, column=0, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            self.pages[name] = frame
            builder(frame)

    def _show_page(self, name: str) -> None:
        self.pages[name].tkraise()
        titles = {
            "Dashboard": "Servo Test Rig",
            "Motion": "Motion Control",
            "Tests": "Motion Profiles",
            "Trends": "Trends & Tracking",
            "Parameters": "Drive Parameters",
            "Logs": "Event & Alarm Logs",
            "Settings": "Application Settings",
        }
        self.page_title.set(titles[name])
        for page, button in self.nav_buttons.items():
            button.configure(background=BLUE if page == name else NAVY, foreground="#FFFFFF" if page == name else "#E8F0F8")
        if name in ("Dashboard", "Trends"):
            self.root.after(20, self._redraw_charts)

    def _card(self, parent: tk.Misc, padding: int | tuple[int, ...] = 14) -> ttk.Frame:
        return ttk.Frame(parent, style="Card.TFrame", padding=padding)

    def _status_card(
        self,
        parent: tk.Misc,
        title: str,
        value: tk.StringVar,
        detail: tk.StringVar,
        column: int,
        command: Callable[[], None] | None = None,
    ) -> ttk.Frame:
        card = self._card(parent, 14)
        card.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 6, 0 if column == 2 else 6))
        card.columnconfigure(1, weight=1)
        lamp = tk.Label(card, text="✓", background=GREEN, foreground="#FFFFFF", font=("Segoe UI", 16, "bold"), width=2, height=1)
        lamp.grid(row=0, column=0, rowspan=2, padx=(0, 12))
        self.status_lamps[title] = lamp
        title_label = ttk.Label(card, text=title, style="Section.TLabel")
        title_label.grid(row=0, column=1, sticky="w")
        value_label = ttk.Label(card, textvariable=value, style="Card.TLabel")
        value_label.grid(row=0, column=2, sticky="e")
        detail_label = ttk.Label(card, textvariable=detail, style="Muted.TLabel")
        detail_label.grid(row=1, column=1, columnspan=2, sticky="w")
        if command is not None:
            for widget in (card, lamp, title_label, value_label, detail_label):
                widget.configure(cursor="hand2")
                widget.bind("<Button-1>", lambda _event: command())
        return card

    def _set_status_lamp(self, title: str, state: str) -> None:
        if title not in self.status_lamps:
            return
        appearance = {
            "ok": ("✓", GREEN),
            "warn": ("!", ORANGE),
            "fault": ("!", RED),
            "unknown": ("?", MUTED),
            "info": ("i", BLUE),
        }
        symbol, color = appearance[state]
        self.status_lamps[title].configure(text=symbol, background=color)

    def _build_dashboard(self, page: ttk.Frame) -> None:
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.dashboard_canvas = tk.Canvas(page, background="#F3F6F9", borderwidth=0, highlightthickness=0)
        dashboard_scrollbar = ttk.Scrollbar(page, orient="vertical", command=self.dashboard_canvas.yview)
        self.dashboard_canvas.configure(yscrollcommand=dashboard_scrollbar.set)
        self.dashboard_canvas.grid(row=0, column=0, sticky="nsew")
        dashboard_scrollbar.grid(row=0, column=1, sticky="ns")
        body = ttk.Frame(self.dashboard_canvas, style="App.TFrame")
        dashboard_window = self.dashboard_canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda _event: self.dashboard_canvas.configure(scrollregion=self.dashboard_canvas.bbox("all")))
        self.dashboard_canvas.bind("<Configure>", lambda event: self.dashboard_canvas.itemconfigure(dashboard_window, width=event.width))
        body.columnconfigure((0, 1, 2), weight=1)
        body.rowconfigure(1, weight=3)
        body.rowconfigure(2, weight=2)
        body.rowconfigure(3, weight=1)
        self._status_card(body, "DRIVE", self.status_drive, self.status_drive_detail, 0)
        mode_card = self._card(body, 14)
        mode_card.grid(row=0, column=1, sticky="nsew", padx=6)
        mode_card.columnconfigure(0, weight=1)
        ttk.Label(mode_card, text="CONTROL MODE", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(mode_card, textvariable=self.control_mode_text, style="Value.TLabel").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(mode_card, textvariable=self.control_mode_detail, style="Muted.TLabel").grid(row=2, column=0, sticky="w", pady=(4, 0))
        self._status_card(body, "ALARM", self.status_alarm, self.status_alarm_detail, 2)

        gauge_card = self._card(body, 12)
        gauge_card.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(12, 6))
        gauge_card.columnconfigure(0, weight=1)
        gauge_card.rowconfigure(1, weight=1)
        ttk.Label(gauge_card, text="FEEDBACK SPEED", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.dashboard_gauge = SpeedGauge(gauge_card)
        self.dashboard_gauge.grid(row=1, column=0, sticky="nsew")

        control = self._card(body, 16)
        control.grid(row=1, column=1, sticky="nsew", padx=6, pady=(12, 6))
        control.columnconfigure(0, weight=1)
        ttk.Label(control, text="SPEED COMMAND", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        speed_row = ttk.Frame(control, style="Card.TFrame")
        speed_row.grid(row=1, column=0, sticky="ew", pady=(16, 4))
        speed_row.columnconfigure(1, weight=1)
        self.dashboard_minus = ttk.Button(speed_row, text="−1", command=lambda: self._nudge_speed(-self.SPEED_NUDGE_RPM), style="Nudge.TButton", width=4)
        self.dashboard_minus.grid(row=0, column=0, sticky="ns", padx=(0, 7))
        self.dashboard_speed_entry = ttk.Entry(speed_row, textvariable=self.speed_entry, justify="center", style="Speed.TEntry")
        self.dashboard_speed_entry.grid(row=0, column=1, sticky="ew")
        self.dashboard_plus = ttk.Button(speed_row, text="+1", command=lambda: self._nudge_speed(self.SPEED_NUDGE_RPM), style="Nudge.TButton", width=4)
        self.dashboard_plus.grid(row=0, column=2, sticky="ns", padx=(7, 0))
        self.dashboard_speed_entry.bind("<Return>", lambda _event: self._send_speed_from_entry())
        self.dashboard_speed_entry.bind("<FocusOut>", lambda _event: self._send_speed_from_entry(silent=True))
        ttk.Label(control, text="Type, drag, or nudge by ±1 rpm - updates immediately", style="Muted.TLabel").grid(row=2, column=0, sticky="w")
        self.dashboard_slider = ttk.Scale(control, from_=-100, to=100, variable=self.speed_scale, command=self._on_speed_scale)
        self.dashboard_slider.grid(row=3, column=0, sticky="ew", pady=(14, 2))
        scale_row = ttk.Frame(control, style="Card.TFrame")
        scale_row.grid(row=4, column=0, sticky="ew")
        scale_row.columnconfigure(1, weight=1)
        ttk.Label(scale_row, textvariable=self.minimum_scale_text, style="Muted.TLabel").grid(row=0, column=0)
        ttk.Label(scale_row, textvariable=self.midpoint_scale_text, style="Muted.TLabel").grid(row=0, column=1)
        ttk.Label(scale_row, textvariable=self.maximum_scale_text, style="Muted.TLabel").grid(row=0, column=2)
        self.dashboard_stop = ttk.Button(control, text="CONTROLLED STOP", command=self._controlled_stop, style="Stop.TButton")
        self.dashboard_stop.grid(row=5, column=0, sticky="ew", pady=(18, 5))
        ttk.Label(control, text="Ramps the speed command to 0 rpm using the configured deceleration", style="Muted.TLabel", wraplength=360).grid(row=6, column=0, sticky="w")

        connection = self._card(body, 16)
        connection.grid(row=1, column=2, sticky="nsew", padx=(6, 0), pady=(12, 6))
        connection.columnconfigure(0, weight=1)
        ttk.Label(connection, text="CONNECTION", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        mode_switch = ttk.Frame(connection, style="Card.TFrame")
        mode_switch.grid(row=1, column=0, sticky="ew", pady=(20, 8))
        mode_switch.columnconfigure((0, 1), weight=1)
        self.dashboard_simulation_mode = ttk.Button(mode_switch, text="SIMULATION", command=lambda: self._select_connection_mode(OperatingMode.SIMULATION.value), style="Outline.TButton")
        self.dashboard_simulation_mode.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        self.dashboard_live_mode = ttk.Button(mode_switch, text="LIVE DRIVE", command=lambda: self._select_connection_mode(OperatingMode.LIVE.value), style="Outline.TButton")
        self.dashboard_live_mode.grid(row=0, column=1, sticky="ew", padx=(3, 0))
        ttk.Label(connection, textvariable=self.connection_text, style="Value.TLabel").grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 2))
        self.connection_detail = tk.StringVar(value="No active connection")
        ttk.Label(connection, textvariable=self.connection_detail, style="Muted.TLabel", wraplength=260).grid(row=3, column=0, columnspan=2, sticky="w")
        buttons = ttk.Frame(connection, style="Card.TFrame")
        buttons.grid(row=4, column=0, columnspan=2, sticky="sew", pady=(30, 0))
        buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(buttons, text="EDIT CONNECTION", command=lambda: self._show_page("Settings"), style="Outline.TButton").grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.dashboard_connect = ttk.Button(buttons, text="CONNECT", command=self._toggle_connection, style="Primary.TButton")
        self.dashboard_connect.grid(row=0, column=1, sticky="ew", padx=(5, 0))
        self.dashboard_live_control = ttk.Button(
            connection,
            text="ENABLE LIVE CONTROL",
            command=self._toggle_live_manual_control,
            style="Outline.TButton",
        )
        self.dashboard_live_control.grid(row=5, column=0, sticky="ew", pady=(10, 0))

        position_feedback = self._card(body, 14)
        position_feedback.grid(row=2, column=0, sticky="nsew", padx=(0, 6), pady=6)
        position_feedback.columnconfigure(0, weight=1)
        position_feedback.rowconfigure(1, weight=1)
        ttk.Label(position_feedback, text="PLATFORM POSITION", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.dashboard_position_compass = MotionCompass(position_feedback)
        self.dashboard_position_compass.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
        ttk.Label(position_feedback, textvariable=self.dashboard_platform_position_text, style="Value.TLabel").grid(row=2, column=0, sticky="w", pady=(4, 0))

        position_command = self._card(body, 16)
        position_command.grid(row=2, column=1, sticky="nsew", padx=6, pady=6)
        position_command.columnconfigure(1, weight=1)
        ttk.Label(position_command, text="POSITION COMMAND — SPEED MODE", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(position_command, text="Target", style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(14, 4))
        ttk.Entry(position_command, textvariable=self.dashboard_position_target, justify="right").grid(row=1, column=1, sticky="ew", pady=(14, 4), padx=(8, 0))
        ttk.Label(position_command, text="Units", style="Muted.TLabel").grid(row=2, column=0, sticky="w", pady=4)
        ttk.Combobox(position_command, textvariable=self.dashboard_position_units, values=("degrees", "revolutions"), state="readonly").grid(row=2, column=1, sticky="ew", pady=4, padx=(8, 0))
        ttk.Label(position_command, text="Direction", style="Muted.TLabel").grid(row=3, column=0, sticky="w", pady=4)
        ttk.Combobox(
            position_command,
            textvariable=self.dashboard_position_direction_choice,
            values=("Automatic (target sign)", "Positive / anticlockwise", "Negative / clockwise"),
            state="readonly",
        ).grid(row=3, column=1, sticky="ew", pady=4, padx=(8, 0))
        ttk.Label(position_command, text="Target motor speed", style="Muted.TLabel").grid(row=4, column=0, sticky="w", pady=4)
        ttk.Entry(position_command, textvariable=self.dashboard_position_speed, justify="right").grid(row=4, column=1, sticky="ew", pady=4, padx=(8, 0))
        ttk.Label(position_command, text="rpm", style="Muted.TLabel").grid(row=5, column=1, sticky="e")
        position_buttons = ttk.Frame(position_command, style="Card.TFrame")
        position_buttons.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(12, 5))
        position_buttons.columnconfigure((0, 1), weight=1)
        self.dashboard_position_go = ttk.Button(position_buttons, text="GO TO POSITION", command=self._start_dashboard_position_command, style="Primary.TButton")
        self.dashboard_position_go.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.dashboard_position_home = ttk.Button(position_buttons, text="RETURN HOME", command=self._start_profile_home, style="Outline.TButton")
        self.dashboard_position_home.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        ttk.Button(position_command, text="STOP", command=self._stop_dashboard_position_command, style="Stop.TButton").grid(row=7, column=0, columnspan=2, sticky="ew", pady=(4, 6))
        ttk.Label(position_command, textvariable=self.dashboard_position_command_status, style="Muted.TLabel", wraplength=360).grid(row=8, column=0, columnspan=2, sticky="w")

        labjack_card = self._card(body, 16)
        labjack_card.grid(row=2, column=2, sticky="nsew", padx=(6, 0), pady=6)
        labjack_card.columnconfigure(0, weight=1)
        ttk.Label(labjack_card, text="SCENE VERTICAL DISPLACEMENT", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(labjack_card, textvariable=self.dashboard_displacement_mm_text, style="Value.TLabel").grid(row=1, column=0, sticky="w", pady=(18, 2))
        ttk.Label(labjack_card, textvariable=self.dashboard_displacement_voltage_text, style="Card.TLabel").grid(row=2, column=0, sticky="w", pady=(2, 6))
        ttk.Label(labjack_card, textvariable=self.labjack_status, style="Muted.TLabel", wraplength=300).grid(row=3, column=0, sticky="w", pady=(4, 12))
        self.dashboard_labjack_connect_button = ttk.Button(labjack_card, text="CONNECT LABJACK", command=self._toggle_labjack, style="Primary.TButton")
        self.dashboard_labjack_connect_button.grid(row=4, column=0, sticky="ew")

        ttk.Separator(labjack_card).grid(row=5, column=0, sticky="ew", pady=14)
        ttk.Label(labjack_card, text="LATTEPANDA CLOCK SYNC", style="Section.TLabel").grid(row=6, column=0, sticky="w")
        ttk.Label(labjack_card, text="LattePanda IP address", style="Muted.TLabel").grid(row=7, column=0, sticky="w", pady=(10, 4))
        # Both pages edit the same saved address and control the same client.
        self.dashboard_clock_sync_host_entry = ttk.Entry(labjack_card, textvariable=self.clock_sync_host)
        self.dashboard_clock_sync_host_entry.grid(row=8, column=0, sticky="ew")
        self.dashboard_clock_sync_button = ttk.Button(
            labjack_card, textvariable=self.clock_sync_button_text,
            command=self._toggle_clock_sync, style="Primary.TButton",
        )
        self.dashboard_clock_sync_button.grid(row=9, column=0, sticky="ew", pady=(10, 6))
        ttk.Label(labjack_card, textvariable=self.clock_sync_connection, style="Card.TLabel").grid(row=10, column=0, sticky="w")
        ttk.Label(labjack_card, textvariable=self.clock_sync_status, style="Muted.TLabel", wraplength=300).grid(row=11, column=0, sticky="w", pady=(4, 0))

        trend = self._card(body, (12, 8))
        trend.grid(row=3, column=0, columnspan=3, sticky="nsew", pady=6)
        trend.columnconfigure(0, weight=1)
        trend.rowconfigure(1, weight=1)
        ttk.Label(trend, text="MOTOR COMMAND / ENCODER FEEDBACK / PLATFORM SPEED", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.dashboard_trend = TrendChart(trend, compact=True)
        self.dashboard_trend.set_channels(True, True, False, True)
        self.dashboard_trend.grid(row=1, column=0, sticky="nsew")

        log_card = self._card(body, (12, 8))
        log_card.grid(row=4, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        log_card.columnconfigure(0, weight=1)
        ttk.Label(log_card, text="EVENT LOG", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Button(log_card, text="View all logs", command=lambda: self._show_page("Logs"), style="Outline.TButton").grid(row=0, column=1, sticky="e")
        self.dashboard_log_tree = ttk.Treeview(log_card, columns=("time", "level", "event"), show="headings", height=3)
        self.dashboard_log_tree.heading("time", text="Time")
        self.dashboard_log_tree.heading("level", text="Level")
        self.dashboard_log_tree.heading("event", text="Event")
        self.dashboard_log_tree.column("time", width=100, stretch=False)
        self.dashboard_log_tree.column("level", width=90, stretch=False)
        self.dashboard_log_tree.column("event", width=700)
        self.dashboard_log_tree.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))

    def _build_motion(self, page: ttk.Frame) -> None:
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.motion_canvas = tk.Canvas(page, background="#F3F6F9", borderwidth=0, highlightthickness=0)
        motion_scrollbar = ttk.Scrollbar(page, orient="vertical", command=self.motion_canvas.yview)
        self.motion_canvas.configure(yscrollcommand=motion_scrollbar.set)
        self.motion_canvas.grid(row=0, column=0, sticky="nsew")
        motion_scrollbar.grid(row=0, column=1, sticky="ns")

        body = ttk.Frame(self.motion_canvas, style="App.TFrame")
        motion_window = self.motion_canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind(
            "<Configure>",
            lambda _event: self.motion_canvas.configure(scrollregion=self.motion_canvas.bbox("all")),
        )
        self.motion_canvas.bind(
            "<Configure>",
            lambda event: self.motion_canvas.itemconfigure(motion_window, width=event.width),
        )
        body.columnconfigure((0, 1, 2), weight=1)
        body.rowconfigure(0, weight=3)
        body.rowconfigure(1, weight=2)
        body.rowconfigure(2, minsize=140)

        manual = self._card(body, 16)
        manual.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        manual.columnconfigure(0, weight=1)
        ttk.Label(manual, text="MANUAL SPEED CONTROL", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        motion_speed_row = ttk.Frame(manual, style="Card.TFrame")
        motion_speed_row.grid(row=1, column=0, sticky="ew", pady=(18, 3))
        motion_speed_row.columnconfigure(1, weight=1)
        self.motion_minus = ttk.Button(motion_speed_row, text="−1", command=lambda: self._nudge_speed(-self.SPEED_NUDGE_RPM), style="Nudge.TButton", width=4)
        self.motion_minus.grid(row=0, column=0, sticky="ns", padx=(0, 7))
        self.motion_speed_entry = ttk.Entry(motion_speed_row, textvariable=self.speed_entry, justify="center", style="Speed.TEntry")
        self.motion_speed_entry.grid(row=0, column=1, sticky="ew")
        self.motion_plus = ttk.Button(motion_speed_row, text="+1", command=lambda: self._nudge_speed(self.SPEED_NUDGE_RPM), style="Nudge.TButton", width=4)
        self.motion_plus.grid(row=0, column=2, sticky="ns", padx=(7, 0))
        self.motion_speed_entry.bind("<Return>", lambda _event: self._send_speed_from_entry())
        self.motion_speed_entry.bind("<FocusOut>", lambda _event: self._send_speed_from_entry(silent=True))
        ttk.Label(manual, text="Updates immediately", style="Muted.TLabel").grid(row=2, column=0)
        self.motion_slider = ttk.Scale(manual, from_=-100, to=100, variable=self.speed_scale, command=self._on_speed_scale)
        self.motion_slider.grid(row=3, column=0, sticky="ew", pady=(14, 10))
        metrics = ttk.Frame(manual, style="Card.TFrame")
        metrics.grid(row=4, column=0, sticky="ew", pady=8)
        metrics.columnconfigure((0, 1), weight=1)
        ttk.Label(metrics, text="ENCODER FEEDBACK", style="Muted.TLabel").grid(row=0, column=0)
        ttk.Label(metrics, text="DIRECTION", style="Muted.TLabel").grid(row=0, column=1)
        ttk.Label(metrics, textvariable=self.feedback_text, style="Value.TLabel").grid(row=1, column=0)
        ttk.Label(metrics, textvariable=self.direction_text, style="Value.TLabel").grid(row=1, column=1)
        self.motion_stop = ttk.Button(manual, text="CONTROLLED STOP", command=self._controlled_stop, style="Stop.TButton")
        self.motion_stop.grid(row=5, column=0, sticky="ew", pady=(14, 3))
        ttk.Label(manual, text="Ramps to 0 rpm using P1.035", style="Muted.TLabel").grid(row=6, column=0)

        ramp = self._card(body, 16)
        ramp.grid(row=0, column=1, sticky="nsew", padx=6)
        ramp.columnconfigure(1, weight=1)
        ttk.Label(ramp, text="RAMP SETTINGS", style="Section.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(ramp, text="Acceleration", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=(20, 8))
        ttk.Entry(ramp, textvariable=self.acceleration_s, width=10, justify="right").grid(row=1, column=1, sticky="ew", pady=(20, 8))
        ttk.Label(ramp, text="s", style="Card.TLabel").grid(row=1, column=2, padx=5, pady=(20, 8))
        ttk.Label(ramp, text="Deceleration", style="Card.TLabel").grid(row=2, column=0, sticky="w", pady=8)
        ttk.Entry(ramp, textvariable=self.deceleration_s, width=10, justify="right").grid(row=2, column=1, sticky="ew", pady=8)
        ttk.Label(ramp, text="s", style="Card.TLabel").grid(row=2, column=2, padx=5, pady=8)
        self.ramp_canvas = tk.Canvas(ramp, height=160, background=SURFACE, highlightthickness=0)
        self.ramp_canvas.grid(row=3, column=0, columnspan=3, sticky="nsew", pady=12)
        self.ramp_canvas.bind("<Configure>", lambda _event: self._draw_ramp_preview())
        ttk.Label(ramp, text="Operator times are converted to the manual's 0-to-3000-rpm basis", style="Muted.TLabel", wraplength=330).grid(row=4, column=0, columnspan=3, sticky="w")
        ttk.Label(ramp, textvariable=self.ramp_limits_text, style="Muted.TLabel", wraplength=330).grid(row=5, column=0, columnspan=3, sticky="w", pady=(6, 0))
        self.ramp_apply_button = ttk.Button(
            ramp,
            text="APPLY RAMPS",
            command=self._apply_ramps,
            style="Primary.TButton",
        )
        self.ramp_apply_button.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(12, 0))

        limits = self._card(body, 16)
        limits.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        limits.columnconfigure(1, weight=1)
        ttk.Label(limits, text="OPERATING LIMITS", style="Section.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")
        limit_fields = (
            ("± Speed", self.speed_limit_magnitude, "rpm"),
            ("Motor-to-platform", self.platform_ratio, ":1"),
        )
        for row, (label, variable, unit) in enumerate(limit_fields, start=1):
            ttk.Label(limits, text=label, style="Card.TLabel").grid(row=row, column=0, sticky="w", pady=(15 if row == 1 else 7, 4))
            ttk.Entry(limits, textvariable=variable, justify="right", width=9).grid(row=row, column=1, sticky="ew", padx=(8, 5), pady=(15 if row == 1 else 7, 4))
            ttk.Label(limits, text=unit, style="Muted.TLabel").grid(row=row, column=2, sticky="w", pady=(15 if row == 1 else 7, 4))
        ttk.Label(limits, text="CSV positive", style="Card.TLabel").grid(row=3, column=0, sticky="w", pady=7)
        ttk.Combobox(
            limits,
            textvariable=self.platform_command_direction,
            values=("Motor positive (+PUU)", "Motor negative (-PUU)"),
            state="readonly",
            width=20,
        ).grid(row=3, column=1, columnspan=2, sticky="ew", padx=(8, 0), pady=7)
        ttk.Button(limits, text="APPLY OPERATOR LIMITS", command=self._apply_operator_limits, style="Outline.TButton").grid(
            row=4, column=0, columnspan=3, sticky="ew", pady=(10, 4)
        )
        ttk.Label(limits, text="Symmetric session range • software hard guard ±1000 rpm", style="Muted.TLabel").grid(
            row=5, column=0, columnspan=3, sticky="w"
        )
        ttk.Separator(limits).grid(row=6, column=0, columnspan=3, sticky="ew", pady=9)
        ttk.Label(limits, text="PLATFORM SPEED", style="Muted.TLabel").grid(row=7, column=0, columnspan=3, sticky="w")
        ttk.Label(limits, textvariable=self.platform_speed_text, style="Value.TLabel").grid(row=8, column=0, columnspan=3, sticky="w", pady=5)

        self.motion_lock_note = tk.StringVar(value="Connect to Simulation for command testing")
        lower = ttk.Frame(body, style="App.TFrame")
        lower.grid(row=1, column=0, columnspan=3, sticky="nsew", pady=(12, 0))
        lower.columnconfigure(0, weight=2)
        lower.columnconfigure(1, weight=1)

        self.motion_io_card = self._card(lower, 16)
        self.motion_io_card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self._build_motion_io_panel(self.motion_io_card)

        history = self._card(lower, 16)
        history.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        history.columnconfigure(0, weight=1)
        history.rowconfigure(1, weight=1)
        ttk.Label(history, text="RECENT COMMAND HISTORY", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.motion_history_tree = ttk.Treeview(history, columns=("time", "command", "result"), show="headings", height=18)
        for column, text, width in (("time", "Time", 105), ("command", "Command", 280), ("result", "Result", 110)):
            self.motion_history_tree.heading(column, text=text)
            self.motion_history_tree.column(column, width=width, stretch=column == "command")
        self.motion_history_tree.grid(row=1, column=0, sticky="nsew", pady=(8, 0))

    def _build_motion_io_panel(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(2, weight=1)
        header = ttk.Frame(parent, style="Card.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="DIGITAL I/O", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.io_edit_button = ttk.Button(header, text="EDIT", command=self._toggle_io_edit, style="Outline.TButton", state="disabled")
        self.io_edit_button.grid(row=0, column=1, padx=(8, 4))
        self.io_apply_button = ttk.Button(header, text="APPLY", command=self._apply_io_configuration, style="Primary.TButton", state="disabled")
        self.io_apply_button.grid(row=0, column=2, padx=(4, 0))
        ttk.Label(parent, textvariable=self.io_detail_summary, style="Muted.TLabel").grid(
            row=1, column=0, sticky="w", pady=(6, 8)
        )

        notebook = ttk.Notebook(parent)
        notebook.grid(row=2, column=0, sticky="nsew")
        inputs = ttk.Frame(notebook, style="Card.TFrame", padding=(4, 6))
        outputs = ttk.Frame(notebook, style="Card.TFrame", padding=(4, 6))
        notebook.add(inputs, text="  INPUTS (DI1-DI13)  ")
        notebook.add(outputs, text="  OUTPUTS (DO1-DO6)  ")
        inputs.columnconfigure((0, 1), weight=1, uniform="di_points")
        outputs.columnconfigure((0, 1), weight=1, uniform="do_points")
        self.io_di_state_labels.clear()
        self.io_di_lamps.clear()
        self.io_di_source_controls.clear()
        self.io_di_function_controls.clear()
        self.io_di_contact_controls.clear()
        self.io_di_state_controls.clear()
        for index in range(1, 14):
            row, column = divmod(index - 1, 2)
            card = tk.Frame(inputs, background="#F6F8FB", highlightbackground="#D8E0E9", highlightthickness=1)
            card.grid(row=row, column=column, sticky="nsew", padx=(0 if column == 0 else 4, 4 if column == 0 else 0), pady=4)
            card.columnconfigure((0, 1, 2), weight=1)
            lamp = tk.Label(card, text="●", background="#F6F8FB", foreground="#A9B4C2", font=("Segoe UI", 13, "bold"))
            lamp.grid(row=0, column=0, sticky="w", padx=(8, 3), pady=(6, 1))
            self.io_di_lamps.append(lamp)
            tk.Label(card, text=f"DI{index}", background="#F6F8FB", foreground=INK, font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w", padx=(27, 3), pady=(6, 1))
            state_label = tk.Label(card, textvariable=self.io_di_values[index - 1], background="#F6F8FB", foreground=MUTED, font=("Segoe UI", 8))
            state_label.grid(row=0, column=1, columnspan=2, sticky="e", padx=(3, 8), pady=(6, 1))
            self.io_di_state_labels.append(state_label)
            function_control = ttk.Button(card, textvariable=self.io_di_function_choices[index - 1], style="IoField.TButton", state="disabled")
            function_control.configure(command=lambda button=function_control, variable=self.io_di_function_choices[index - 1]: self._show_function_menu(button, variable, self.DI_FUNCTION_CATALOG))
            function_control.grid(row=1, column=0, columnspan=3, sticky="ew", padx=7, pady=(4, 3))
            self.io_di_function_controls.append(function_control)
            contact_control = ttk.Button(card, textvariable=self.io_di_contact_choices[index - 1], command=lambda variable=self.io_di_contact_choices[index - 1]: self._toggle_io_contact(variable), style="IoField.TButton", state="disabled")
            contact_control.grid(row=2, column=0, sticky="ew", padx=(7, 3), pady=(3, 7))
            self.io_di_contact_controls.append(contact_control)
            source_control = ttk.Button(card, textvariable=self.io_di_source_choices[index - 1], command=lambda channel=index - 1: self._toggle_di_source(channel), style="IoField.TButton", state="disabled")
            source_control.grid(row=2, column=1, sticky="ew", padx=3, pady=(3, 7))
            self.io_di_source_controls.append(source_control)
            state_control = ttk.Button(card, textvariable=self.io_di_state_button_texts[index - 1], command=lambda channel=index - 1: self._toggle_di_software_state(channel), style="IoState.TButton", state="disabled")
            state_control.grid(row=2, column=2, sticky="ew", padx=(3, 7), pady=(3, 7))
            self.io_di_state_controls.append(state_control)
        ttk.Label(
            inputs,
            text="Buttons: function • NO/NC contact • HW/SW source • software TRUE/FALSE. In Live Drive mode, the DI assigned EMGS must be normally closed and hardware-sourced; EMGS can be moved to any DI.",
            style="Muted.TLabel",
            wraplength=720,
        ).grid(row=7, column=0, columnspan=2, sticky="w", pady=(7, 2))

        self.io_do_state_labels.clear()
        self.io_do_lamps.clear()
        self.io_do_function_controls.clear()
        self.io_do_contact_controls.clear()
        self.io_do_state_controls.clear()
        for index in range(1, 7):
            row, column = divmod(index - 1, 2)
            card = tk.Frame(outputs, background="#F6F8FB", highlightbackground="#D8E0E9", highlightthickness=1)
            card.grid(row=row, column=column, sticky="nsew", padx=(0 if column == 0 else 4, 4 if column == 0 else 0), pady=4)
            card.columnconfigure((0, 1, 2), weight=1)
            lamp = tk.Label(card, text="●", background="#F6F8FB", foreground="#A9B4C2", font=("Segoe UI", 13, "bold"))
            lamp.grid(row=0, column=0, sticky="w", padx=(8, 3), pady=(6, 1))
            self.io_do_lamps.append(lamp)
            tk.Label(card, text=f"DO{index}", background="#F6F8FB", foreground=INK, font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w", padx=(30, 3), pady=(6, 1))
            state_label = tk.Label(card, textvariable=self.io_do_values[index - 1], background="#F6F8FB", foreground=MUTED, font=("Segoe UI", 8))
            state_label.grid(row=0, column=1, columnspan=2, sticky="e", padx=(3, 8), pady=(6, 1))
            self.io_do_state_labels.append(state_label)
            function_control = ttk.Button(card, textvariable=self.io_do_function_choices[index - 1], style="IoField.TButton", state="disabled")
            function_control.configure(command=lambda button=function_control, variable=self.io_do_function_choices[index - 1]: self._show_function_menu(button, variable, self.DO_FUNCTION_CATALOG))
            function_control.grid(row=1, column=0, columnspan=3, sticky="ew", padx=7, pady=(4, 3))
            self.io_do_function_controls.append(function_control)
            contact_control = ttk.Button(card, textvariable=self.io_do_contact_choices[index - 1], command=lambda variable=self.io_do_contact_choices[index - 1]: self._toggle_io_contact(variable), style="IoField.TButton", state="disabled")
            contact_control.grid(row=2, column=0, sticky="ew", padx=(7, 3), pady=(3, 7))
            self.io_do_contact_controls.append(contact_control)
            tk.Label(card, textvariable=self.io_do_source_texts[index - 1], background="#F6F8FB", foreground=MUTED, font=("Segoe UI", 8)).grid(row=2, column=1, sticky="ew", padx=3, pady=(3, 7))
            state_control = ttk.Button(card, textvariable=self.io_do_state_button_texts[index - 1], command=lambda channel=index - 1: self._toggle_do_software_state(channel), style="IoState.TButton", state="disabled")
            state_control.grid(row=2, column=2, sticky="ew", padx=(3, 7), pady=(3, 7))
            self.io_do_state_controls.append(state_control)
        ttk.Label(
            outputs,
            text="DO readback is P4.009. Software state applies only when the selected meaning is SPO_0 to SPO_F (P4.006); this does not force ordinary drive outputs.",
            style="Muted.TLabel",
            wraplength=720,
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(7, 2))

    def _build_tests(self, page: ttk.Frame) -> None:
        # Visible workflow is Import -> Write/Verify -> Start/Abort. Legacy sweep
        # helpers remain below but are not a separate visible workflow on this page.
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.test_canvas = tk.Canvas(page, background="#F3F6F9", borderwidth=0, highlightthickness=0)
        test_scrollbar = ttk.Scrollbar(page, orient="vertical", command=self.test_canvas.yview)
        self.test_canvas.configure(yscrollcommand=test_scrollbar.set)
        self.test_canvas.grid(row=0, column=0, sticky="nsew")
        test_scrollbar.grid(row=0, column=1, sticky="ns")
        body = ttk.Frame(self.test_canvas, style="App.TFrame")
        test_window = self.test_canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda _event: self.test_canvas.configure(scrollregion=self.test_canvas.bbox("all")))
        self.test_canvas.bind("<Configure>", lambda event: self.test_canvas.itemconfigure(test_window, width=event.width))
        body.columnconfigure(0, weight=2)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)

        workflow = self._card(body, 16)
        workflow.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        workflow.columnconfigure((0, 1, 2), weight=1)
        ttk.Label(workflow, text="MOTION PROFILE", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(workflow, textvariable=self.profile_type_text, foreground=BLUE, background=SURFACE, font=("Segoe UI", 9, "bold")).grid(row=0, column=2, sticky="e")
        ttk.Label(workflow, textvariable=self.profile_stats_summary, style="Muted.TLabel").grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 12))
        self.profile_import_button = ttk.Button(workflow, text="IMPORT CSV", command=self._import_motion_profile, style="Primary.TButton")
        self.profile_import_button.grid(row=2, column=0, sticky="ew", padx=(0, 5))
        self.profile_write_button = ttk.Button(workflow, text="WRITE / VERIFY", command=self._write_motion_profile, style="Outline.TButton", state="disabled")
        self.profile_write_button.grid(row=2, column=1, sticky="ew", padx=5)
        self.test_start_button = ttk.Button(workflow, text="START", command=self._start_or_abort_motion_profile, style="Primary.TButton", state="disabled")
        self.test_start_button.grid(row=2, column=2, sticky="ew", padx=(5, 0))
        self.test_abort_button = self.test_start_button
        ttk.Label(workflow, textvariable=self.profile_workflow_text, style="Muted.TLabel").grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(10, 2)
        )
        ttk.Label(workflow, textvariable=self.profile_validation_text, style="Muted.TLabel", wraplength=1050).grid(
            row=4, column=0, columnspan=3, sticky="w", pady=(0, 6)
        )
        self.profile_home_button = ttk.Button(
            workflow,
            text="HOME TO 0°",
            command=self._start_profile_home,
            style="Outline.TButton",
            state="disabled",
        )
        self.profile_home_button.grid(row=5, column=0, sticky="ew", padx=(0, 5))
        # Kept as a disabled compatibility handle for the older arming code.
        self.live_test_arm_button = ttk.Button(workflow, text="LIVE PROFILE ARMING", state="disabled")

        preview_card = self._card(body, 16)
        preview_card.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        preview_card.columnconfigure(0, weight=1)
        preview_card.rowconfigure(2, weight=3)
        preview_card.rowconfigure(3, weight=1)
        preview_header = ttk.Frame(preview_card, style="Card.TFrame")
        preview_header.grid(row=0, column=0, sticky="ew")
        preview_header.columnconfigure(0, weight=1)
        ttk.Label(preview_header, text="MOTION PREVIEW & RECORDED FEEDBACK", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.profile_export_button = ttk.Button(preview_header, text="EXPORT RECORDED CSV", command=self._export_profile_run, style="Outline.TButton", state="disabled")
        self.profile_export_button.grid(row=0, column=1, sticky="e")
        channels = ttk.Frame(preview_card, style="Card.TFrame")
        channels.grid(row=1, column=0, sticky="ew", pady=(8, 2))
        channel_items = (
            (self.profile_channel_labels[0], self.profile_channel_command),
            (self.profile_channel_labels[1], self.profile_channel_estimated_motor),
            (self.profile_channel_labels[2], self.profile_channel_estimated_platform),
            (self.profile_channel_labels[3], self.profile_channel_recorded_motor),
            (self.profile_channel_labels[4], self.profile_channel_recorded_platform),
        )
        for index, (text, variable) in enumerate(channel_items):
            ttk.Checkbutton(channels, textvariable=text, variable=variable, command=self._update_profile_channels).grid(row=index // 3, column=index % 3, sticky="w", padx=(0, 16), pady=2)
        self.profile_preview = ProfilePreview(preview_card)
        self.profile_preview.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        self.profile_displacement_chart = SceneDisplacementChart(preview_card)
        self.profile_displacement_chart.grid(row=3, column=0, sticky="nsew", pady=(12, 0))

        compass_card = self._card(body, 16)
        compass_card.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        compass_card.columnconfigure(0, weight=1)
        compass_card.rowconfigure((1, 4), weight=1)
        ttk.Label(compass_card, text="PLATFORM COMPASS", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.motion_compass = MotionCompass(compass_card, height=235)
        self.motion_compass.grid(row=1, column=0, sticky="nsew", pady=(5, 8))
        ttk.Separator(compass_card).grid(row=2, column=0, sticky="ew", pady=8)
        ttk.Label(compass_card, text="SPEED", style="Section.TLabel").grid(row=3, column=0, sticky="w")
        self.profile_speed_gauge = SpeedGauge(compass_card, minimum=-self.applied_speed_limit_rpm, maximum=self.applied_speed_limit_rpm, height=215, compact=True)
        self.profile_speed_gauge.grid(row=4, column=0, sticky="nsew", pady=(4, 0))

        checks = self._card(body, 14)
        checks.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        checks.columnconfigure(6, weight=1)
        ttk.Label(checks, text="RUN READINESS", style="Section.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 16))
        self.preflight_labels = {}
        for column, label in enumerate(("Drive ready and Servo On", "EMGS hardware input healthy", "No active alarm", "Encoder monitor available", "Zero command and zero speed", "Telemetry recording ready"), start=1):
            widget = ttk.Label(checks, text=f"○ {label}", style="Muted.TLabel")
            widget.grid(row=0, column=column, sticky="w", padx=7)
            self.preflight_labels[label] = widget
        self.test_trend = None

    def _import_motion_profile(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root,
            title="Import motion profile",
            filetypes=(("CSV motion profiles", "*.csv"), ("All files", "*.*")),
        )
        if path:
            self._load_motion_profile_path(path)

    def _load_example_profile(self) -> None:
        example = Path(__file__).resolve().parent.parent / "examples" / "speed_profile_example.csv"
        self._load_motion_profile_path(str(example))

    def _load_motion_profile_path(self, path: str) -> None:
        try:
            candidate = load_motion_profile(path, float(self.platform_ratio.get()))
        except (OSError, ValueError) as exc:
            self.profile_validation_text.set(f"IMPORT FAILED • {exc}")
            messagebox.showerror("Invalid motion profile", str(exc), parent=self.root)
            return
        self.motion_profile = candidate
        self._reset_profile_recording()
        self.profile_cursor_s = None
        self.profile_export_button.configure(state="disabled")
        self.compiled_profile = None
        self.written_profile = None
        self.compiled_pr_program = None
        self.pr_direction_control = None
        self.profile_position_target_history = []
        self.profile_preview.set_position_targets(())
        self.profile_path.set(candidate.source.name)
        path_speed_format = candidate.position_path_speeds_rpm is not None
        self.profile_type_text.set(
            "SPEED SEQUENCE"
            if candidate.kind == "speed"
            else "SPEED-CONTROLLED POSITION SEQUENCE"
        )
        channel_names = (
            (
                "Motor Speed Command", "Estimated Motor Speed Profile", "Estimated Platform Speed Profile",
                "Recorded Motor Speed Feedback", "Recorded Platform Speed Feedback",
            )
            if candidate.kind == "speed"
            else (
                "Active Platform Target", "Nominal Motor Trajectory", "Nominal Platform Trajectory",
                "Recorded Motor Position Feedback", "Recorded Platform Position Feedback",
            )
        )
        for variable, name in zip(self.profile_channel_labels, channel_names):
            variable.set(name)
        value_name = (
            "P1.009 motor-speed command"
            if candidate.kind == "speed"
            else (
                "platform targets + motor speed limits + waypoint holds"
                if path_speed_format
                else "continuous total platform angle"
            )
        )
        self.profile_detail_text.set(
            f"Detected from CSV headers • {value_name} • {candidate.ratio:g}:1 motor-to-platform ratio"
        )
        self.profile_duration_text.set(
            f"~{candidate.duration_s:g} s" if path_speed_format else f"{candidate.duration_s:g} s"
        )
        self.profile_points_text.set(str(candidate.pr_path_count if path_speed_format else len(candidate.points)))
        self.profile_speed_text.set(f"{candidate.maximum_motor_speed_rpm:.1f} rpm")
        self.profile_ramp_text.set(f"{self.acceleration_s.get()} / {self.deceleration_s.get()} s")
        self.profile_stats_summary.set(
            f"• {candidate.source.name}   • ~{candidate.duration_s:g} s   • {candidate.pr_path_count} waypoints   "
            f"• peak ±{candidate.maximum_motor_speed_rpm:.1f} motor rpm   "
            f"• accel/decel {self.acceleration_s.get()} / {self.deceleration_s.get()} s"
        )
        self.profile_preview.set_ramps(
            float(self.acceleration_s.get()),
            float(self.deceleration_s.get()),
            self.applied_speed_limit_rpm,
        )
        self.profile_preview.set_profile(candidate)
        self.profile_write_button.configure(text="STAGE / VERIFY")
        self.profile_write_button.configure(state="disabled")
        self._log("Motion profile imported", "INFO", "PROFILE", f"{candidate.kind}; {candidate.pr_path_count} paths; ~{candidate.duration_s:g} s")
        self._compile_motion_profile()

    def _compile_motion_profile(self) -> None:
        profile = self.motion_profile
        if profile is None:
            return
        try:
            if profile.maximum_motor_speed_rpm > self.applied_speed_limit_rpm + 1e-9:
                raise ValueError(
                    f"Peak motor speed {profile.maximum_motor_speed_rpm:.1f} rpm exceeds the applied ±{self.applied_speed_limit_rpm:g} rpm operator limit"
                )
            if profile.kind == "position":
                if abs(profile.points[0].value) > 1e-9:
                    raise ValueError("A position profile must begin at platform_position_deg = 0")
        except ValueError as exc:
            self.compiled_profile = None
            self.profile_validation_text.set(f"COMPILE FAILED • {exc}")
            self.profile_write_button.configure(state="disabled")
            return
        self.compiled_profile = profile
        self.written_profile = None
        self.compiled_pr_program = None
        self.pr_direction_control = None
        if profile.kind == "speed":
            details = f"{len(profile.segments)} speed intervals • final command is forced to 0 rpm"
        elif profile.position_path_speeds_rpm is not None:
            overlap_count = sum(
                position_waypoint_can_overlap(profile, index)
                for index in range(profile.pr_path_count)
            )
            details = (
                f"{profile.pr_path_count} speed-controlled position waypoints • "
                f"{overlap_count} same-direction pass-through transitions • "
                "slowdown only for holds/reversals/final target"
            )
        else:
            details = f"{len(profile.segments)} speed-controlled position intervals • feedback slowdown and stop"
        self.profile_validation_text.set(f"COMPILED • {details}")
        self.profile_workflow_text.set("✓  IMPORT   →   ✓  COMPILED   →   3  WRITE / VERIFY   →   4  START")
        self.profile_write_button.configure(state="normal" if self.controller.state is ConnectionState.CONNECTED else "disabled")
        self._update_test_start_state()
        self._log("Motion profile compiled", "INFO", "PROFILE", details)

    def _start_or_abort_motion_profile(self) -> None:
        if self.test_running or self.test_preparing or self.profile_recording_active:
            self._abort_motion_profile()
        else:
            self._start_motion_profile()

    @staticmethod
    def _nearest_home_error(platform_angle: float) -> float:
        """Return the signed shortest angular displacement from physical 0°."""
        return (platform_angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def _resolve_dashboard_position_target(
        current_deg: float,
        requested: float,
        units: str,
        direction_choice: str,
    ) -> float:
        """Resolve a user target onto the continuous, unwrapped position axis."""
        if units == "revolutions":
            travel_deg = requested * 360.0
            if direction_choice == "Positive / anticlockwise":
                travel_deg = abs(travel_deg)
            elif direction_choice == "Negative / clockwise":
                travel_deg = -abs(travel_deg)
            return current_deg + travel_deg

        target_orientation = requested % 360.0
        current_orientation = current_deg % 360.0
        if direction_choice == "Positive / anticlockwise":
            delta = (target_orientation - current_orientation) % 360.0
        elif direction_choice == "Negative / clockwise":
            delta = -((current_orientation - target_orientation) % 360.0)
        else:
            delta = (target_orientation - current_orientation + 180.0) % 360.0 - 180.0
        return current_deg + delta

    def _start_profile_home(self) -> None:
        if (
            self.controller.state is not ConnectionState.CONNECTED
            or self.test_running
            or self.test_preparing
        ):
            return
        if self.session_origin_pulses is None:
            messagebox.showerror(
                "Home unavailable",
                "The connection-time encoder zero has not been captured yet.",
                parent=self.root,
            )
            return
        if not self._automatic_preflight_ready():
            self._show_preflight_error("Home not ready")
            return
        _, platform_angle = self._session_angles()
        if platform_angle is None:
            messagebox.showerror("Home unavailable", "Platform position feedback is unavailable.", parent=self.root)
            return
        error = self._nearest_home_error(platform_angle)
        if abs(error) <= 0.5:
            messagebox.showinfo("Already home", f"Platform position is {error:+.3f}° from home.", parent=self.root)
            return
        if not self.controller.simulated and not messagebox.askyesno(
            "Jog platform to home?",
            f"The platform will take the shortest path from {platform_angle % 360.0:.2f}° to the connection-time 0° orientation. "
            "Motor jog speed is limited to 5 rpm.\n\n"
            "Confirm the SON selector is ON, the travel path is clear, and Abort is available.",
            parent=self.root,
        ):
            return
        self.profile_manual_home = True
        self.test_preparing = True
        self.profile_homing_started_at = monotonic()
        self.profile_home_previous_error = abs(error)
        self.profile_home_error_growth_count = 0
        self.test_status.set(f"HOMING • {error:+.1f}°")
        self.dashboard_position_command_status.set(f"Returning home • error {error:+.1f}°")
        self.profile_validation_text.set("HOMING TO SESSION 0° • shortest angular path • 5 rpm cap")
        self._update_test_start_state()
        self._log("Manual home started", "WARN", "PROFILE", f"Platform error {error:+.3f}°")
        if self.controller.simulated:
            self._run_manual_home_tick()
            return

        def worker() -> None:
            try:
                self.controller.arm_live_motion(
                    min(self.applied_speed_limit_rpm, 5.0), (True, True, True, True)
                )
                self.events.put(("profile_home_armed", None))
            except Exception as exc:
                self.events.put(("profile_home_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="profile-home-arm").start()

    def _start_dashboard_position_command(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED:
            messagebox.showerror("Position unavailable", "Connect to the servo drive first.", parent=self.root)
            return
        if self.test_running or self.test_preparing or self.dashboard_position_active:
            messagebox.showerror("Motion busy", "Stop the active test or position movement first.", parent=self.root)
            return
        if self.session_origin_pulses is None or not self.telemetry.samples:
            messagebox.showerror("Position unavailable", "Encoder position feedback is not available yet.", parent=self.root)
            return
        if not self._automatic_preflight_ready():
            self._show_preflight_error("Position not ready")
            return
        _, platform_angle = self._session_angles()
        if platform_angle is None:
            messagebox.showerror("Position unavailable", "Platform position feedback is unavailable.", parent=self.root)
            return
        current_deg = platform_angle * self._platform_command_sign()
        try:
            requested = float(self.dashboard_position_target.get())
            direction_choice = self.dashboard_position_direction_choice.get()
            target_deg = self._resolve_dashboard_position_target(
                current_deg,
                requested,
                self.dashboard_position_units.get(),
                direction_choice,
            )
            speed_rpm = float(self.dashboard_position_speed.get())
            if not 0 < speed_rpm <= self.applied_speed_limit_rpm:
                raise ValueError(f"Target speed must be greater than 0 and no more than {self.applied_speed_limit_rpm:g} rpm")
        except ValueError as exc:
            messagebox.showerror("Invalid position command", str(exc), parent=self.root)
            return
        if self.dashboard_position_units.get() == "degrees":
            self.dashboard_position_goal_label = (
                f"{requested % 360.0:.2f}° orientation • travel {target_deg - current_deg:+.2f}°"
            )
        else:
            self.dashboard_position_goal_label = (
                f"travel {target_deg - current_deg:+.2f}° ({(target_deg - current_deg) / 360.0:+.3f} rev)"
            )
        if not self.controller.simulated and not self.controller.hardware_motion_enabled:
            messagebox.showerror(
                "Live control disabled",
                "Enable Live Control on the Dashboard before commanding a position.",
                parent=self.root,
            )
            return
        if abs(target_deg - current_deg) <= 0.5:
            self.dashboard_position_command_status.set(f"Already at target • {current_deg:+.2f}°")
            return
        if not self.controller.simulated and not messagebox.askyesno(
            "Run speed-mode position movement?",
            f"Move from {current_deg:+.2f}° to {self.dashboard_position_goal_label} at up to {speed_rpm:g} motor rpm?\n\n"
            "Confirm the travel path is clear and the physical E-stop is available.",
            parent=self.root,
        ):
            return
        self.dashboard_position_goal_deg = target_deg
        self.dashboard_position_direction = 1 if target_deg > current_deg else -1
        self.dashboard_position_started_at = monotonic()
        nominal_s = abs(target_deg - current_deg) * float(self.platform_ratio.get()) / 360.0 * 60.0 / speed_rpm
        self.dashboard_position_timeout_s = max(60.0, nominal_s * 4.0 + 20.0)
        self.dashboard_position_active = True
        self.dashboard_position_braking = False
        self.dashboard_position_creep = False
        self.dashboard_position_deceleration_rpm_s = None
        self.dashboard_position_command_status.set(f"Preparing • {self.dashboard_position_goal_label}")
        self.dashboard_position_go.configure(text="MOVING…", state="disabled")
        # Manual moves share the session timeline; retain existing measurements.
        self._log(
            "Dashboard position started", "INFO", "MOTION",
            f"{current_deg:+.2f} deg -> {target_deg:+.2f} deg; "
            f"maximum {speed_rpm:g} motor rpm; "
            f"monotonic start {self.dashboard_position_started_at:.6f} s",
        )

        def worker() -> None:
            try:
                self.controller.select_speed_mode()
                _acceleration_rpm_s, deceleration_rpm_s = self.controller.read_speed_ramp_rates()
                self.events.put(("dashboard_position_ready", (speed_rpm, deceleration_rpm_s)))
            except Exception as exc:
                self.events.put(("dashboard_position_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="dashboard-position-start").start()

    def _run_dashboard_position_tick(self, speed_rpm: float) -> None:
        if not self.dashboard_position_active or self.dashboard_position_goal_deg is None:
            return
        if not self.telemetry.samples or self.dashboard_position_direction is None:
            self.dashboard_position_after_id = self.root.after(50, lambda: self._run_dashboard_position_tick(speed_rpm))
            return
        sample = self.telemetry.samples[-1]
        if sample.motor_position_deg is None or sample.encoder_feedback_rpm is None:
            self._stop_dashboard_position_command("Encoder feedback lost")
            return
        _, raw_platform = self._session_angles()
        if raw_platform is None:
            self._stop_dashboard_position_command("Platform position feedback lost")
            return
        current_deg = raw_platform * self._platform_command_sign()
        if self.dashboard_position_started_at is not None and monotonic() - self.dashboard_position_started_at > self.dashboard_position_timeout_s:
            self._stop_dashboard_position_command("Position movement timed out")
            return
        ratio = float(self.platform_ratio.get())
        directed_error = (
            self.dashboard_position_goal_deg - current_deg
        ) * self.dashboard_position_direction
        overshoot = max(0.0, -directed_error)
        if overshoot > 5.0:
            self._stop_dashboard_position_command(f"Overshoot {overshoot:.2f}°")
            return

        signed_platform_speed_deg_s = (
            sample.encoder_feedback_rpm / ratio * 6.0 * self._platform_command_sign()
        )
        speed_toward_target_deg_s = max(
            0.0, signed_platform_speed_deg_s * self.dashboard_position_direction
        )

        if self.dashboard_position_braking:
            try:
                self.controller.update_speed_command_rpm(0.0)
            except Exception as exc:
                self._stop_dashboard_position_command(f"Braking command failed: {exc}")
                return
            self.dashboard_position_command_status.set(
                f"Braking • remaining {directed_error:.2f}° • speed {speed_toward_target_deg_s / 6.0:.1f} platform rpm"
            )
            if abs(sample.encoder_feedback_rpm) <= 0.5:
                if directed_error <= 0.5:
                    self._stop_dashboard_position_command(
                        f"Target reached • orientation {current_deg % 360.0:.2f}° • continuous position {current_deg:+.2f}°",
                        completed=True,
                    )
                    return
                # The safety margin may intentionally stop short. Finish at a
                # low speed where the drive ramp contributes negligible travel.
                self.dashboard_position_braking = False
                self.dashboard_position_creep = True
            self.dashboard_position_after_id = self.root.after(50, lambda: self._run_dashboard_position_tick(speed_rpm))
            return

        if self.dashboard_position_creep:
            motor_rpm, reached, overshoot = position_speed_command(
                current_deg,
                self.dashboard_position_goal_deg,
                min(speed_rpm, 10.0),
                ratio,
                self.dashboard_position_direction,
            )
            if overshoot > 5.0:
                self._stop_dashboard_position_command(f"Final approach overshoot {overshoot:.2f}°")
                return
            if reached:
                self.dashboard_position_braking = True
                motor_rpm = 0.0
        else:
            deceleration_rpm_s = self.dashboard_position_deceleration_rpm_s
            if deceleration_rpm_s is None or deceleration_rpm_s <= 0:
                self._stop_dashboard_position_command("Drive deceleration rate is unavailable")
                return
            platform_deceleration_deg_s2 = deceleration_rpm_s / ratio * 6.0
            braking_distance_deg = (
                1.15 * speed_toward_target_deg_s**2 / (2.0 * platform_deceleration_deg_s2)
                + speed_toward_target_deg_s * 0.15
                + 2.0
            )
            if directed_error <= braking_distance_deg and speed_toward_target_deg_s > 1.0:
                self.dashboard_position_braking = True
                motor_rpm = 0.0
            else:
                motor_rpm = self.dashboard_position_direction * speed_rpm
        try:
            self.controller.update_speed_command_rpm(motor_rpm * self._platform_command_sign())
        except Exception as exc:
            self._stop_dashboard_position_command(f"Command failed: {exc}")
            return
        self.dashboard_position_command_status.set(
            f"{'Final approach' if self.dashboard_position_creep else 'Moving'} • "
            f"{self.dashboard_position_goal_label} • remaining {directed_error:.2f}° • {motor_rpm:+.1f} rpm"
        )
        self.dashboard_position_after_id = self.root.after(50, lambda: self._run_dashboard_position_tick(speed_rpm))

    def _stop_dashboard_position_command(self, reason: str = "Operator stop", *, completed: bool = False) -> None:
        was_active = self.dashboard_position_active
        self.dashboard_position_active = False
        if self.dashboard_position_after_id is not None:
            self.root.after_cancel(self.dashboard_position_after_id)
            self.dashboard_position_after_id = None
        self.dashboard_position_go.configure(text="GO TO POSITION", state="normal")
        self.dashboard_position_command_status.set(reason)
        if self.controller.state is ConnectionState.CONNECTED and was_active:
            self._run_command(
                "Dashboard position stop",
                lambda: self.controller.update_speed_command_rpm(0.0, force=True),
            )
        self._log(
            "Dashboard position complete" if completed else "Dashboard position stopped",
            "INFO" if completed else "WARN",
            "MOTION",
            reason,
        )

    def _run_manual_home_tick(self) -> None:
        if not self.profile_manual_home or not self.test_preparing:
            return
        if self.profile_homing_started_at is None or monotonic() - self.profile_homing_started_at > 60.0:
            self._abort_motion_profile("Home jog timed out")
            return
        _, platform_angle = self._session_angles()
        feedback = None if self.last_snapshot is None else self.last_snapshot.feedback_speed_rpm
        if platform_angle is None or feedback is None:
            self.profile_run_after_id = self.root.after(100, self._run_manual_home_tick)
            return
        error = self._nearest_home_error(platform_angle)
        if abs(error) <= 0.5 and abs(feedback) <= 0.5:
            def stop_worker() -> None:
                try:
                    self.controller.controlled_stop()
                    # A successful home is a completed dashboard move. Keep the
                    # current live permission so the next command remains usable;
                    # never re-arm permission revoked by a fault or the operator.
                    self.events.put(("profile_home_complete", error))
                except Exception as exc:
                    self.events.put(("profile_home_error", str(exc)))

            threading.Thread(target=stop_worker, daemon=True, name="profile-home-stop").start()
            return

        previous_error = self.profile_home_previous_error
        if previous_error is not None and abs(error) > previous_error + 1.0:
            self.profile_home_error_growth_count += 1
        else:
            self.profile_home_error_growth_count = 0
        self.profile_home_previous_error = abs(error)
        if self.profile_home_error_growth_count >= 3:
            self._abort_motion_profile("Home error increased; check the configured direction")
            return

        motor_speed = max(0.5, min(5.0, abs(error) * float(self.platform_ratio.get()) * 0.2))
        motor_speed = -motor_speed if error > 0 else motor_speed
        self.test_status.set(f"HOMING • {error:+.1f}°")

        def step_worker() -> None:
            try:
                self.controller.command_speed_rpm(motor_speed)
                self.events.put(("profile_home_step", None))
            except Exception as exc:
                self.events.put(("profile_home_error", str(exc)))

        threading.Thread(target=step_worker, daemon=True, name="profile-home-step").start()

    def _update_profile_channels(self) -> None:
        self.profile_preview.set_channels(
            command=self.profile_channel_command.get(),
            estimated_motor=self.profile_channel_estimated_motor.get(),
            estimated_platform=self.profile_channel_estimated_platform.get(),
            recorded_motor=self.profile_channel_recorded_motor.get(),
            recorded_platform=self.profile_channel_recorded_platform.get(),
        )

    def _cancel_profile_safety_stop_timer(self) -> None:
        timer = self.profile_safety_stop_timer
        self.profile_safety_stop_timer = None
        if timer is not None:
            timer.cancel()

    def _cancel_speed_profile_execution(self) -> None:
        self.profile_execution_cancel.set()
        self.profile_execution_thread = None

    def _reset_profile_recording(self) -> None:
        self._cancel_speed_profile_execution()
        self._cancel_profile_safety_stop_timer()
        self.profile_safety_stop_complete.clear()
        self.profile_safety_stop_monotonic_s = None
        self.profile_recording_active = False
        self.profile_recorded_samples = []
        self.profile_clock_start_ns = None
        self.profile_clock_end_ns = None
        self.profile_run_clock = None
        self.profile_local_events = []
        self.profile_remote_events = []
        self.profile_remote_metadata = {}
        self.profile_first_motion_command_recorded = False
        if hasattr(self, "profile_preview"):
            self.profile_preview.set_recorded_samples(())
        if hasattr(self, "profile_displacement_chart"):
            duration = 0.0 if self.motion_profile is None else self.motion_profile.duration_s
            self.profile_displacement_chart.set_samples((), duration)

    def _start_profile_recording(self) -> None:
        self.telemetry.clear()
        self.labjack.clear()
        self.profile_clock_start_ns = perf_counter_ns()
        self.profile_clock_end_ns = None
        profile = self.written_profile or self.compiled_profile
        selected_sync = self.clock_sync.best_recent_sample() if self.clock_sync.running else None
        if self.profile_run_clock is None:
            self.profile_run_clock = (
                None
                if profile is None
                else TestRunClock.create(profile.source, self.profile_clock_start_ns, selected_sync)
            )
        if self.clock_sync.running:
            latest = selected_sync
            if latest is None:
                self._log(
                    "Clock sync has no replies at profile start",
                    "ERROR",
                    "SYNC",
                    "Client is running, but no valid exchange has been received",
                )
            else:
                age_s = (
                    self.profile_clock_start_ns - latest.local_receive_monotonic_ns
                ) / 1_000_000_000.0
                if age_s > 2.0:
                    self._log(
                        "Clock sync reply is stale at profile start",
                        "ERROR",
                        "SYNC",
                        f"Latest valid reply is {age_s:.1f} s old",
                    )
        self.profile_recorded_samples = []
        self.profile_recording_active = True
        self.profile_preview.set_recorded_samples(())
        duration = 0.0 if self.written_profile is None else self.written_profile.duration_s
        self.profile_displacement_chart.set_samples((), duration)

    def _freeze_profile_recording(self) -> None:
        self.profile_recording_active = False
        self.profile_clock_end_ns = perf_counter_ns()
        self.profile_preview.set_recorded_samples(self.profile_recorded_samples)
        duration = 0.0 if self.written_profile is None else self.written_profile.duration_s
        self.profile_displacement_chart.set_samples(self.profile_recorded_samples, duration)

    def _export_profile_run(self, automatic_path: str | Path | None = None) -> Path | None:
        # This function is the authoritative motor CSV schema. Speed angles retain
        # the continuous encoder origin; position angles subtract the run origin.
        # Keep downstream REQUIRED_MOTION_COLUMNS and export tests in sync.
        profile = self.written_profile or self.compiled_profile
        if profile is None or not self.profile_recorded_samples:
            messagebox.showinfo("Nothing to export", "Run a motion profile before exporting recorded data.", parent=self.root)
            return
        path = str(automatic_path) if automatic_path is not None else filedialog.asksaveasfilename(
                parent=self.root,
                title="Export motion profile run",
                defaultextension=".csv",
                filetypes=(("CSV files", "*.csv"),),
                initialfile=f"{profile.source.stem}_recorded.csv",
            )
        if not path:
            return None
        run_clock = getattr(self, "profile_run_clock", None)

        def run_clock_values(sample: TelemetrySample) -> tuple[str, str, str, str, str, str, str]:
            motor_pc_ns = round(sample.sampled_monotonic_s * 1_000_000_000.0)
            if run_clock is None or not run_clock.synchronized:
                return ("", "", str(motor_pc_ns), "", "", "", "unsynchronized")
            lattepanda_ns = run_clock.motor_pc_to_lattepanda_ns(motor_pc_ns)
            relative_s = run_clock.latte_panda_time_s(motor_pc_ns)
            return (
                run_clock.run_id,
                "" if relative_s is None else f"{relative_s:.9f}",
                str(motor_pc_ns),
                "" if lattepanda_ns is None else str(lattepanda_ns),
                f"{run_clock.remote_minus_local_ns:.3f}",
                str(run_clock.clock_round_trip_ns),
                "synchronized",
            )
        try:
            if profile.kind == "position":
                origin_motor = (
                    self.profile_run_origin_motor_deg
                    if self.profile_run_origin_motor_deg is not None
                    else (0.0 if self.session_origin_pulses is None else self.session_origin_pulses * 360.0 / (1 << 24))
                )
                with open(path, "w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow((
                        "elapsed_s", "run_id", "lattepanda_time_s", "motor_pc_monotonic_ns",
                        "lattepanda_monotonic_ns", "clock_offset_ns", "clock_round_trip_ns",
                        "clock_sync_status", "active_platform_target_deg", "nominal_motor_position_deg",
                        "nominal_platform_position_deg", "recorded_motor_position_deg",
                        "recorded_platform_position_deg", "sampled_monotonic_s", "acquisition_duration_ms",
                        "scene_displacement_ain0_v", "scene_vertical_displacement_mm",
                    ))
                    for sample in self.profile_recorded_samples:
                        active_target = (
                            self.profile_position_target_history[0][1]
                            if self.profile_position_target_history
                            else profile.points[0].value
                        )
                        for target_time, target_value in self.profile_position_target_history[1:]:
                            if sample.elapsed_s < target_time:
                                break
                            active_target = target_value
                        nominal_platform = interpolate_profile(profile, sample.elapsed_s)
                        recorded_motor = None if sample.motor_position_deg is None else sample.motor_position_deg - origin_motor
                        writer.writerow((
                            f"{sample.elapsed_s:.6f}", *run_clock_values(sample), f"{active_target:.6f}",
                            f"{nominal_platform * profile.ratio:.6f}", f"{nominal_platform:.6f}",
                            "" if recorded_motor is None else f"{recorded_motor:.6f}",
                            "" if recorded_motor is None else f"{recorded_motor / profile.ratio:.6f}",
                            f"{sample.sampled_monotonic_s:.6f}",
                            "" if sample.acquisition_duration_s is None else f"{sample.acquisition_duration_s * 1000.0:.3f}",
                            "" if sample.drawstring_voltage_v is None else f"{sample.drawstring_voltage_v:.6f}",
                            "" if sample.drawstring_extension_mm is None else f"{sample.drawstring_extension_mm:.6f}",
                        ))
            else:
                with open(path, "w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow((
                        "elapsed_s", "run_id", "lattepanda_time_s", "motor_pc_monotonic_ns",
                        "lattepanda_monotonic_ns", "clock_offset_ns", "clock_round_trip_ns",
                        "clock_sync_status", "motor_speed_command_rpm", "recorded_motor_feedback_rpm",
                        "recorded_platform_feedback_rpm", "encoder_position_pulses",
                        "platform_angle_deg", "platform_wrapped_angle_deg",
                        "sampled_monotonic_s", "acquisition_duration_ms",
                        "scene_displacement_ain0_v", "scene_vertical_displacement_mm",
                    ))
                    for sample in self.profile_recorded_samples:
                        writer.writerow((
                            f"{sample.elapsed_s:.6f}", *run_clock_values(sample), f"{sample.requested_rpm:.3f}",
                            "" if sample.encoder_feedback_rpm is None else f"{sample.encoder_feedback_rpm:.3f}",
                            "" if sample.platform_speed_rpm is None else f"{sample.platform_speed_rpm:.3f}",
                            "" if sample.encoder_position_pulses is None else str(sample.encoder_position_pulses),
                            "" if sample.platform_position_deg is None else f"{sample.platform_position_deg:.6f}",
                            "" if sample.platform_position_deg is None else f"{sample.platform_position_deg % 360.0:.6f}",
                            f"{sample.sampled_monotonic_s:.6f}",
                            "" if sample.acquisition_duration_s is None else f"{sample.acquisition_duration_s * 1000.0:.3f}",
                            "" if sample.drawstring_voltage_v is None else f"{sample.drawstring_voltage_v:.6f}",
                            "" if sample.drawstring_extension_mm is None else f"{sample.drawstring_extension_mm:.6f}",
                        ))
        except OSError as exc:
            if automatic_path is None:
                messagebox.showerror("Export failed", str(exc), parent=self.root)
            else:
                self._log("Automatic motion export failed", "ERROR", "DATA", str(exc))
            return None
        sync_samples = self.clock_sync.samples()
        if sync_samples:
            sync_path = Path(path).with_name(f"{Path(path).stem}_clock_sync.csv")
            # Retain ten seconds before and after the actual motion-recording
            # window. Do not include minutes of unrelated post-run exchanges,
            # and never erase the useful pre-run clock history.
            if self.profile_clock_start_ns is not None:
                window_start_ns = self.profile_clock_start_ns - 10_000_000_000
                window_end_ns = (
                    (self.profile_clock_end_ns or perf_counter_ns())
                    + 10_000_000_000
                )
                sync_samples = [
                    sample
                    for sample in sync_samples
                    if window_start_ns
                    <= sample.local_midpoint_monotonic_ns
                    <= window_end_ns
                ]
            if not sync_samples:
                messagebox.showwarning(
                    "Clock alignment unavailable",
                    "The clock-sync client was running, but no valid replies fall "
                    "within the recording window. The motion CSV will still be "
                    "saved; check that the responder status shows fresh replies "
                    "before the next run.",
                    parent=self.root,
                )
                self._log(
                    "Clock alignment window is empty",
                    "ERROR",
                    "SYNC",
                    "No valid exchange from 10 s before recording through 10 s after",
                )
                sync_samples = []
            try:
                self.clock_sync.export_csv(sync_path, sync_samples)
                self._log("Clock alignment exported", "INFO", "SYNC", str(sync_path))
            except OSError as exc:
                messagebox.showwarning(
                    "Clock export failed",
                    f"The motion CSV was saved, but the clock-alignment sidecar failed:\n{exc}",
                    parent=self.root,
                )
        elif self.clock_sync.running:
            messagebox.showwarning(
                "Clock alignment unavailable",
                "Clock sync is running, but it has not received any valid replies. "
                "The motion CSV was saved without a clock-sync sidecar.",
                parent=self.root,
            )
            self._log(
                "Clock alignment export unavailable",
                "ERROR",
                "SYNC",
                "Client thread is running with zero valid exchanges",
            )
        self._log("Motion profile run exported", "INFO", "PROFILE", path)
        try:
            ServoControllerApp._export_profile_event_feed(self, Path(path))
        except OSError as exc:
            self._log("Profile event/metadata export failed", "ERROR", "DATA", str(exc))
            if automatic_path is None:
                messagebox.showwarning(
                    "Event export failed",
                    f"The motion CSV was saved, but its event or metadata sidecar failed:\n{exc}",
                    parent=self.root,
                )
        return Path(path)

    def _automatic_profile_export_path(self) -> Path:
        profile = self.written_profile or self.compiled_profile
        if profile is None:
            raise RuntimeError("No profile is available for automatic export")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        candidate = profile.source.parent / f"{profile.source.stem}_{stamp}_recorded.csv"
        suffix = 1
        while candidate.exists():
            candidate = profile.source.parent / f"{profile.source.stem}_{stamp}_{suffix}_recorded.csv"
            suffix += 1
        return candidate

    def _auto_export_profile_run(self) -> None:
        if not self.profile_recorded_samples:
            self._log("Automatic profile export skipped", "WARN", "DATA", "No recorded samples")
            return
        path = self._automatic_profile_export_path()
        exported = self._export_profile_run(path)
        if exported is not None:
            self.profile_validation_text.set(f"{self.profile_validation_text.get()} • EXPORTED {exported.name}")

    def _record_profile_event(
        self, event: str, motor_pc_ns: int | None = None, details: str = "",
    ) -> None:
        local_ns = perf_counter_ns() if motor_pc_ns is None else motor_pc_ns
        run_clock = self.profile_run_clock
        latte_ns = None if run_clock is None else run_clock.motor_pc_to_lattepanda_ns(local_ns)
        relative_s = None if run_clock is None else run_clock.latte_panda_time_s(local_ns)
        self.profile_local_events.append({
            "source": "motor_pc",
            "event": event,
            "motor_pc_monotonic_ns": local_ns,
            "lattepanda_monotonic_ns": latte_ns,
            "lattepanda_time_s": relative_s,
            "details": details,
        })

    def _export_profile_event_feed(self, motion_path: Path) -> None:
        # Events and metadata are sidecars, not an atomic multi-file commit. A saved
        # motion CSV alone does not prove remote finalization or sidecar success.
        event_path = motion_path.with_name(f"{motion_path.stem}_events.csv")
        with event_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow((
                "source", "event", "motor_pc_monotonic_ns",
                "lattepanda_monotonic_ns", "lattepanda_time_s", "sequence",
                "scheduled_lattepanda_monotonic_ns", "schedule_error_us", "details",
            ))
            local_events = getattr(self, "profile_local_events", [])
            remote_events = getattr(self, "profile_remote_events", [])
            combined_events = [*local_events, *remote_events]
            combined_events.sort(
                key=lambda item: (
                    item.get("lattepanda_monotonic_ns") is None,
                    int(item.get("lattepanda_monotonic_ns") or 0),
                )
            )
            for event in combined_events:
                latte_ns = event.get("lattepanda_monotonic_ns")
                relative = event.get("lattepanda_time_s")
                if relative is None and latte_ns is not None and self.profile_run_clock is not None:
                    relative = (
                        int(latte_ns) - self.profile_run_clock.lattepanda_t0_monotonic_ns
                    ) / 1_000_000_000.0
                scheduled_ns = event.get("scheduled_lattepanda_monotonic_ns")
                schedule_error_us = (
                    None
                    if latte_ns is None or scheduled_ns is None
                    else (int(latte_ns) - int(scheduled_ns)) / 1000.0
                )
                writer.writerow((
                    event.get("source", "lattepanda"),
                    event.get("event", ""),
                    event.get("motor_pc_monotonic_ns", ""),
                    "" if latte_ns is None else latte_ns,
                    "" if relative is None else f"{float(relative):.9f}",
                    event.get("sequence", ""),
                    "" if scheduled_ns is None else scheduled_ns,
                    "" if schedule_error_us is None else f"{schedule_error_us:.3f}",
                    event.get("details", ""),
                ))
        self._log("Profile event feed exported", "INFO", "DATA", str(event_path))
        run_clock = getattr(self, "profile_run_clock", None)
        if run_clock is not None:
            metadata_path = motion_path.with_name(f"{motion_path.stem}_metadata.json")
            remote_metadata = getattr(self, "profile_remote_metadata", {})
            metadata_path.write_text(
                json.dumps({
                    "format_version": 1,
                    "motor_app_build_id": ServoControllerApp.APP_BUILD_ID,
                    "run_id": run_clock.run_id,
                    "source_profile": str(run_clock.source_profile_path),
                    "motor_pc_t0_monotonic_ns": run_clock.motor_pc_t0_monotonic_ns,
                    "lattepanda_t0_monotonic_ns": run_clock.lattepanda_t0_monotonic_ns,
                    "clock_offset_ns": run_clock.remote_minus_local_ns,
                    "clock_round_trip_ns": run_clock.clock_round_trip_ns,
                    "clock_sync_sequence": run_clock.clock_sync_sequence,
                    "coordinator_state": remote_metadata.get("state"),
                    "coordinator_finalization_error": remote_metadata.get("finalization_error", ""),
                    "camera_healthy_at_finish": remote_metadata.get("camera_healthy"),
                    "serial_trigger_healthy_at_finish": remote_metadata.get("serial_trigger_healthy"),
                    "camera_files_on_lattepanda": remote_metadata.get("camera_files", []),
                    "motion_csv": motion_path.name,
                    "event_csv": event_path.name,
                }, indent=2) + "\n",
                encoding="utf-8",
            )
            self._log("Profile metadata exported", "INFO", "DATA", str(metadata_path))

    def _write_motion_profile(self) -> None:
        profile = self.compiled_profile
        if profile is None or self.controller.state is not ConnectionState.CONNECTED:
            return
        if self.controller.simulated:
            self.written_profile = profile
            self.profile_validation_text.set("WRITTEN + VERIFIED • simulator execution plan matches the compiled profile")
            self.profile_workflow_text.set("✓  IMPORT   →   ✓  COMPILED   →   ✓  WRITTEN / VERIFIED   →   4  START")
            self._update_test_start_state()
            self._log("Motion profile written and verified", "INFO", "PROFILE", "Simulation execution plan")
            return
        if profile.kind == "speed":
            try:
                validate_live_speed_profile(profile)
            except ValueError as exc:
                messagebox.showerror(
                    "Unsafe live Speed CSV",
                    f"{exc}. The first command may be non-zero.",
                    parent=self.root,
                )
                return
            self.written_profile = profile
            self.compiled_pr_program = None
            self.pr_direction_control = None
            staging_description = "live Speed-mode timeline"
        else:
            if profile.position_path_targets_deg is None:
                messagebox.showerror(
                    "Position CSV requires speed limits",
                    "Physical speed-controlled positioning requires the recommended "
                    "platform_position_deg,motor_speed_rpm,hold_s format.",
                    parent=self.root,
                )
                return
            staging_description = (
                "speed-controlled position waypoints • encoder slowdown/stop • "
                "no PR registers written"
            )
        if profile.kind in ("speed", "position"):
            self.written_profile = profile
            self.compiled_pr_program = None
            self.pr_direction_control = None
            self.profile_validation_text.set(f"STAGED + VERIFIED • {staging_description}")
            self.profile_workflow_text.set(
                "✓  IMPORT   →   ✓  COMPILED   →   ✓  STAGED / VERIFIED   →   4  START"
            )
            self._update_test_start_state()
            self._log(
                "Live speed-mode CSV staged",
                "INFO",
                "PROFILE",
                f"{len(profile.points)} rows; ~{profile.duration_s:g} s; peak ±{profile.maximum_motor_speed_rpm:g} rpm",
            )
            return
        if not messagebox.askyesno(
            "Write verified PR program?",
            "Turn the physical SON selector OFF and confirm the shaft is stopped.\n\n"
            "The app will read P1.044/P1.045, compile incremental PR paths, back up every affected P5/P6/P7 register, write the program, and verify every readback. This operation does not trigger motion.\n\n"
            "Continue?",
            parent=self.root,
        ):
            return
        self.profile_write_button.configure(state="disabled")
        self.profile_validation_text.set("WRITING • reading E-gear and staging verified PR registers")
        try:
            acceleration_s = float(self.acceleration_s.get())
            deceleration_s = float(self.deceleration_s.get())
            operator_limit = float(self.applied_speed_limit_rpm)
            command_sign = self._platform_command_sign()
        except (TypeError, ValueError) as exc:
            self.profile_validation_text.set(f"WRITE FAILED • invalid ramp or speed setting: {exc}")
            self._update_test_start_state()
            return

        def worker() -> None:
            try:
                numerator, denominator, direction_control = self.controller.read_pr_setup()
                acceleration_ms, deceleration_ms = self.controller.ramp_parameter_values(
                    acceleration_s,
                    deceleration_s,
                    operator_limit,
                )
                program = compile_position_pr_program(
                    profile,
                    egear_numerator=numerator,
                    egear_denominator=denominator,
                    acceleration_ms=acceleration_ms,
                    deceleration_ms=deceleration_ms,
                    maximum_motor_speed_rpm=operator_limit,
                    command_sign=command_sign,
                )
                writes = self.controller.write_and_verify_pr_program(program)
                self.events.put(("profile_pr_written", (program, direction_control, len(writes))))
            except Exception as exc:
                self.events.put(("profile_pr_write_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="profile-pr-write").start()

    def _start_motion_profile(self) -> None:
        profile = self.written_profile
        if profile is None or self.test_running or self.test_preparing or self.profile_recording_active:
            return
        if not self._automatic_preflight_ready():
            messagebox.showerror("Profile not ready", "Every automatic run-readiness check must pass.", parent=self.root)
            return
        self.profile_run_clock = None
        self.profile_run_coordinator = None
        self.profile_local_events = []
        self.profile_remote_events = []
        self.profile_remote_metadata = {}
        self.profile_first_motion_command_recorded = False
        if not self.controller.simulated:
            if profile.kind in ("speed", "position"):
                position_mode = profile.kind == "position"
                motion_description = (
                    f"This run uses Speed mode to approach {profile.pr_path_count} platform-position waypoints. "
                    "It slows from encoder position error, stops without reverse correction after crossing a target, "
                    "and aborts excessive overshoot. No physical platform encoder is present."
                    if position_mode
                    else (
                        f"This timeline runs for {profile.duration_s:g} s with a peak command of "
                        f"±{profile.maximum_motor_speed_rpm:g} motor rpm."
                    )
                )
                if not messagebox.askyesno(
                    "Start physical speed-mode CSV motion?",
                    motion_description + " Commands are sent through P1.009 in Speed mode; no PR paths are used.\n\n"
                    "Confirm the SON selector is ON, EMGS/STO have been tested, the shaft has clearance, "
                    "and the applied speed/ramp settings are safe.\n\nStart the Speed CSV now?",
                    parent=self.root,
                ):
                    return
                self.test_preparing = True
                self.test_status.set("ARMING SPEED-CONTROLLED POSITION" if position_mode else "ARMING LIVE SPEED CSV")
                self.profile_validation_text.set("ARMING • final live Speed-mode checks")
                self._update_test_start_state()

                def speed_worker() -> None:
                    try:
                        self.controller.arm_live_motion(
                            profile.maximum_motor_speed_rpm,
                            (True, True, True, True),
                        )
                        self.controller.select_speed_mode()
                        self.controller.command_speed_rpm(0.0)
                        self.events.put(("profile_speed_armed", self._arm_lattepanda_profile(profile)))
                    except Exception as exc:
                        try:
                            self.controller.disarm_live_motion(request_stop=True)
                        except Exception:
                            pass
                        self.events.put(("profile_speed_start_error", str(exc)))

                threading.Thread(
                    target=speed_worker,
                    daemon=True,
                    name="profile-speed-start",
                ).start()
                return
            if self.compiled_pr_program is None or profile.kind != "position":
                messagebox.showerror("Live start unavailable", "No verified live PR position plan is present.", parent=self.root)
                return
            positive_puu_direction = "counter-clockwise" if self.pr_direction_control == 0 else "clockwise"
            csv_positive_direction = (
                positive_puu_direction
                if self.compiled_pr_program.command_sign == 1
                else ("clockwise" if positive_puu_direction == "counter-clockwise" else "counter-clockwise")
            )
            if not messagebox.askyesno(
                "Start physical PR motion?",
                f"P1.001.Z={self.pr_direction_control}; positive PUU is {positive_puu_direction} at the motor shaft. "
                f"With the selected CSV sign, positive platform commands drive the motor {csv_positive_direction}.\n\n"
                "Confirm the SON selector is ON, EMGS/STO have been tested, the installed platform direction and clearance are known, and the operating speed cap shown in Settings is safe.\n\n"
                "Start PR#1 now?",
                parent=self.root,
            ):
                return
            if not self._capture_profile_run_origin():
                messagebox.showerror(
                    "Profile start unavailable",
                    "A current encoder position is required to establish this run's 0° reference.",
                    parent=self.root,
                )
                return
            self.test_preparing = True
            self.test_status.set("ARMING LIVE PR")
            self.profile_validation_text.set("ARMING • final live checks")
            self._update_test_start_state()

            def worker() -> None:
                try:
                    cap = self.applied_speed_limit_rpm
                    self.controller.arm_live_motion(cap, (True, True, True, True))
                    self.events.put(("profile_pr_armed", self._arm_lattepanda_profile(profile)))
                except Exception as exc:
                    try:
                        self.controller.disarm_live_motion(request_stop=True)
                    except Exception:
                        pass
                    self.events.put(("profile_pr_start_error", str(exc)))

            threading.Thread(target=worker, daemon=True, name="profile-pr-start").start()
            return
        self.test_preparing = True
        self.test_running = False
        self.test_status.set("HOMING")
        self.profile_homing_started_at = monotonic()
        self.profile_export_button.configure(state="disabled")
        self._update_test_start_state()
        self._log("Session homing started", "INFO", "PROFILE", "Returning to connection-time encoder origin")
        self._run_profile_homing_tick()

    def _fresh_lattepanda_clock_sample(self) -> ClockSyncSample:
        """Return a fresh exchange, restarting a stale UDP client once."""
        # A running sync thread can have no fresh replies. Arming needs a usable
        # observation, not merely a connected-looking status label.
        sync_sample = self.clock_sync.best_recent_sample()
        age_s = (
            float("inf")
            if sync_sample is None
            else (perf_counter_ns() - sync_sample.local_receive_monotonic_ns) / 1_000_000_000.0
        )
        if self.clock_sync.running and sync_sample is not None and age_s <= 2.0:
            return sync_sample

        host = self.clock_sync.resolved_ip or self.clock_sync.host
        if not host or host.casefold() == "auto":
            raise RuntimeError("Start LattePanda clock sync before arming a physical profile")
        port = self.clock_sync.port
        rate_hz = self.clock_sync.rate_hz
        self.clock_sync.clear()
        self.clock_sync.start(host, port, rate_hz)
        deadline = monotonic() + 4.0
        while monotonic() < deadline:
            sync_sample = self.clock_sync.best_recent_sample()
            if sync_sample is not None:
                age_s = (
                    perf_counter_ns() - sync_sample.local_receive_monotonic_ns
                ) / 1_000_000_000.0
                if age_s <= 1.0:
                    return sync_sample
            threading.Event().wait(0.05)
        raise RuntimeError(
            f"LattePanda clock sync did not recover within 4 seconds ({host}:{port})"
        )

    def _arm_lattepanda_profile(self, profile: MotionProfile) -> TestRunClock:
        sync_sample = self._fresh_lattepanda_clock_sample()
        host = self.clock_sync.resolved_ip or self.clock_sync.host
        if not host or host.casefold() == "auto":
            raise RuntimeError("The LattePanda coordinator address is unavailable")
        client = UdpRunCoordinatorClient(host)
        run_id = uuid.uuid4().hex
        acknowledgement = client.arm(
            run_id=run_id,
            profile_name=profile.source.name,
            expected_duration_s=profile.duration_s,
        )
        if not acknowledgement.camera_enabled:
            try:
                client.cancel(run_id)
            finally:
                raise RuntimeError(
                    "LattePanda acknowledged the run without EVK4 recording enabled; "
                    "start its responder with --camera-recording"
                )
        if not acknowledgement.serial_trigger_enabled:
            try:
                client.cancel(run_id)
            finally:
                raise RuntimeError(
                    "LattePanda acknowledged the run without the COM1 T0 trigger enabled; "
                    "start its responder with --trigger"
                )
        if not acknowledgement.serial_trigger_healthy:
            try:
                client.cancel(run_id)
            finally:
                raise RuntimeError("LattePanda reported that the COM1 trigger is not healthy")
        if not acknowledgement.camera_healthy:
            try:
                client.cancel(run_id)
            finally:
                raise RuntimeError("LattePanda reported that the EVK4 stream is not healthy")
        alignment_sample = self.clock_sync.best_recent_sample()
        if alignment_sample is None:
            client.cancel(run_id)
            raise RuntimeError("Clock synchronization was lost while the LattePanda prepared the run")
        alignment_age_s = (
            perf_counter_ns() - alignment_sample.local_receive_monotonic_ns
        ) / 1_000_000_000.0
        if alignment_age_s > 2.0:
            client.cancel(run_id)
            raise RuntimeError(
                f"Clock synchronization became stale while arming ({alignment_age_s:.1f} s)"
            )
        self.profile_run_coordinator = client
        run_clock = TestRunClock.from_scheduled_t0(
            profile.source,
            acknowledgement.lattepanda_t0_monotonic_ns,
            alignment_sample,
            acknowledgement.run_id,
        )
        remaining_s = (run_clock.motor_pc_t0_monotonic_ns - perf_counter_ns()) / 1_000_000_000.0
        if remaining_s < 0.5:
            try:
                client.cancel(run_id)
            finally:
                self.profile_run_coordinator = None
                raise RuntimeError(
                    f"LattePanda T0 acknowledgement arrived too late ({remaining_s:+.3f} s remaining)"
                )
        self._start_lattepanda_health_monitor(client, run_id)
        return run_clock

    def _start_lattepanda_health_monitor(
        self, client: UdpRunCoordinatorClient, run_id: str,
    ) -> None:
        previous = getattr(self, "profile_coordinator_monitor_stop", None)
        if previous is not None:
            previous.set()
        stop_event = threading.Event()
        self.profile_coordinator_monitor_stop = stop_event

        def worker() -> None:
            consecutive_errors = 0
            while not stop_event.wait(1.0):
                try:
                    status = client.status(run_id)
                    consecutive_errors = 0
                except Exception as exc:
                    consecutive_errors += 1
                    if consecutive_errors >= 3:
                        self.events.put((
                            "profile_coordinator_health_error",
                            f"Lost LattePanda run monitoring after 3 checks: {exc}",
                        ))
                        return
                    continue
                state = str(status.get("state", ""))
                if state.startswith("WATCHDOG_TIMEOUT"):
                    self.events.put((
                        "profile_coordinator_health_error",
                        "LattePanda safety watchdog finalized the camera recording",
                    ))
                    return
                if not bool(status.get("camera_healthy", False)):
                    self.events.put((
                        "profile_coordinator_health_error",
                        f"EVK4 stream failed: {status.get('camera_health_details', 'unknown reason')}",
                    ))
                    return
                if not bool(status.get("camera_recording_active", False)):
                    self.events.put((
                        "profile_coordinator_health_error",
                        "LattePanda reports that RAW recording stopped during the run",
                    ))
                    return
                if not bool(status.get("serial_trigger_healthy", False)):
                    self.events.put((
                        "profile_coordinator_health_error",
                        f"COM1 trigger failed: "
                        f"{status.get('serial_trigger_health_details', 'unknown reason')}",
                    ))
                    return
                if state == "RUNNING" and not bool(status.get("serial_trigger_active", False)):
                    self.events.put((
                        "profile_coordinator_health_error",
                        "LattePanda reports that the 1 Hz COM1 trigger thread stopped",
                    ))
                    return

        threading.Thread(
            target=worker, daemon=True, name="lattepanda-run-health",
        ).start()

    def _schedule_armed_profile(self, run_clock: TestRunClock, pr_mode: bool) -> None:
        self.profile_run_clock = run_clock
        delay_ms = max(0, round((run_clock.motor_pc_t0_monotonic_ns - perf_counter_ns()) / 1_000_000.0))
        self.test_status.set(f"ARMED • T0 IN {delay_ms / 1000.0:.3f} s")
        self.profile_validation_text.set("ARMED • LattePanda selected T0 • waiting to release motion")
        self._log(
            "LattePanda run armed", "INFO", "SYNC",
            f"run {run_clock.run_id}; T0 {run_clock.lattepanda_t0_monotonic_ns} ns",
        )
        self._record_profile_event(
            "run_armed",
            details=f"scheduled LattePanda T0={run_clock.lattepanda_t0_monotonic_ns}",
        )
        self.profile_local_events.append({
            "source": "lattepanda",
            "event": "scheduled_t0",
            "motor_pc_monotonic_ns": run_clock.motor_pc_t0_monotonic_ns,
            "lattepanda_monotonic_ns": run_clock.lattepanda_t0_monotonic_ns,
            "lattepanda_time_s": 0.0,
            "details": "Authoritative run time zero",
        })

        def release() -> None:
            self.profile_run_after_id = None
            if not self.test_preparing:
                return
            self._record_profile_event("motor_release_requested")
            if not pr_mode:
                self._begin_motion_profile_execution()
                return

            def trigger() -> None:
                try:
                    self._record_profile_event("pr_trigger_write_started")
                    self.controller.trigger_verified_pr_program()
                    self._record_profile_event("pr_trigger_write_completed")
                    self.events.put(("profile_pr_started", None))
                except Exception as exc:
                    self.events.put(("profile_pr_start_error", str(exc)))

            threading.Thread(target=trigger, daemon=True, name="profile-pr-release").start()

        self.profile_run_after_id = self.root.after(delay_ms, release)

    def _finish_lattepanda_profile(self, completed: bool) -> None:
        monitor_stop = getattr(self, "profile_coordinator_monitor_stop", None)
        if monitor_stop is not None:
            monitor_stop.set()
        client = self.profile_run_coordinator
        run_clock = self.profile_run_clock
        self.profile_run_coordinator = None
        if client is None or run_clock is None:
            self._auto_export_profile_run()
            return

        def worker() -> None:
            try:
                if completed:
                    response = client.complete(run_clock.run_id)
                else:
                    response = client.cancel(run_clock.run_id)
                self.events.put(("profile_coordinator_finished", response))
            except Exception as exc:
                self.events.put(("profile_coordinator_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="profile-coordinator-finish").start()

    def _begin_live_pr_execution(self) -> None:
        if self.written_profile is None:
            return
        self.test_preparing = False
        self.test_running = True
        self.test_status.set("RUNNING LIVE PR")
        self.profile_running_started_at = (
            perf_counter() if self.profile_run_clock is None
            else self.profile_run_clock.motor_pc_t0_monotonic_ns / 1_000_000_000.0
        )
        self.profile_cursor_s = 0.0
        self.profile_waypoint_index = 0
        self.profile_waypoint_direction = None
        self.profile_waypoint_started_at = monotonic()
        self.profile_waypoint_hold_started_at = None
        self.profile_waypoint_stop_latched = False
        self._start_profile_recording()
        self.profile_validation_text.set("RUNNING LIVE PR • P5.007 = 1")
        self._update_test_start_state()
        self._log("Live PR profile started", "WARN", "PROFILE", "PR#1 triggered through P5.007")
        self._schedule_live_pr_status()

    def _capture_profile_run_origin(self) -> bool:
        """Set the profile plot/export zero from the current encoder position."""
        origin_motor_deg: float | None = None
        if self.telemetry.samples:
            origin_motor_deg = self.telemetry.samples[-1].motor_position_deg
        if (
            origin_motor_deg is None
            and self.last_snapshot is not None
            and self.last_snapshot.encoder_position_pulses is not None
        ):
            origin_motor_deg = self.last_snapshot.encoder_position_pulses * 360.0 / (1 << 24)
        if origin_motor_deg is None:
            return False
        self.profile_run_origin_motor_deg = origin_motor_deg
        self.profile_preview.set_position_origin(origin_motor_deg)
        self._log(
            "Profile run origin captured",
            "INFO",
            "PROFILE",
            f"Recorded position zero = motor {origin_motor_deg:+.3f}°",
        )
        return True

    def _schedule_live_pr_status(self) -> None:
        if not self.test_running or self.controller.simulated:
            return

        def worker() -> None:
            try:
                self.events.put(("profile_pr_status", self.controller.read_pr_execution_status()))
            except Exception as exc:
                self.events.put(("profile_pr_status_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="profile-pr-status").start()

    def _handle_live_pr_status(self, status: PrExecutionStatus) -> None:
        profile = self.written_profile
        if not self.test_running or profile is None or self.profile_running_started_at is None:
            return
        elapsed = perf_counter() - self.profile_running_started_at
        self.profile_cursor_s = min(elapsed, profile.duration_s)
        self._update_running_profile_preview(profile, self.profile_cursor_s)
        final_path = len(self.compiled_pr_program.paths) if self.compiled_pr_program is not None else profile.pr_path_count
        if status.path_number is not None:
            self.test_status.set(f"LIVE PR#{status.path_number}/{final_path}")
        if status.target_reached and status.path_number == final_path:
            self.test_running = False
            self.profile_run_after_id = None
            self.test_status.set("COMPLETE")
            self.profile_validation_text.set("COMPLETE • final PR path reached and holding position")
            self.profile_workflow_text.set("✓  IMPORT   →   ✓  COMPILED   →   ✓  WRITTEN   →   ✓  COMPLETE")
            self._freeze_profile_recording()
            self._finish_lattepanda_profile(completed=True)
            self.profile_export_button.configure(state="normal" if self.profile_recorded_samples else "disabled")
            self.controller.disarm_live_motion(request_stop=False)
            self._update_test_start_state()
            self._resume_snapshot_polling()
            self._log("Live PR profile complete", "INFO", "PROFILE", f"PR#{final_path} reached TPOS")
            return
        expected_duration_s = (
            sum(path.predicted_duration_s for path in self.compiled_pr_program.paths)
            if self.compiled_pr_program is not None
            else profile.duration_s
        )
        timeout_s = expected_duration_s + max(5.0, expected_duration_s * 0.25)
        if elapsed > timeout_s:
            self._abort_motion_profile(f"Live PR exceeded {timeout_s:g} s completion timeout")
            return
        self.profile_run_after_id = self.root.after(100, self._schedule_live_pr_status)

    def _run_profile_homing_tick(self) -> None:
        if not self.test_preparing or self.written_profile is None:
            return
        if self.profile_homing_started_at is None or monotonic() - self.profile_homing_started_at > 60.0:
            self._abort_motion_profile("Session homing timed out")
            return
        motor_angle, platform_angle = self._session_angles()
        feedback = None if self.last_snapshot is None else self.last_snapshot.feedback_speed_rpm
        if platform_angle is None or feedback is None:
            self.profile_run_after_id = self.root.after(100, self._run_profile_homing_tick)
            return
        if abs(platform_angle) <= 0.5 and abs(feedback) <= 0.5:
            try:
                self.controller.update_speed_command_rpm(0.0, force=True)
            except Exception as exc:
                self._abort_motion_profile(f"Homing stop failed: {exc}")
                return
            self.test_status.set("HOMED • SETTLING")
            self._log("Session home reached", "INFO", "PROFILE", f"Platform error {platform_angle:+.3f}°")
            self.profile_run_after_id = self.root.after(300, self._begin_motion_profile_execution)
            return
        home_speed = max(-30.0, min(30.0, -platform_angle * self.written_profile.ratio * 0.4))
        try:
            self.controller.update_speed_command_rpm(home_speed)
        except Exception as exc:
            self._abort_motion_profile(f"Homing command failed: {exc}")
            return
        self.test_status.set(f"HOMING • {platform_angle:+.1f}°")
        self.profile_run_after_id = self.root.after(100, self._run_profile_homing_tick)

    def _begin_motion_profile_execution(self) -> None:
        if not self.test_preparing or self.written_profile is None:
            return
        profile = self.written_profile
        if not self._capture_profile_run_origin():
            self._abort_motion_profile("Current encoder position is unavailable for the run origin")
            return
        self.profile_run_after_id = None
        self.test_preparing = False
        self.test_running = True
        self.test_status.set("RUNNING")
        self.profile_running_started_at = (
            perf_counter() if self.profile_run_clock is None
            else self.profile_run_clock.motor_pc_t0_monotonic_ns / 1_000_000_000.0
        )
        self.profile_stopping_started_at = None
        self.profile_zero_speed_since = None
        self.profile_last_zero_sample_time = None
        self.profile_cursor_s = 0.0
        self.profile_waypoint_index = 0
        self.profile_waypoint_direction = None
        self.profile_waypoint_started_at = monotonic()
        self.profile_waypoint_hold_started_at = None
        self.profile_waypoint_stop_latched = False
        initial_target = (
            profile.position_path_targets_deg[0]
            if profile.kind == "position" and profile.position_path_targets_deg
            else 0.0
        )
        self.profile_position_target_history = [(0.0, initial_target)]
        self.profile_preview.set_position_targets(self.profile_position_target_history)
        self._start_profile_recording()
        self.profile_validation_text.set("RUNNING")
        self._update_test_start_state()
        self._log("Motion profile started", "INFO", "PROFILE", f"{profile.kind}; t=0 after session homing")
        if profile.kind == "speed":
            self._schedule_speed_profile_safety_stop(profile)
            self._start_speed_profile_execution_worker(profile)
        self._run_motion_profile_tick()

    def _start_speed_profile_execution_worker(self, profile: MotionProfile) -> None:
        """Execute CSV speed rows from absolute deadlines, independent of Tk."""
        # Capture this run's identity/cancellation token. Do not gate this worker on
        # the legacy shared test_running flag or Tk callbacks; unrelated UI updates
        # must not suppress later CSV rows or the terminal zero command.
        cancel = threading.Event()
        self.profile_execution_cancel = cancel
        run_id = None if self.profile_run_clock is None else self.profile_run_clock.run_id
        # TestRunClock is based on perf_counter_ns. Deadlines must use the same
        # clock: on Windows, monotonic() and perf_counter() can have different
        # epochs, which otherwise delays every command by the epoch offset.
        started_at = self.profile_running_started_at or perf_counter()

        def worker() -> None:
            for index, point in enumerate(profile.points):
                remaining_s = started_at + point.time_s - perf_counter()
                if cancel.wait(max(0.0, remaining_s)):
                    return
                current_id = None if self.profile_run_clock is None else self.profile_run_clock.run_id
                if current_id != run_id:
                    return
                is_final_stop = index == len(profile.points) - 1
                try:
                    if is_final_stop:
                        stopped_at = self._issue_verified_speed_profile_stop("csv_timeline")
                        self.events.put(("profile_safety_stop_completed", stopped_at))
                        return
                    self._record_profile_event(
                        "motor_command_write_started",
                        details=f"command={point.value:+.3f} rpm; row={index + 1}; source=csv_timeline",
                    )
                    command_pending = getattr(self, "motion_command_pending", None)
                    if command_pending is not None:
                        command_pending.set()
                    try:
                        self.controller.update_speed_command_rpm(point.value, force=True)
                    finally:
                        if command_pending is not None:
                            command_pending.clear()
                    self.profile_first_motion_command_recorded = True
                    self._record_profile_event(
                        "motor_command_write_completed",
                        details=f"command={point.value:+.3f} rpm; row={index + 1}; source=csv_timeline",
                    )
                    self.events.put(("profile_speed_command_completed", (point.time_s, point.value)))
                except Exception as exc:
                    self.events.put((
                        "profile_speed_timeline_error",
                        f"CSV row {index + 1} at t={point.time_s:g} s failed: {exc}",
                    ))
                    return

        thread = threading.Thread(target=worker, daemon=True, name="speed-profile-timeline")
        self.profile_execution_thread = thread
        thread.start()

    def _schedule_speed_profile_safety_stop(self, profile: MotionProfile) -> None:
        """Arm an independent stop deadline that does not depend on Tk callbacks."""
        # This independent deadline intentionally overlaps the timeline's final
        # row. The stop lock/event coalesce competing requests into one verified stop.
        self._cancel_profile_safety_stop_timer()
        self.profile_safety_stop_complete.clear()
        self.profile_safety_stop_monotonic_s = None
        run_id = None if self.profile_run_clock is None else self.profile_run_clock.run_id
        started_at = self.profile_running_started_at or perf_counter()
        delay_s = max(0.0, started_at + profile.duration_s - perf_counter())

        def worker() -> None:
            current_id = None if self.profile_run_clock is None else self.profile_run_clock.run_id
            if current_id != run_id or self.profile_execution_cancel.is_set():
                return
            try:
                stopped_at = self._issue_verified_speed_profile_stop("independent_deadline")
                self.events.put(("profile_safety_stop_completed", stopped_at))
            except Exception as exc:
                self.events.put(("profile_safety_stop_error", str(exc)))

        timer = threading.Timer(delay_s, worker)
        timer.daemon = True
        timer.name = "profile-independent-stop"
        self.profile_safety_stop_timer = timer
        timer.start()

    def _run_motion_profile_tick(self) -> None:
        if self.written_profile is None or self.profile_running_started_at is None:
            return
        profile = self.written_profile
        if profile.kind == "position" and not self.test_running:
            return
        if profile.kind == "speed" and not self.profile_recording_active:
            return
        elapsed = perf_counter() - self.profile_running_started_at
        self.profile_cursor_s = elapsed if profile.kind == "position" else min(elapsed, profile.duration_s)
        self._update_running_profile_preview(profile, self.profile_cursor_s)
        if profile.kind == "position":
            self._run_speed_controlled_position_tick()
            return
        if self.profile_safety_stop_complete.is_set():
            self._run_speed_profile_stop_tick()
            return
        self.test_status.set(
            f"RUNNING • t={min(elapsed, profile.duration_s):.1f}/{profile.duration_s:g} s • "
            f"command {self.controller.cached_speed_command_rpm:+.1f} rpm"
        )
        self.profile_run_after_id = self.root.after(100, self._run_motion_profile_tick)

    def _issue_verified_speed_profile_stop(self, source: str) -> float:
        """Stop telemetry, write/verify zero once, and return its monotonic time."""
        # Bus priority requires stopping the telemetry producer before the write.
        # The returned timestamp uses monotonic for local stop/settling bookkeeping;
        # profile T0 and exported event timestamps use perf_counter separately.
        with self.profile_safety_stop_lock:
            if self.profile_safety_stop_complete.is_set():
                return self.profile_safety_stop_monotonic_s or monotonic()
            self._stop_motion_polling()
            thread = self.motion_poll_thread
            if (
                thread is not None
                and thread.is_alive()
                and thread is not threading.current_thread()
            ):
                thread.join(timeout=max(1.0, self.controller.config.timeout_s + 0.5))
            self._record_profile_event(
                "motor_stop_write_started",
                details=f"command=+0.000 rpm; verified=true; source={source}",
            )
            try:
                self.controller.controlled_stop()
            except Exception as exc:
                self._record_profile_event(
                    "motor_stop_write_failed",
                    details=f"command=+0.000 rpm; source={source}; error={exc}",
                )
                raise
            stopped_at = monotonic()
            self.profile_safety_stop_monotonic_s = stopped_at
            self.profile_safety_stop_complete.set()
            self._record_profile_event(
                "motor_stop_write_completed",
                details=f"command=+0.000 rpm; verified=true; source={source}",
            )
            return stopped_at

    def _run_speed_profile_stop_tick(self) -> None:
        """Keep recording after the CSV ends until encoder speed is stably zero."""
        now = monotonic()
        if self.profile_stopping_started_at is None:
            zero_command_verified = False
            try:
                stopped_at = self._issue_verified_speed_profile_stop("ui_deadline")
                zero_command_verified = True
            except Exception as exc:
                self._abort_motion_profile(f"Final zero-speed command failed: {exc}")
                return
            finally:
                if (
                    zero_command_verified
                    and self.controller.state is ConnectionState.CONNECTED
                    and not self.motion_polling
                ):
                    self._start_motion_polling()
            self.profile_stopping_started_at = stopped_at
            self.profile_zero_speed_since = None
            self.profile_last_zero_sample_time = None
            self.test_status.set("STOPPING • waiting for zero speed")
            self.profile_validation_text.set(
                "STOPPING • CSV commands complete • recording until encoder speed is zero"
            )
            self._log(
                "Speed CSV commands complete",
                "INFO",
                "PROFILE",
                "0 rpm commanded; telemetry recording continues",
            )

        feedback: float | None = None
        sample_time: float | None = None
        if self.telemetry.samples:
            latest = self.telemetry.samples[-1]
            feedback = latest.encoder_feedback_rpm
            sample_time = latest.sampled_monotonic_s

        # Only accept telemetry acquired after the zero command. Require a
        # short stable interval so one rounded/isolated zero sample cannot end
        # the recording while the shaft is still decelerating.
        fresh = (
            sample_time is not None
            and self.profile_stopping_started_at is not None
            and sample_time >= self.profile_stopping_started_at
        )
        if fresh and feedback is not None and abs(feedback) <= self.PROFILE_ZERO_SPEED_RPM:
            if self.profile_zero_speed_since is None:
                self.profile_zero_speed_since = sample_time
                self.profile_last_zero_sample_time = sample_time
            elif (
                sample_time is not None
                and self.profile_last_zero_sample_time is not None
                and sample_time > self.profile_last_zero_sample_time
            ):
                self.profile_last_zero_sample_time = sample_time
            if (
                sample_time is not None
                and self.profile_zero_speed_since is not None
                and self.profile_last_zero_sample_time == sample_time
                and sample_time - self.profile_zero_speed_since >= self.PROFILE_ZERO_STABLE_S
            ):
                self._complete_speed_profile_after_stop(feedback)
                return
        else:
            self.profile_zero_speed_since = None
            self.profile_last_zero_sample_time = None

        if (
            self.profile_stopping_started_at is not None
            and now - self.profile_stopping_started_at > self.PROFILE_STOP_TIMEOUT_S
        ):
            feedback_text = "unavailable" if feedback is None else f"{feedback:+.2f} rpm"
            self._abort_motion_profile(
                f"Encoder did not reach zero speed within {self.PROFILE_STOP_TIMEOUT_S:g} s "
                f"(last feedback {feedback_text})"
            )
            return

        self.test_status.set(
            "STOPPING • waiting for fresh encoder feedback"
            if feedback is None or not fresh
            else f"STOPPING • encoder {feedback:+.2f} rpm"
        )
        self.profile_cursor_s = self.written_profile.duration_s if self.written_profile else None
        self.profile_run_after_id = self.root.after(
            self._position_control_interval_ms(),
            self._run_motion_profile_tick,
        )

    def _complete_speed_profile_after_stop(self, feedback_rpm: float) -> None:
        self._cancel_speed_profile_execution()
        self._cancel_profile_safety_stop_timer()
        try:
            if not self.controller.simulated:
                self.controller.disarm_live_motion(request_stop=False)
        except Exception as exc:
            self._abort_motion_profile(f"Final motion disarm failed: {exc}")
            return
        self.test_running = False
        self.profile_run_after_id = None
        self.test_status.set("COMPLETE • ZERO SPEED")
        self.profile_validation_text.set(
            f"COMPLETE • encoder settled at {feedback_rpm:+.2f} rpm • telemetry retained for export"
        )
        self.profile_workflow_text.set(
            "✓  IMPORT   →   ✓  COMPILED   →   ✓  WRITTEN   →   ✓  STOPPED / COMPLETE"
        )
        self._freeze_profile_recording()
        self._finish_lattepanda_profile(completed=True)
        self._update_test_start_state()
        self._resume_snapshot_polling()
        self.profile_export_button.configure(
            state="normal" if self.profile_recorded_samples else "disabled"
        )
        self._log(
            "Motion profile complete",
            "INFO",
            "PROFILE",
            f"Encoder zero speed confirmed at {feedback_rpm:+.2f} rpm",
        )

    def _run_speed_controlled_position_tick(self) -> None:
        """Execute position waypoints through bounded Speed-mode commands."""
        # Unlike the dedicated speed timeline, this feedback position loop is
        # Tk-scheduled. Its waypoint tolerances are software behavior, not a claim
        # of platform accuracy through backlash/compliance.
        profile = self.written_profile
        if not self.test_running or profile is None:
            return
        targets = profile.position_path_targets_deg
        speeds = profile.position_path_speeds_rpm
        holds = profile.position_path_holds_s
        if targets is None or speeds is None or holds is None:
            self._abort_motion_profile("Position profile has no speed-controlled waypoint plan")
            return
        if self.profile_waypoint_index >= len(targets):
            self._complete_speed_controlled_position_profile()
            return
        if not self.telemetry.samples or self.profile_run_origin_motor_deg is None:
            self.profile_run_after_id = self.root.after(self._position_control_interval_ms(), self._run_motion_profile_tick)
            return

        sample = self.telemetry.samples[-1]
        if sample.motor_position_deg is None:
            self._abort_motion_profile("Encoder position feedback was lost")
            return
        command_sign = self._platform_command_sign()
        current_platform = (
            (sample.motor_position_deg - self.profile_run_origin_motor_deg)
            / profile.ratio
            * command_sign
        )
        index = self.profile_waypoint_index
        target = targets[index]
        feedback_speed = sample.encoder_feedback_rpm
        if feedback_speed is None:
            self._abort_motion_profile("Motor speed feedback was lost")
            return

        # A zero-speed row is a hold at the current target. Moving rows acquire
        # their direction once and never reverse to chase belt take-up.
        if index == 0 or speeds[index] == 0:
            try:
                self.controller.update_speed_command_rpm(0.0)
            except Exception as exc:
                self._abort_motion_profile(f"Position stop command failed: {exc}")
                return
            if feedback_speed is not None and abs(feedback_speed) > 0.5:
                self.profile_run_after_id = self.root.after(self._position_control_interval_ms(), self._run_motion_profile_tick)
                return
            if self.profile_waypoint_hold_started_at is None:
                self.profile_waypoint_hold_started_at = monotonic()
            if monotonic() - self.profile_waypoint_hold_started_at >= holds[index]:
                self._advance_speed_controlled_waypoint()
            else:
                self.test_status.set(f"HOLDING POSITION • {index + 1}/{len(targets)}")
            self.profile_run_after_id = self.root.after(self._position_control_interval_ms(), self._run_motion_profile_tick)
            return

        if self.profile_waypoint_direction is None:
            error = target - current_platform
            if abs(error) <= 0.5:
                self.profile_waypoint_direction = 1 if index == 0 or target >= targets[index - 1] else -1
            else:
                self.profile_waypoint_direction = 1 if error > 0 else -1
            self.profile_waypoint_started_at = monotonic()

        if self.profile_waypoint_started_at is not None:
            distance = abs(targets[index] - targets[index - 1])
            nominal_s = distance * profile.ratio / 360.0 * 60.0 / speeds[index]
            timeout_s = max(10.0, nominal_s * 3.0 + 5.0)
            if monotonic() - self.profile_waypoint_started_at > timeout_s:
                self._abort_motion_profile(f"Waypoint {index + 1} exceeded its {timeout_s:.1f} s timeout")
                return

        if position_waypoint_can_overlap(profile, index):
            directed_error = (target - current_platform) * self.profile_waypoint_direction
            if directed_error <= 0:
                overshoot = -directed_error
                sample_period_s = 1.0 / max(1.0, self.motion_target_hz)
                expected_step_deg = abs(feedback_speed) / profile.ratio * 6.0 * sample_period_s
                pass_through_tolerance_deg = max(5.0, expected_step_deg * 3.0 + 2.0)
                if overshoot > pass_through_tolerance_deg:
                    self._abort_motion_profile(
                        f"Pass-through waypoint {index + 1} overshot by {overshoot:.2f} platform degrees "
                        f"(dynamic limit {pass_through_tolerance_deg:.2f}°)"
                    )
                    return
                self._advance_speed_controlled_waypoint()
                self.profile_run_after_id = self.root.after(1, self._run_motion_profile_tick)
                return
            motor_rpm = self.profile_waypoint_direction * speeds[index]
            try:
                self.controller.update_speed_command_rpm(motor_rpm * command_sign)
            except Exception as exc:
                self._abort_motion_profile(f"Pass-through speed command failed: {exc}")
                return
            self.test_status.set(
                f"PASS-THROUGH {index + 1}/{len(targets)} • target {target:+.1f}° • "
                f"error {target - current_platform:+.2f}° • {motor_rpm:+.1f} rpm"
            )
            self.profile_run_after_id = self.root.after(self._position_control_interval_ms(), self._run_motion_profile_tick)
            return

        if self.profile_waypoint_stop_latched:
            try:
                self.controller.update_speed_command_rpm(0.0)
            except Exception as exc:
                self._abort_motion_profile(f"Position stop command failed: {exc}")
                return
            if feedback_speed is not None and abs(feedback_speed) <= 0.5:
                if self.profile_waypoint_hold_started_at is None:
                    self.profile_waypoint_hold_started_at = monotonic()
                if monotonic() - self.profile_waypoint_hold_started_at >= holds[index]:
                    self._advance_speed_controlled_waypoint()
            self.profile_run_after_id = self.root.after(self._position_control_interval_ms(), self._run_motion_profile_tick)
            return

        try:
            motor_rpm, reached, overshoot = position_speed_command(
                current_platform,
                target,
                speeds[index],
                profile.ratio,
                self.profile_waypoint_direction,
            )
        except ValueError as exc:
            self._abort_motion_profile(str(exc))
            return

        if overshoot > 5.0:
            self._abort_motion_profile(
                f"Waypoint {index + 1} overshot by {overshoot:.2f} platform degrees"
            )
            return
        if reached:
            motor_rpm = 0.0
            self.profile_waypoint_stop_latched = True
        try:
            self.controller.update_speed_command_rpm(motor_rpm * command_sign)
        except Exception as exc:
            self._abort_motion_profile(f"Position speed command failed: {exc}")
            return

        remaining = target - current_platform
        self.test_status.set(
            f"POSITION {index + 1}/{len(targets)} • target {target:+.1f}° • error {remaining:+.2f}° • {motor_rpm:+.1f} rpm"
        )
        if reached and abs(feedback_speed) <= 0.5:
            if self.profile_waypoint_hold_started_at is None:
                self.profile_waypoint_hold_started_at = monotonic()
            if monotonic() - self.profile_waypoint_hold_started_at >= holds[index]:
                self._advance_speed_controlled_waypoint()
        self.profile_run_after_id = self.root.after(self._position_control_interval_ms(), self._run_motion_profile_tick)

    def _advance_speed_controlled_waypoint(self) -> None:
        self.profile_waypoint_index += 1
        self.profile_waypoint_direction = None
        self.profile_waypoint_started_at = monotonic()
        self.profile_waypoint_hold_started_at = None
        self.profile_waypoint_stop_latched = False
        profile = self.written_profile
        if (
            profile is not None
            and profile.position_path_targets_deg is not None
            and self.profile_waypoint_index < len(profile.position_path_targets_deg)
        ):
            elapsed = (
                0.0
                if self.profile_running_started_at is None
                else perf_counter() - self.profile_running_started_at
            )
            self.profile_position_target_history.append(
                (elapsed, profile.position_path_targets_deg[self.profile_waypoint_index])
            )
            self.profile_preview.set_position_targets(self.profile_position_target_history)

    def _complete_speed_controlled_position_profile(self) -> None:
        """Command zero and require fresh, stable zero-speed feedback."""
        now = monotonic()
        if self.profile_stopping_started_at is None:
            try:
                self.controller.update_speed_command_rpm(0.0, force=True)
            except Exception as exc:
                self._abort_motion_profile(f"Final position stop failed: {exc}")
                return
            self.profile_stopping_started_at = now
            self.profile_zero_speed_since = None
            self.profile_last_zero_sample_time = None
            self.test_status.set("STOPPING POSITION PROFILE • waiting for zero speed")
            self.profile_validation_text.set(
                "STOPPING • final waypoint reached • recording until encoder speed is zero"
            )

        feedback: float | None = None
        sample_time: float | None = None
        if self.telemetry.samples:
            latest = self.telemetry.samples[-1]
            feedback = latest.encoder_feedback_rpm
            sample_time = latest.sampled_monotonic_s
        fresh = (
            sample_time is not None
            and self.profile_stopping_started_at is not None
            and sample_time >= self.profile_stopping_started_at
        )
        if fresh and feedback is not None and abs(feedback) <= self.PROFILE_ZERO_SPEED_RPM:
            if self.profile_zero_speed_since is None:
                self.profile_zero_speed_since = sample_time
                self.profile_last_zero_sample_time = sample_time
            elif (
                self.profile_last_zero_sample_time is not None
                and sample_time > self.profile_last_zero_sample_time
            ):
                self.profile_last_zero_sample_time = sample_time
            if (
                self.profile_zero_speed_since is not None
                and self.profile_last_zero_sample_time == sample_time
                and sample_time - self.profile_zero_speed_since >= self.PROFILE_ZERO_STABLE_S
            ):
                self._finalize_speed_controlled_position_profile(feedback)
                return
        else:
            self.profile_zero_speed_since = None
            self.profile_last_zero_sample_time = None

        if (
            self.profile_stopping_started_at is not None
            and now - self.profile_stopping_started_at > self.PROFILE_STOP_TIMEOUT_S
        ):
            feedback_text = "unavailable" if feedback is None else f"{feedback:+.2f} rpm"
            self._abort_motion_profile(
                f"Position profile did not reach zero speed within "
                f"{self.PROFILE_STOP_TIMEOUT_S:g} s (last feedback {feedback_text})"
            )
            return
        self.profile_run_after_id = self.root.after(
            self._position_control_interval_ms(), self._run_motion_profile_tick,
        )

    def _finalize_speed_controlled_position_profile(self, feedback_rpm: float) -> None:
        try:
            if not self.controller.simulated:
                self.controller.disarm_live_motion(request_stop=False)
        except Exception as exc:
            self._abort_motion_profile(f"Final position disarm failed: {exc}")
            return
        self.test_running = False
        self.profile_run_after_id = None
        self.test_status.set("COMPLETE • ZERO SPEED")
        self.profile_validation_text.set(
            f"COMPLETE • position waypoints reached • encoder settled at {feedback_rpm:+.2f} rpm"
        )
        self.profile_workflow_text.set("✓  IMPORT   →   ✓  COMPILED   →   ✓  STAGED   →   ✓  COMPLETE")
        self._freeze_profile_recording()
        self._finish_lattepanda_profile(completed=True)
        self.profile_export_button.configure(state="normal" if self.profile_recorded_samples else "disabled")
        self._update_test_start_state()
        self._resume_snapshot_polling()
        self._log(
            "Speed-controlled position profile complete", "INFO", "PROFILE",
            f"Encoder zero speed confirmed at {feedback_rpm:+.2f} rpm",
        )

    def _abort_motion_profile(self, reason: str = "Operator abort") -> None:
        self._cancel_speed_profile_execution()
        self._cancel_profile_safety_stop_timer()
        active = self.test_running or self.test_preparing or self.profile_recording_active
        was_manual_home = self.profile_manual_home
        self.profile_manual_home = False
        self.test_running = False
        self.test_preparing = False
        if self.profile_run_after_id is not None:
            self.root.after_cancel(self.profile_run_after_id)
            self.profile_run_after_id = None
        if self.controller.state is ConnectionState.CONNECTED:
            if self.controller.simulated:
                self._run_command("Profile abort: zero speed", self.controller.controlled_stop)
            elif was_manual_home:
                def stop_home() -> None:
                    self.controller.controlled_stop()
                    self.controller.disarm_live_motion(request_stop=False)

                self._run_command("Home abort: zero speed", stop_home)
            elif self.written_profile is not None and self.compiled_pr_program is None:
                def stop_speed_profile() -> None:
                    self.controller.controlled_stop()
                    self.controller.disarm_live_motion(request_stop=False)

                self._run_command("Speed-mode CSV abort: zero speed", stop_speed_profile)
            else:
                self._run_command("Profile abort: P5.007 stop", self.controller.stop_pr_program)
        if active:
            self._freeze_profile_recording()
            self._finish_lattepanda_profile(completed=False)
            self.test_status.set("ABORTED")
            self.profile_validation_text.set(f"ABORTED • {reason} • 0 rpm requested")
            self.profile_workflow_text.set("✓  IMPORT   →   ✓  COMPILED   →   ✓  WRITTEN   →   !  ABORTED")
            self._log("Motion profile aborted", "WARN", "PROFILE", reason)
            self.profile_export_button.configure(state="normal" if self.profile_recorded_samples else "disabled")
        self._update_test_start_state()
        self._resume_snapshot_polling()

    def _session_angles(self) -> tuple[float | None, float | None]:
        if not self.telemetry.samples:
            return None, None
        sample = self.telemetry.samples[-1]
        if sample.motor_position_deg is None or sample.platform_position_deg is None:
            return None, None
        if self.session_origin_pulses is None:
            return 0.0, 0.0
        origin_motor = self.session_origin_pulses * 360.0 / (1 << 24)
        motor = sample.motor_position_deg - origin_motor
        return motor, motor / max(float(self.platform_ratio.get()), 1e-9)

    def _build_trends(self, page: ttk.Frame) -> None:
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.trends_notebook = ttk.Notebook(page)
        self.trends_notebook.grid(row=0, column=0, sticky="nsew")
        speed_tab = ttk.Frame(self.trends_notebook, style="App.TFrame", padding=(8, 10))
        position_tab = ttk.Frame(self.trends_notebook, style="App.TFrame", padding=(8, 10))
        self.trends_notebook.add(speed_tab, text="SPEED TRACKING")
        self.trends_notebook.add(position_tab, text="ANGULAR POSITION")

        for tab in (speed_tab, position_tab):
            tab.columnconfigure(0, weight=4)
            tab.columnconfigure(1, weight=1)
            tab.rowconfigure(1, weight=1)

        speed_toolbar = ttk.Frame(speed_tab, style="App.TFrame")
        speed_toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        speed_toolbar.columnconfigure(0, weight=1)
        ttk.Label(speed_toolbar, text="Encoder-based command tracking", background="#F3F6F9", foreground=MUTED, font=("Segoe UI", 10)).grid(row=0, column=0, sticky="w")
        self.speed_live_indicator = ttk.Label(speed_toolbar, text="● Live", background="#F3F6F9", foreground=GREEN, font=("Segoe UI", 10, "bold"))
        self.speed_live_indicator.grid(row=0, column=1, padx=6)
        ttk.Button(speed_toolbar, text="Pause", command=lambda: self._toggle_trend_pause("speed"), style="Outline.TButton").grid(row=0, column=2, padx=6)
        ttk.Button(speed_toolbar, text="Clear", command=self._clear_telemetry, style="Outline.TButton").grid(row=0, column=3, padx=6)
        ttk.Button(speed_toolbar, text="EXPORT CSV", command=lambda: self._export_telemetry("speed"), style="Primary.TButton").grid(row=0, column=4, padx=(6, 0))

        speed_chart_card = self._card(speed_tab, 14)
        speed_chart_card.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        speed_chart_card.columnconfigure(0, weight=1)
        speed_chart_card.rowconfigure(1, weight=1)
        ttk.Label(speed_chart_card, text="SPEED TRACKING", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.trends_chart = TrendChart(speed_chart_card)
        self.trends_chart.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
        ttk.Label(speed_chart_card, text="Requested motor speed -> application setpoint. Encoder feedback -> P0.010 when P0.018 = 7.", style="Muted.TLabel").grid(row=2, column=0, sticky="w", pady=(8, 0))

        speed_side = ttk.Frame(speed_tab, style="App.TFrame")
        speed_side.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        speed_side.columnconfigure(0, weight=1)
        speed_channels = self._card(speed_side, 14)
        speed_channels.grid(row=0, column=0, sticky="ew")
        ttk.Label(speed_channels, text="CHANNELS", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        for label, variable in (
            ("Requested motor speed", self.channel_command),
            ("Encoder feedback (P0.010)", self.channel_feedback),
            ("Tracking error (calculated)", self.channel_error),
        ):
            ttk.Checkbutton(speed_channels, text=label, variable=variable, command=self._update_trend_channels).pack(anchor="w", pady=6)
        ttk.Checkbutton(speed_channels, textvariable=self.channel_platform_label, variable=self.channel_platform, command=self._update_trend_channels).pack(anchor="w", pady=6)
        speed_stats = self._card(speed_side, 14)
        speed_stats.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        ttk.Label(speed_stats, text="CURRENT / STATS", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        for row, (label, variable) in enumerate((
            ("Requested motor speed", self.integrated_text),
            ("Encoder feedback", self.feedback_text),
            ("Tracking error", self.tracking_error_text),
            ("Platform speed", self.platform_speed_text),
            ("RMS error (60 s)", self.rms_error_text),
            ("Peak error (60 s)", self.peak_error_text),
            ("Motion acquisition", self.motion_quality_text),
        ), start=1):
            ttk.Label(speed_stats, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=(14 if row == 1 else 7, 4))
            ttk.Label(speed_stats, textvariable=variable, style="Card.TLabel").grid(row=row, column=1, sticky="e", pady=(14 if row == 1 else 7, 4))

        position_toolbar = ttk.Frame(position_tab, style="App.TFrame")
        position_toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        position_toolbar.columnconfigure(0, weight=1)
        ttk.Label(position_toolbar, text="Periodic encoder orientation: sin(θ)", background="#F3F6F9", foreground=MUTED, font=("Segoe UI", 10)).grid(row=0, column=0, sticky="w")
        self.position_live_indicator = ttk.Label(position_toolbar, text="● Live", background="#F3F6F9", foreground=GREEN, font=("Segoe UI", 10, "bold"))
        self.position_live_indicator.grid(row=0, column=1, padx=6)
        ttk.Button(position_toolbar, text="Pause", command=lambda: self._toggle_trend_pause("position"), style="Outline.TButton").grid(row=0, column=2, padx=6)
        ttk.Button(position_toolbar, text="Clear", command=self._clear_telemetry, style="Outline.TButton").grid(row=0, column=3, padx=6)
        ttk.Button(position_toolbar, text="EXPORT CSV", command=lambda: self._export_telemetry("position"), style="Primary.TButton").grid(row=0, column=4, padx=(6, 0))

        position_chart_card = self._card(position_tab, 14)
        position_chart_card.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        position_chart_card.columnconfigure(0, weight=1)
        position_chart_card.rowconfigure(1, weight=1)
        ttk.Label(position_chart_card, text="ANGULAR ORIENTATION — sin(θ)", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.position_chart = PositionChart(position_chart_card)
        self.position_chart.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
        ttk.Label(position_chart_card, text="θ comes from P0.011 when P0.019 = 3. The plotted value is sin(θ), with a fixed −1 to +1 scale.", style="Muted.TLabel").grid(row=2, column=0, sticky="w", pady=(8, 0))

        position_side = ttk.Frame(position_tab, style="App.TFrame")
        position_side.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        position_side.columnconfigure(0, weight=1)
        position_channels = self._card(position_side, 14)
        position_channels.grid(row=0, column=0, sticky="ew")
        ttk.Label(position_channels, text="CHANNELS", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        ttk.Checkbutton(position_channels, text="Motor sin(θ) (P0.011)", variable=self.channel_motor_position, command=self._update_position_channels).pack(anchor="w", pady=6)
        ttk.Checkbutton(position_channels, textvariable=self.channel_platform_position_label, variable=self.channel_platform_position, command=self._update_position_channels).pack(anchor="w", pady=6)
        position_stats = self._card(position_side, 14)
        position_stats.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        ttk.Label(position_stats, text="CURRENT / STATS", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        for row, (label, variable) in enumerate((
            ("Encoder position", self.encoder_position_text),
            ("Motor angle", self.motor_position_text),
            ("Motor sin(θ)", self.motor_sine_text),
            ("Platform angle", self.platform_position_text),
            ("Scene vertical displacement", self.drawstring_position_text),
            ("Platform sin(θ)", self.platform_sine_text),
            ("Platform speed", self.platform_speed_text),
            ("Motion acquisition", self.motion_quality_text),
        ), start=1):
            ttk.Label(position_stats, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=(14 if row == 1 else 7, 4))
            ttk.Label(position_stats, textvariable=variable, style="Card.TLabel").grid(row=row, column=1, sticky="e", pady=(14 if row == 1 else 7, 4))

    def _build_parameters(self, page: ttk.Frame) -> None:
        body = ttk.Frame(page, style="App.TFrame")
        body.grid(sticky="nsew")
        body.columnconfigure(0, weight=4)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)
        toolbar = ttk.Frame(body, style="App.TFrame")
        toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        toolbar.columnconfigure(0, weight=1)
        self.parameter_search = tk.StringVar()
        search = ttk.Entry(toolbar, textvariable=self.parameter_search, width=34)
        search.grid(row=0, column=0, sticky="w")
        search.insert(0, "")
        search.bind("<KeyRelease>", lambda _event: self._populate_parameter_table())
        groups = tuple(sorted({register.parameter.split(".", 1)[0] for register in PROJECT_PARAMETER_CATALOG}))
        self.parameter_group = tk.StringVar(value="All groups")
        group_filter = ttk.Combobox(toolbar, textvariable=self.parameter_group, values=("All groups", *groups), state="readonly", width=13)
        group_filter.grid(row=0, column=1, padx=6)
        group_filter.bind("<<ComboboxSelected>>", lambda _event: self._populate_parameter_table())
        self.parameter_read_button = ttk.Button(toolbar, text="READ FROM DRIVE", command=self._request_parameter_read, style="Primary.TButton")
        self.parameter_read_button.grid(row=0, column=2, padx=6)
        ttk.Button(toolbar, text="COMPARE PROFILE", command=self._request_parameter_read, style="Outline.TButton").grid(row=0, column=3, padx=(6, 0))
        ttk.Label(toolbar, text="Manual-verified project-relevant subset • filter by P0, P1, P2, P3…", background="#F3F6F9", foreground=MUTED, font=("Segoe UI", 8)).grid(row=1, column=0, columnspan=4, sticky="w", pady=(5, 0))

        table_card = self._card(body, (0, 0))
        table_card.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        table_card.columnconfigure(0, weight=1)
        table_card.rowconfigure(0, weight=1)
        columns = ("parameter", "description", "drive", "profile", "status")
        self.parameter_tree = ttk.Treeview(table_card, columns=columns, show="headings")
        headings = {"parameter": "Parameter", "description": "Description", "drive": "Drive value", "profile": "Project profile", "status": "Status"}
        widths = {"parameter": 100, "description": 330, "drive": 130, "profile": 130, "status": 110}
        for column in columns:
            self.parameter_tree.heading(column, text=headings[column])
            self.parameter_tree.column(column, width=widths[column], stretch=column == "description")
        self.parameter_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(table_card, orient="vertical", command=self.parameter_tree.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.parameter_tree.configure(yscrollcommand=scrollbar.set)
        self.parameter_tree.bind("<<TreeviewSelect>>", self._show_parameter_details)
        self.parameter_tree.tag_configure("match", foreground=GREEN)
        self.parameter_tree.tag_configure("mismatch", foreground=RED)
        self.parameter_tree.tag_configure("runtime", foreground=BLUE)

        detail = self._card(body, 16)
        detail.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        detail.columnconfigure(1, weight=1)
        self.parameter_detail_title = tk.StringVar(value="Select a parameter")
        self.parameter_detail_rows = {
            "Drive value": tk.StringVar(value="--"),
            "Project profile": tk.StringVar(value="--"),
            "Unit": tk.StringVar(value="--"),
            "Allowed range": tk.StringVar(value="--"),
            "Persistence": tk.StringVar(value="--"),
            "Address": tk.StringVar(value="--"),
        }
        ttk.Label(detail, textvariable=self.parameter_detail_title, style="Section.TLabel", wraplength=270).grid(row=0, column=0, columnspan=2, sticky="w")
        for row, (label, variable) in enumerate(self.parameter_detail_rows.items(), start=1):
            ttk.Label(detail, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=(18 if row == 1 else 8, 4))
            ttk.Label(detail, textvariable=variable, style="Card.TLabel").grid(row=row, column=1, sticky="e", pady=(18 if row == 1 else 8, 4))
        ttk.Separator(detail).grid(row=7, column=0, columnspan=2, sticky="ew", pady=14)
        self.parameter_note = tk.StringVar(value="Select a writable parameter. Simulation and guarded Live Drive writes are supported.")
        ttk.Label(detail, textvariable=self.parameter_note, style="Muted.TLabel", wraplength=290).grid(row=8, column=0, columnspan=2, sticky="w")
        ttk.Label(detail, text="New value", style="Muted.TLabel").grid(row=9, column=0, sticky="w", pady=(18, 4))
        edit_row = ttk.Frame(detail, style="Card.TFrame")
        edit_row.grid(row=9, column=1, sticky="ew", pady=(18, 4))
        edit_row.columnconfigure(0, weight=1)
        self.parameter_edit_entry = ttk.Entry(edit_row, textvariable=self.parameter_edit_value, justify="right", state="disabled")
        self.parameter_edit_entry.grid(row=0, column=0, sticky="ew")
        ttk.Label(edit_row, textvariable=self.parameter_edit_unit, style="Muted.TLabel").grid(row=0, column=1, padx=(5, 0))
        self.parameter_write_button = ttk.Button(
            detail,
            text="WRITE PARAMETER",
            command=self._write_selected_parameter,
            state="disabled",
            style="Primary.TButton",
        )
        self.parameter_write_button.grid(row=10, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        ttk.Label(detail, text="Every write is range-checked, read back, compared, and recorded. Live Drive additionally requires zero command, zero speed and Servo Off; motion and raw I/O commands retain their purpose-built safeguards.", style="Muted.TLabel", wraplength=290).grid(row=11, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self._populate_parameter_table()

    def _build_logs(self, page: ttk.Frame) -> None:
        body = ttk.Frame(page, style="App.TFrame")
        body.grid(sticky="nsew")
        body.columnconfigure(0, weight=4)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(body, style="App.TFrame")
        toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="Filter", background="#F3F6F9", foreground=MUTED).grid(row=0, column=0, padx=(0, 8))
        self.log_filter = tk.StringVar()
        log_filter_entry = ttk.Entry(toolbar, textvariable=self.log_filter, width=38)
        log_filter_entry.grid(row=0, column=1, sticky="w")
        log_filter_entry.bind("<KeyRelease>", lambda _event: self._refresh_log_trees())
        ttk.Button(toolbar, text="CLEAR VIEW", command=self._clear_logs, style="Outline.TButton").grid(row=0, column=2, padx=6)
        ttk.Button(toolbar, text="EXPORT CSV", command=self._export_logs, style="Primary.TButton").grid(row=0, column=3, padx=(6, 0))

        log_card = self._card(body, (0, 0))
        log_card.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        log_card.columnconfigure(0, weight=1)
        log_card.rowconfigure(0, weight=1)
        columns = ("time", "level", "source", "event", "details")
        self.logs_tree = ttk.Treeview(log_card, columns=columns, show="headings")
        headings = {"time": "Time", "level": "Level", "source": "Source", "event": "Event", "details": "Details"}
        widths = {"time": 155, "level": 80, "source": 95, "event": 260, "details": 360}
        for column in columns:
            self.logs_tree.heading(column, text=headings[column])
            self.logs_tree.column(column, width=widths[column], stretch=column in ("event", "details"))
        self.logs_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_card, orient="vertical", command=self.logs_tree.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.logs_tree.configure(yscrollcommand=scrollbar.set)
        self.logs_tree.tag_configure("ERROR", foreground=RED)
        self.logs_tree.tag_configure("WARN", foreground="#A85B00")
        self.logs_tree.tag_configure("INFO", foreground=INK)

        side = self._card(body, 16)
        side.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        side.columnconfigure(1, weight=1)
        ttk.Label(side, text="SESSION SUMMARY", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        self.log_summary = {
            "Records": tk.StringVar(value="0"),
            "Warnings": tk.StringVar(value="0"),
            "Errors": tk.StringVar(value="0"),
            "Telemetry samples": tk.StringVar(value="0"),
        }
        for row, (label, variable) in enumerate(self.log_summary.items(), start=1):
            ttk.Label(side, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=(18 if row == 1 else 8, 4))
            ttk.Label(side, textvariable=variable, style="Card.TLabel").grid(row=row, column=1, sticky="e", pady=(18 if row == 1 else 8, 4))
        ttk.Separator(side).grid(row=5, column=0, columnspan=2, sticky="ew", pady=16)
        ttk.Label(side, text="Alarm reporting is based on P0.001 and P0.046. STO and hardwired EMGS remain independent safety functions.", style="Muted.TLabel", wraplength=280).grid(row=6, column=0, columnspan=2, sticky="w")

    def _build_settings(self, page: ttk.Frame) -> None:
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.settings_canvas = tk.Canvas(page, background="#F3F6F9", borderwidth=0, highlightthickness=0)
        settings_scrollbar = ttk.Scrollbar(page, orient="vertical", command=self.settings_canvas.yview)
        self.settings_canvas.configure(yscrollcommand=settings_scrollbar.set)
        self.settings_canvas.grid(row=0, column=0, sticky="nsew")
        settings_scrollbar.grid(row=0, column=1, sticky="ns")
        body = ttk.Frame(self.settings_canvas, style="App.TFrame")
        settings_window = self.settings_canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind(
            "<Configure>",
            lambda _event: self.settings_canvas.configure(scrollregion=self.settings_canvas.bbox("all")),
        )
        self.settings_canvas.bind(
            "<Configure>",
            lambda event: self.settings_canvas.itemconfigure(settings_window, width=event.width),
        )
        body.columnconfigure((0, 1), weight=1)

        connection = self._card(body, 16)
        connection.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="CONNECTION PROFILE", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(connection, text="Mode", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=(18, 5))
        settings_mode_switch = ttk.Frame(connection, style="Card.TFrame")
        settings_mode_switch.grid(row=1, column=1, sticky="ew", pady=(18, 5))
        settings_mode_switch.columnconfigure((0, 1), weight=1)
        self.settings_simulation_mode = ttk.Button(settings_mode_switch, text="SIMULATION", command=lambda: self._select_connection_mode(OperatingMode.SIMULATION.value), style="Outline.TButton")
        self.settings_simulation_mode.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        self.settings_live_mode = ttk.Button(settings_mode_switch, text="LIVE DRIVE", command=lambda: self._select_connection_mode(OperatingMode.LIVE.value), style="Outline.TButton")
        self.settings_live_mode.grid(row=0, column=1, sticky="ew", padx=(3, 0))

        ttk.Label(connection, text="Serial port", style="Card.TLabel").grid(row=2, column=0, sticky="w", pady=(7, 5))
        port_row = ttk.Frame(connection, style="Card.TFrame")
        port_row.grid(row=2, column=1, sticky="ew", pady=(7, 5))
        port_row.columnconfigure(0, weight=1)
        self.port_combo = ttk.Combobox(port_row, textvariable=self.port, values=("COM6",), state="normal", width=18)
        self.port_combo.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.port_refresh_button = ttk.Button(port_row, text="REFRESH", command=lambda: self._refresh_serial_ports(True), style="Outline.TButton")
        self.port_refresh_button.grid(row=0, column=1)
        fields: list[tuple[str, tk.StringVar, tuple[str, ...]]] = [
            ("Slave address", self.slave, ("1",)),
            ("Baud rate", self.baud, ("9600", "19200", "38400", "57600", "115200")),
            ("Parity", self.parity, ("N", "E", "O")),
            ("Stop bits", self.stopbits, ("1", "2")),
            ("Timeout", self.timeout, ("0.5", "1.0", "2.0")),
        ]
        self.connection_fields: list[ttk.Combobox] = [self.port_combo]
        for row, (label, variable, values) in enumerate(fields, start=3):
            ttk.Label(connection, text=label, style="Card.TLabel").grid(row=row, column=0, sticky="w", pady=(7, 5))
            combo = ttk.Combobox(connection, textvariable=variable, values=values, state="normal", width=18)
            combo.grid(row=row, column=1, sticky="ew", pady=(7, 5))
            self.connection_fields.append(combo)
        self.port_discovery_text = tk.StringVar(value="Refresh to list currently available Windows COM ports; manual entry is allowed.")
        ttk.Label(connection, textvariable=self.port_discovery_text, style="Muted.TLabel", wraplength=520).grid(row=8, column=0, columnspan=2, sticky="w", pady=(12, 4))
        buttons = ttk.Frame(connection, style="Card.TFrame")
        buttons.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        buttons.columnconfigure((0, 1), weight=1)
        self.settings_connect = ttk.Button(buttons, text="CONNECT", command=self._toggle_connection, style="Primary.TButton")
        self.settings_connect.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        ttk.Button(buttons, text="APPLY PROFILE", command=self._apply_connection_profile, style="Outline.TButton").grid(row=0, column=1, sticky="ew", padx=(5, 0))

        simulator = self._card(body, 16)
        simulator.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        simulator.columnconfigure(1, weight=1)
        ttk.Label(simulator, text="SIMULATOR MODEL", style="Section.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")
        sim_fields = [
            ("Feedback noise", self.feedback_noise, "rpm"),
            ("Telemetry sample rate", self.sample_rate, "Hz"),
        ]
        for row, (label, variable, unit) in enumerate(sim_fields, start=1):
            ttk.Label(simulator, text=label, style="Card.TLabel").grid(row=row, column=0, sticky="w", pady=(18 if row == 1 else 8, 5))
            ttk.Entry(simulator, textvariable=variable, justify="right", width=12).grid(row=row, column=1, sticky="ew", pady=(18 if row == 1 else 8, 5), padx=(10, 6))
            ttk.Label(simulator, text=unit, style="Muted.TLabel").grid(row=row, column=2, sticky="w", pady=(18 if row == 1 else 8, 5))
        ttk.Label(simulator, text="These two settings affect only the software simulator and GUI sampling. Motion-page ramps set the simulated P1.034/P1.035 profile; operating limits and motor-to-platform ratio are shared session/rig settings.", style="Muted.TLabel", wraplength=520).grid(row=3, column=0, columnspan=3, sticky="w", pady=(16, 5))
        ttk.Button(simulator, text="APPLY SIMULATOR SETTINGS", command=self._apply_simulation_settings, style="Primary.TButton").grid(row=4, column=0, columnspan=3, sticky="ew", pady=(14, 6))
        ttk.Separator(simulator).grid(row=5, column=0, columnspan=3, sticky="ew", pady=14)
        ttk.Label(simulator, text="LABJACK SCENE DISPLACEMENT — AIN0", style="Section.TLabel").grid(row=6, column=0, columnspan=3, sticky="w")
        for row, (label, variable, unit) in enumerate((
            ("Sample rate", self.labjack_rate, "Hz"),
            ("Voltage at scene 0 mm", self.labjack_zero_voltage, "V"),
            ("Calibration scale", self.labjack_mm_per_volt, "mm/V"),
        ), start=7):
            ttk.Label(simulator, text=label, style="Card.TLabel").grid(row=row, column=0, sticky="w", pady=(10, 4))
            ttk.Entry(simulator, textvariable=variable, justify="right", width=12).grid(row=row, column=1, sticky="ew", padx=(10, 6), pady=(10, 4))
            ttk.Label(simulator, text=unit, style="Muted.TLabel").grid(row=row, column=2, sticky="w", pady=(10, 4))
        ttk.Label(simulator, textvariable=self.labjack_status, style="Muted.TLabel").grid(row=10, column=0, columnspan=3, sticky="w", pady=(8, 4))
        self.labjack_connect_button = ttk.Button(simulator, text="CONNECT LABJACK", command=self._toggle_labjack, style="Primary.TButton")
        self.labjack_connect_button.grid(row=11, column=0, columnspan=3, sticky="ew", pady=(6, 4))
        ttk.Label(simulator, text="Calibration: 3.620 V = 0 mm; 1.671 V = +125 mm. Signed scale is -64.135454 mm/V: height increases as voltage decreases. LabJack uses a separate USB thread, not Modbus.", style="Muted.TLabel", wraplength=520).grid(row=12, column=0, columnspan=3, sticky="w", pady=(6, 4))
        ttk.Separator(simulator).grid(row=13, column=0, columnspan=3, sticky="ew", pady=14)
        ttk.Label(simulator, text="SAVED SETTINGS", style="Section.TLabel").grid(row=14, column=0, columnspan=3, sticky="w")
        ttk.Label(simulator, text="Settings are saved automatically on Apply and when the application closes. Importing never connects to a drive.", style="Muted.TLabel", wraplength=520).grid(row=15, column=0, columnspan=3, sticky="w", pady=(8, 10))
        settings_file_buttons = ttk.Frame(simulator, style="Card.TFrame")
        settings_file_buttons.grid(row=16, column=0, columnspan=3, sticky="ew")
        settings_file_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(settings_file_buttons, text="IMPORT SETTINGS", command=self._import_settings, style="Outline.TButton").grid(row=0, column=0, sticky="ew", padx=(0, 5))
        ttk.Button(settings_file_buttons, text="EXPORT SETTINGS", command=self._export_settings, style="Outline.TButton").grid(row=0, column=1, sticky="ew", padx=(5, 0))

        clock_sync = self._card(body, 16)
        clock_sync.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        clock_sync.columnconfigure(1, weight=1)
        ttk.Label(clock_sync, text="LATTEPANDA UDP CLOCK ALIGNMENT", style="Section.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")
        self.clock_sync_connection_label = tk.Label(
            clock_sync,
            textvariable=self.clock_sync_connection,
            background=SURFACE,
            foreground=MUTED,
            font=("Segoe UI", 10, "bold"),
        )
        self.clock_sync_connection_label.grid(row=0, column=2, sticky="e")
        for row, (label, variable, unit) in enumerate((
            ("LattePanda IP / hostname", self.clock_sync_host, ""),
            ("UDP port", self.clock_sync_port, ""),
            ("Exchange rate", self.clock_sync_rate, "Hz"),
        ), start=1):
            ttk.Label(clock_sync, text=label, style="Card.TLabel").grid(row=row, column=0, sticky="w", pady=(12 if row == 1 else 7, 4))
            ttk.Entry(clock_sync, textvariable=variable, width=24).grid(row=row, column=1, sticky="ew", padx=(12, 6), pady=(12 if row == 1 else 7, 4))
            ttk.Label(clock_sync, text=unit, style="Muted.TLabel").grid(row=row, column=2, sticky="w", pady=(12 if row == 1 else 7, 4))
        ttk.Label(clock_sync, textvariable=self.clock_sync_status, style="Card.TLabel", wraplength=900).grid(row=4, column=0, columnspan=3, sticky="w", pady=(12, 3))
        ttk.Label(clock_sync, textvariable=self.clock_sync_metrics, style="Muted.TLabel", wraplength=900).grid(row=5, column=0, columnspan=3, sticky="w", pady=(2, 5))
        self.clock_sync_button = ttk.Button(clock_sync, textvariable=self.clock_sync_button_text, command=self._toggle_clock_sync, style="Primary.TButton")
        self.clock_sync_button.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(6, 4))
        ttk.Label(
            clock_sync,
            text="Run lattepanda_clock_responder.py on the LattePanda. Exchanges are timestamped with high-resolution monotonic and UTC clocks; profile exports create a separate clock-sync CSV.",
            style="Muted.TLabel",
            wraplength=900,
        ).grid(row=7, column=0, columnspan=3, sticky="w", pady=(6, 2))

        safety = self._card(body, 16)
        safety.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        safety.columnconfigure((0, 1), weight=1)
        ttk.Label(safety, text="SAFETY & LIVE-DRIVE POLICY", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        checks = (
            "Start every connection with a zero software speed request",
            "Simulation motion enabled within the applied range (default ±100 rpm; hard ±1000 rpm)",
            "Live control requires an explicit per-connection enable and healthy drive readback",
            "Configuration writes retain zero-speed and Servo-Off safeguards",
        )
        for row, text in enumerate(checks, start=1):
            variable = tk.BooleanVar(value=True)
            ttk.Checkbutton(safety, text=text, variable=variable, state="disabled").grid(row=row, column=0, sticky="w", pady=(14 if row == 1 else 7, 3))
        ttk.Label(safety, text="The GUI cannot prove STO integrity, EMGS operation, motor/encoder phasing, direction, or mechanical clearance. These require controlled checks at the physical rig.", style="Muted.TLabel", wraplength=760).grid(row=1, column=1, rowspan=4, sticky="nw", padx=(24, 0), pady=(14, 0))
        self._refresh_serial_ports(False)
        self._refresh_connection_mode_buttons()

    @staticmethod
    def _available_serial_ports() -> list[str]:
        discovered: set[str] = set()
        try:
            from serial.tools import list_ports

            discovered.update(port.device for port in list_ports.comports() if port.device)
        except (ImportError, OSError):
            pass
        if not discovered:
            try:
                import winreg

                key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
                index = 0
                while True:
                    try:
                        _name, value, _kind = winreg.EnumValue(key, index)
                    except OSError:
                        break
                    discovered.add(str(value))
                    index += 1
                winreg.CloseKey(key)
            except (ImportError, OSError):
                pass

        def sort_key(port: str) -> tuple[str, int]:
            prefix = port.rstrip("0123456789").upper()
            suffix = port[len(prefix):]
            return prefix, int(suffix) if suffix.isdigit() else 0

        return sorted(discovered, key=sort_key)

    def _refresh_serial_ports(self, notify_user: bool = False) -> None:
        ports = self._available_serial_ports()
        current = self.port.get().strip()
        choices = list(ports)
        if current and current not in choices:
            choices.append(current)
        self.port_combo.configure(values=tuple(choices))
        if ports:
            self.port_discovery_text.set(
                f"Available now: {', '.join(ports)}. Select one or type a COM name manually."
            )
        else:
            self.port_discovery_text.set(
                "No active Windows COM ports were detected. A saved or manually typed COM name can still be used."
            )
        if notify_user:
            self._log("Serial-port list refreshed", "INFO", "CONNECTION", ", ".join(ports) or "No ports detected")

    def _platform_command_sign(self) -> int:
        return 1 if self.platform_command_direction.get() == "Motor positive (+PUU)" else -1

    def _settings_payload(self) -> dict[str, object]:
        return {
            "connection": {
                "mode": self.mode.get(),
                "port": self.port.get().strip(),
                "slave_address": int(self.slave.get()),
                "baudrate": int(self.baud.get()),
                "parity": self.parity.get().strip().upper(),
                "stop_bits": int(self.stopbits.get()),
                "timeout_s": float(self.timeout.get()),
            },
            "motion": {
                "operator_limit_rpm": float(self.speed_limit_magnitude.get()),
                "motor_to_platform_ratio": float(self.platform_ratio.get()),
                "platform_command_sign": self._platform_command_sign(),
                "acceleration_s": float(self.acceleration_s.get()),
                "deceleration_s": float(self.deceleration_s.get()),
            },
            "simulator": {
                "feedback_noise_rpm": float(self.feedback_noise.get()),
                "sample_rate_hz": float(self.sample_rate.get()),
            },
            "labjack": {
                "sample_rate_hz": float(self.labjack_rate.get()),
                "zero_voltage_v": float(self.labjack_zero_voltage.get()),
                "mm_per_volt": (
                    None if not self.labjack_mm_per_volt.get().strip()
                    else float(self.labjack_mm_per_volt.get())
                ),
            },
            "clock_sync": {
                "host": self.clock_sync_host.get().strip(),
                "port": int(self.clock_sync_port.get()),
                "rate_hz": float(self.clock_sync_rate.get()),
            },
            "trends": {
                "integrated_command": self.channel_command.get(),
                "encoder_feedback": self.channel_feedback.get(),
                "tracking_error": self.channel_error.get(),
                "platform_speed": self.channel_platform.get(),
                "motor_position": self.channel_motor_position.get(),
                "platform_position": self.channel_platform_position.get(),
            },
            "test": {
                "type": self.test_type.get(),
                "start_rpm": self.test_start.get(),
                "end_rpm": self.test_end.get(),
                "step_rpm": self.test_step.get(),
                "hold_s": self.test_hold.get(),
                "cycles": self.test_cycles.get(),
                "tolerance_rpm": self.test_tolerance.get(),
                "live_speed_cap_rpm": self.live_test_speed_cap.get(),
            },
        }

    def _apply_settings_payload(self, payload: dict[str, object]) -> None:
        connection = payload.get("connection")
        motion = payload.get("motion")
        simulator = payload.get("simulator")
        trends = payload.get("trends", {})
        test = payload.get("test", {})
        labjack = payload.get("labjack", {})
        clock_sync = payload.get("clock_sync", {})
        if not isinstance(connection, dict) or not isinstance(motion, dict) or not isinstance(simulator, dict):
            raise ValueError("Settings must contain connection, motion, and simulator sections")
        mode = str(connection.get("mode"))
        config = SerialConfig(
            port=str(connection.get("port", "")).strip(),
            slave_address=int(connection.get("slave_address")),
            baudrate=int(connection.get("baudrate")),
            bytesize=8,
            parity=str(connection.get("parity", "")).upper(),
            stopbits=int(connection.get("stop_bits")),
            timeout_s=float(connection.get("timeout_s")),
        )
        config.validate()
        if mode not in (OperatingMode.SIMULATION.value, OperatingMode.LIVE.value):
            raise ValueError("Connection mode must be Simulation or Live Drive")
        limit = float(motion.get("operator_limit_rpm"))
        ratio = float(motion.get("motor_to_platform_ratio"))
        command_sign = int(motion.get("platform_command_sign", 1))
        acceleration = float(motion.get("acceleration_s"))
        deceleration = float(motion.get("deceleration_s"))
        noise = float(simulator.get("feedback_noise_rpm"))
        sample_rate = float(simulator.get("sample_rate_hz"))
        if not 0 < limit <= self.controller.MAX_SPEED_RPM:
            raise ValueError("Operator speed limit must be greater than 0 and no more than 1000 rpm")
        if ratio <= 0 or acceleration <= 0 or deceleration <= 0:
            raise ValueError("Ratio and ramp times must be greater than 0")
        if command_sign not in (-1, 1):
            raise ValueError("Platform command sign must be +1 or -1")
        if not 0 <= noise <= 10 or not 1 <= sample_rate <= 100:
            raise ValueError("Simulator noise must be 0-10 rpm and sample rate 1-100 Hz")
        self.mode.set(mode)
        self.port.set(config.port)
        self.slave.set(str(config.slave_address))
        self.baud.set(str(config.baudrate))
        self.configured_baudrate = config.baudrate
        self.parity.set(config.parity)
        self.stopbits.set(str(config.stopbits))
        self.timeout.set(f"{config.timeout_s:g}")
        self.speed_limit_magnitude.set(f"{limit:g}")
        self.platform_ratio.set(f"{ratio:g}")
        self.platform_command_direction.set(
            "Motor positive (+PUU)" if command_sign == 1 else "Motor negative (-PUU)"
        )
        self.acceleration_s.set(f"{acceleration:g}")
        self.deceleration_s.set(f"{deceleration:g}")
        self.feedback_noise.set(f"{noise:g}")
        self.sample_rate.set(f"{sample_rate:g}")
        self.sim_motion_sample_hz = sample_rate
        self.applied_speed_limit_rpm = limit
        self.applied_min_speed_rpm = -limit
        self.applied_max_speed_rpm = limit
        self.telemetry.platform_ratio = ratio
        self.sim_poll_ms = round(1000.0 / sample_rate)
        if isinstance(labjack, dict):
            self.labjack_rate.set(str(labjack.get("sample_rate_hz", 100)))
            zero, scale = load_calibration(labjack)
            self.labjack_zero_voltage.set(str(zero))
            self.labjack_mm_per_volt.set("" if scale is None else str(scale))
        if isinstance(clock_sync, dict):
            sync_host = str(clock_sync.get("host", "")).strip()
            sync_port = int(clock_sync.get("port", DEFAULT_PORT))
            sync_rate = float(clock_sync.get("rate_hz", 2.0))
            if not 1 <= sync_port <= 65535:
                raise ValueError("UDP clock-sync port must be between 1 and 65535")
            if not 0.2 <= sync_rate <= 20.0:
                raise ValueError("UDP clock exchange rate must be between 0.2 and 20 Hz")
            self.clock_sync_host.set(sync_host)
            self.clock_sync_port.set(str(sync_port))
            self.clock_sync_rate.set(f"{sync_rate:g}")
        if isinstance(trends, dict):
            self.channel_command.set(bool(trends.get("integrated_command", True)))
            self.channel_feedback.set(bool(trends.get("encoder_feedback", True)))
            self.channel_error.set(bool(trends.get("tracking_error", True)))
            self.channel_platform.set(bool(trends.get("platform_speed", False)))
            self.channel_motor_position.set(bool(trends.get("motor_position", True)))
            self.channel_platform_position.set(bool(trends.get("platform_position", True)))
        if isinstance(test, dict):
            self.test_type.set(str(test.get("type", self.test_type.get())))
            self.test_start.set(str(test.get("start_rpm", self.test_start.get())))
            self.test_end.set(str(test.get("end_rpm", self.test_end.get())))
            self.test_step.set(str(test.get("step_rpm", self.test_step.get())))
            self.test_hold.set(str(test.get("hold_s", self.test_hold.get())))
            self.test_cycles.set(str(test.get("cycles", self.test_cycles.get())))
            self.test_tolerance.set(str(test.get("tolerance_rpm", self.test_tolerance.get())))
            self.live_test_speed_cap.set(str(test.get("live_speed_cap_rpm", self.live_test_speed_cap.get())))
            if float(self.test_tolerance.get()) < 0:
                raise ValueError("Test tolerance must be zero or greater")
            live_cap = float(self.live_test_speed_cap.get())
            if not 0 < live_cap <= self.controller.MAX_SPEED_RPM:
                raise ValueError("Live test speed cap must be greater than 0 and no more than 1000 rpm")

    def _refresh_settings_dependent_widgets(self) -> None:
        limit = float(self.speed_limit_magnitude.get())
        ratio = float(self.platform_ratio.get())
        self.applied_speed_limit_rpm = limit
        self.applied_min_speed_rpm = -limit
        self.applied_max_speed_rpm = limit
        self.telemetry.platform_ratio = ratio
        self.dashboard_slider.configure(from_=-limit, to=limit)
        self.motion_slider.configure(from_=-limit, to=limit)
        self.dashboard_gauge.set_range(-limit, limit)
        if hasattr(self, "profile_speed_gauge"):
            self.profile_speed_gauge.set_range(-limit, limit)
        if hasattr(self, "profile_preview"):
            self.profile_preview.set_ramps(float(self.acceleration_s.get()), float(self.deceleration_s.get()), limit)
            if self.motion_profile is not None:
                self.profile_stats_summary.set(
                    f"• {self.motion_profile.source.name}   • {self.motion_profile.duration_s:g} s   "
                    f"• {len(self.motion_profile.points)} points   • peak ±{self.motion_profile.maximum_motor_speed_rpm:.1f} motor rpm   "
                    f"• accel/decel {self.acceleration_s.get()} / {self.deceleration_s.get()} s"
                )
        self.minimum_scale_text.set(f"{-limit:g}")
        self.maximum_scale_text.set(f"{limit:+g} rpm")
        self.channel_platform_label.set(f"Platform speed {ratio:g}:1 (calculated)")
        self.channel_platform_position_label.set(f"Platform sin(θ) {ratio:g}:1 (calculated)")
        for chart in (self.dashboard_trend, self.trends_chart):
            chart.set_platform_ratio(ratio)
        self.position_chart.set_platform_ratio(ratio)
        self._update_trend_channels()
        self._update_position_channels()
        self.dashboard_trend.set_channels(True, True, False, True)
        self._refresh_connection_mode_buttons()
        self._draw_ramp_preview()
        if self.motion_profile is not None:
            self.profile_preview.set_profile(self.motion_profile, self.profile_cursor_s)

    def _save_settings_silent(self) -> None:
        try:
            write_settings(self.settings_path, self._settings_payload())
        except (OSError, TypeError, ValueError) as exc:
            self.settings_load_error = f"Settings save failed: {exc}"

    def _import_settings(self) -> None:
        if self.controller.state is ConnectionState.CONNECTED:
            messagebox.showinfo("Disconnect first", "Disconnect before importing settings.", parent=self.root)
            return
        path = filedialog.askopenfilename(parent=self.root, title="Import Servo Test Rig settings", filetypes=(("JSON settings", "*.json"),))
        if not path:
            return
        try:
            payload = read_settings(path)
            self._apply_settings_payload(payload)
            self._refresh_settings_dependent_widgets()
            self._refresh_serial_ports(False)
            self._save_settings_silent()
        except (OSError, TypeError, ValueError) as exc:
            messagebox.showerror("Import failed", str(exc), parent=self.root)
            return
        self._log("Settings imported", "INFO", "SETTINGS", path)
        messagebox.showinfo("Settings imported", "Settings were validated and applied. The drive was not connected.", parent=self.root)

    def _export_settings(self) -> None:
        path = filedialog.asksaveasfilename(parent=self.root, title="Export Servo Test Rig settings", defaultextension=".json", filetypes=(("JSON settings", "*.json"),), initialfile="servo_test_rig_settings.json")
        if not path:
            return
        try:
            write_settings(path, self._settings_payload())
        except (OSError, TypeError, ValueError) as exc:
            messagebox.showerror("Export failed", str(exc), parent=self.root)
            return
        self._log("Settings exported", "INFO", "SETTINGS", path)

    def _serial_config_from_form(self) -> SerialConfig:
        config = SerialConfig(
            port=self.port.get().strip(),
            slave_address=int(self.slave.get()),
            baudrate=int(self.baud.get()),
            bytesize=8,
            parity=self.parity.get().strip().upper(),
            stopbits=int(self.stopbits.get()),
            timeout_s=float(self.timeout.get()),
        )
        config.validate()
        self.configured_baudrate = config.baudrate
        if self.mode.get() not in (OperatingMode.SIMULATION.value, OperatingMode.LIVE.value):
            raise ValueError("Mode must be Simulation or Live Drive")
        return config

    def _toggle_connection(self) -> None:
        if self.controller.state is ConnectionState.CONNECTED:
            self._abort_test(log=False)
            self.polling = False
            self._stop_motion_polling()
            self.controller.disconnect()
            self.poll_inflight = False
            self.last_snapshot = None
            self.connection_text.set("DISCONNECTED")
            self.connection_detail.set("No active connection")
            self._set_connection_controls(False)
            self._set_disconnected_status()
            self._log("Disconnected", "INFO", "CONNECTION")
            return
        try:
            config = self._serial_config_from_form()
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Invalid connection profile", str(exc), parent=self.root)
            return
        simulated = self.mode.get() == OperatingMode.SIMULATION.value
        self.connection_text.set("CONNECTING")
        self.connection_detail.set("Opening simulator" if simulated else f"Opening {config.port}")
        self._set_connection_controls(True, connecting=True)

        def worker() -> None:
            try:
                self.controller.connect(config, simulated=simulated)
                self.events.put(("connected", (config, simulated)))
            except Exception as exc:
                self.events.put(("connection_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="drive-connect").start()

    def _set_connection_controls(self, connected: bool, connecting: bool = False) -> None:
        if connecting:
            button_text = "CONNECTING..."
        else:
            button_text = "DISCONNECT" if connected else "CONNECT"
        button_state = "disabled" if connecting else "normal"
        for button in (self.dashboard_connect, self.settings_connect):
            button.configure(text=button_text, state=button_state)
        field_state = "disabled" if connected or connecting else "normal"
        for field in self.connection_fields:
            field.configure(state=field_state)
        self.port_refresh_button.configure(state="disabled" if connected or connecting else "normal")
        mode_state = "disabled" if connected or connecting else "normal"
        for button in (
            self.dashboard_simulation_mode,
            self.dashboard_live_mode,
            self.settings_simulation_mode,
            self.settings_live_mode,
        ):
            button.configure(state=mode_state)
        self._refresh_connection_mode_buttons()
        self.parameter_read_button.configure(state="normal" if connected else "disabled")
        self._refresh_parameter_write_state()
        if hasattr(self, "ramp_apply_button"):
            if connected and not connecting:
                ramp_text = "UPDATE SIMULATOR RAMPS" if self.controller.simulated else "APPLY LIVE DRIVE RAMPS"
                self.ramp_apply_button.configure(text=ramp_text, state="normal")
            else:
                self.ramp_apply_button.configure(text="APPLY RAMPS", state="disabled")
        commands_enabled = connected and (
            self.controller.simulated or self.controller.hardware_motion_enabled
        )
        command_state = "normal" if commands_enabled else "disabled"
        for widget in (
            self.dashboard_speed_entry,
            self.motion_speed_entry,
            self.dashboard_minus,
            self.dashboard_plus,
            self.motion_minus,
            self.motion_plus,
            self.dashboard_slider,
            self.motion_slider,
        ):
            widget.configure(state=command_state)
        # A zero-speed request remains available before arming and after an
        # automatic fault disarm.
        stop_state = "normal" if connected and not connecting else "disabled"
        for widget in (self.dashboard_stop, self.motion_stop):
            widget.configure(state=stop_state)
        if not connected:
            self.io_edit_mode = False
        self._refresh_io_editor_state()
        if hasattr(self, "dashboard_position_go") and not self.dashboard_position_active:
            self.dashboard_position_go.configure(state=command_state)
        if hasattr(self, "dashboard_position_home"):
            self.dashboard_position_home.configure(state="normal" if connected and not connecting else "disabled")
        if hasattr(self, "dashboard_live_control"):
            live_connected = connected and not self.controller.simulated and not connecting
            if self.live_manual_arming:
                live_text = "CHECKING LIVE DRIVE..."
            elif self.controller.hardware_motion_enabled:
                live_text = "DISABLE LIVE CONTROL"
            else:
                live_text = "ENABLE LIVE CONTROL"
            self.dashboard_live_control.configure(
                text=live_text,
                state="normal" if live_connected and not self.live_manual_arming else "disabled",
                style="Stop.TButton" if self.controller.hardware_motion_enabled else "Outline.TButton",
            )
        self._update_test_start_state()

    def _toggle_live_manual_control(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED or self.controller.simulated:
            return
        if self.controller.hardware_motion_enabled:
            self.controller.disarm_live_motion(request_stop=True)
            self.live_test_armed = False
            self.armed_live_speed_cap = None
            self._set_connection_controls(True)
            self.connection_detail.set(f"{self.controller.config.port} • live control disabled")
            self._log("Live manual control disabled", "WARN", "MOTION", "Zero speed requested")
            return
        try:
            cap = float(self.applied_speed_limit_rpm)
        except (TypeError, ValueError):
            messagebox.showerror("Invalid speed limit", "Apply a valid operator speed limit first.", parent=self.root)
            return
        if not messagebox.askyesno(
            "Enable live motor control?",
            f"Enable Dashboard and Motion speed controls up to ±{cap:g} rpm for this connection?\n\n"
            "Confirm the motor and encoder are connected, EMGS and STO have been tested, "
            "direction and mechanical clearance are safe, and the controlled stop is available.\n\n"
            "Permission will be removed automatically by an alarm, unhealthy EMGS, "
            "loss of SON/SRDY, disconnect, or application restart.",
            parent=self.root,
        ):
            return
        self.live_manual_arming = True
        self._set_connection_controls(True)

        def worker() -> None:
            try:
                self.controller.arm_live_motion(cap, (True, True, True, True))
                self.events.put(("live_manual_armed", cap))
            except Exception as exc:
                self.events.put(("live_manual_arm_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="live-manual-arm").start()

    def _on_mode_changed(self, _event: tk.Event | None = None) -> None:
        if self.controller.state is not ConnectionState.CONNECTED:
            self._set_disconnected_status()

    def _select_connection_mode(self, mode: str) -> None:
        if self.controller.state is ConnectionState.CONNECTED:
            return
        self.mode.set(mode)
        self._refresh_connection_mode_buttons()
        self._set_disconnected_status()
        self._save_settings_silent()

    def _refresh_connection_mode_buttons(self) -> None:
        if not hasattr(self, "settings_simulation_mode"):
            return
        simulated = self.mode.get() == OperatingMode.SIMULATION.value
        for button in (self.dashboard_simulation_mode, self.settings_simulation_mode):
            button.configure(style="Primary.TButton" if simulated else "Outline.TButton")
        for button in (self.dashboard_live_mode, self.settings_live_mode):
            button.configure(style="Outline.TButton" if simulated else "Primary.TButton")

    def _go_to_motion_io(self) -> None:
        self._show_page("Motion")
        self.motion_io_card.focus_set()
        self.root.after_idle(self._scroll_motion_to_io)

    def _scroll_motion_to_io(self) -> None:
        self.root.update_idletasks()
        scroll_region = self.motion_canvas.bbox("all")
        if scroll_region is None or scroll_region[3] <= 0:
            return
        target_y = self.motion_io_card.winfo_y()
        self.motion_canvas.yview_moveto(max(0.0, min(1.0, target_y / scroll_region[3])))

    @staticmethod
    def _function_label(code: int, catalog: dict[int, str]) -> str:
        return f"0x{code:02X} {catalog.get(code, 'Unknown')}"

    @staticmethod
    def _function_code(label: str) -> int:
        try:
            return int(label.split(maxsplit=1)[0], 16)
        except (ValueError, IndexError) as exc:
            raise ValueError(f"Invalid I/O function selection: {label}") from exc

    def _show_function_menu(
        self,
        button: ttk.Button,
        variable: tk.StringVar,
        catalog: dict[int, str],
    ) -> None:
        menu = tk.Menu(self.root, tearoff=False)
        grouped: dict[int, list[tuple[int, str]]] = {}
        for code, name in catalog.items():
            grouped.setdefault(code >> 4, []).append((code, name))
        for group, entries in sorted(grouped.items()):
            submenu = tk.Menu(menu, tearoff=False)
            for code, name in entries:
                submenu.add_command(
                    label=f"0x{code:02X}  {name}",
                    command=lambda selected=code: self._select_io_function(variable, selected, catalog),
                )
            menu.add_cascade(label=f"0x{group:X}0 – 0x{group:X}F", menu=submenu)
        try:
            menu.tk_popup(button.winfo_rootx(), button.winfo_rooty() + button.winfo_height())
        finally:
            menu.grab_release()

    def _select_io_function(self, variable: tk.StringVar, code: int, catalog: dict[int, str]) -> None:
        variable.set(self._function_label(code, catalog))
        self._mark_io_dirty()

    def _toggle_io_contact(self, variable: tk.StringVar) -> None:
        variable.set("Normally closed" if variable.get() == "Normally open" else "Normally open")
        self._mark_io_dirty()

    def _toggle_di_source(self, channel: int) -> None:
        variable = self.io_di_source_choices[channel]
        function_code = self._function_code(self.io_di_function_choices[channel].get())
        if (
            not self.controller.simulated
            and function_code == self.controller.EMGS_FUNCTION_CODE
            and variable.get() == "Hardware"
        ):
            messagebox.showwarning(
                "EMGS hardware source required",
                f"DI{channel + 1} is currently assigned EMGS. Move EMGS to another hardware-sourced DI before selecting Software here.",
                parent=self.root,
            )
            return
        variable.set("Software" if variable.get() == "Hardware" else "Hardware")
        self._mark_io_dirty()

    def _toggle_di_software_state(self, channel: int) -> None:
        self.io_di_software_states[channel].set(not self.io_di_software_states[channel].get())
        self._mark_io_dirty()

    def _toggle_do_software_state(self, channel: int) -> None:
        self.io_do_software_states[channel].set(not self.io_do_software_states[channel].get())
        self._mark_io_dirty()

    def _sync_io_state_button_texts(self) -> None:
        for state, text_variable in zip(self.io_di_software_states, self.io_di_state_button_texts):
            text_variable.set("TRUE" if state.get() else "FALSE")
        for state, text_variable in zip(self.io_do_software_states, self.io_do_state_button_texts):
            text_variable.set("TRUE" if state.get() else "FALSE")

    def _mark_io_dirty(self, _event: tk.Event | None = None) -> None:
        self.io_source_dirty = True
        self._sync_io_state_button_texts()
        self._update_do_control_texts()
        self._refresh_io_editor_state()

    def _toggle_io_edit(self) -> None:
        if self.io_edit_mode:
            self.io_edit_mode = False
            if self.io_configuration is not None:
                self._load_io_configuration(self.io_configuration)
            self._refresh_io_editor_state()
            return
        if self.controller.state is not ConnectionState.CONNECTED:
            return
        if not self.io_configuration_loaded:
            self._request_io_configuration()
            return
        self.io_edit_mode = True
        self.io_source_dirty = False
        self._refresh_io_editor_state()

    def _refresh_io_editor_state(self) -> None:
        connected = self.controller.state is ConnectionState.CONNECTED
        editable = connected and self.io_edit_mode
        if hasattr(self, "io_edit_button"):
            self.io_edit_button.configure(
                text="CANCEL" if self.io_edit_mode else "EDIT",
                state="normal" if connected and self.io_configuration_loaded else "disabled",
            )
            self.io_apply_button.configure(state="normal" if editable else "disabled")
        for control in (*self.io_di_function_controls, *self.io_di_contact_controls, *self.io_do_function_controls, *self.io_do_contact_controls):
            control.configure(state="normal" if editable else "disabled")
        for control in self.io_di_source_controls:
            control.configure(state="normal" if editable else "disabled")
        for channel, control in enumerate(self.io_di_state_controls, start=1):
            enabled = editable and self.io_di_source_choices[channel - 1].get() == "Software"
            control.configure(state="normal" if enabled else "disabled")
        for index, control in enumerate(self.io_do_state_controls):
            code = self._function_code(self.io_do_function_choices[index].get())
            control.configure(state="normal" if editable and 0x30 <= code <= 0x3F else "disabled")

    def _update_do_control_texts(self) -> None:
        for index, choice in enumerate(self.io_do_function_choices):
            code = self._function_code(choice.get())
            self.io_do_source_texts[index].set(
                f"P4.006 bit {code - 0x30}" if 0x30 <= code <= 0x3F else "Drive function"
            )

    def _request_io_configuration(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED:
            return
        self.io_edit_button.configure(text="READING...", state="disabled")

        def worker() -> None:
            try:
                self.events.put(("io_configuration", self.controller.read_io_configuration()))
            except Exception as exc:
                self.events.put(("io_configuration_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="io-config-read").start()

    def _load_io_configuration(self, configuration: dict[str, object]) -> None:
        di_planning = tuple(configuration["di_planning"])  # type: ignore[arg-type]
        do_planning = tuple(configuration["do_planning"])  # type: ignore[arg-type]
        di_sources = int(configuration["di_sources"])
        di_states = int(configuration["di_software_states"])
        do_states = int(configuration["do_software_states"])
        for index, value in enumerate(di_planning):
            code = value & 0x00FF
            self.io_di_function_choices[index].set(self._function_label(code, self.DI_FUNCTION_CATALOG))
            self.io_di_contact_choices[index].set("Normally open" if value & 0x0100 else "Normally closed")
            self.io_di_source_choices[index].set("Software" if di_sources & (1 << index) else "Hardware")
            self.io_di_software_states[index].set(bool(di_states & (1 << index)))
        for index, value in enumerate(do_planning):
            code = value & 0x00FF
            self.io_do_function_choices[index].set(self._function_label(code, self.DO_FUNCTION_CATALOG))
            self.io_do_contact_choices[index].set("Normally open" if value & 0x0100 else "Normally closed")
            self.io_do_software_states[index].set(bool(do_states & (1 << (code - 0x30))) if 0x30 <= code <= 0x3F else False)
        self.io_configuration = dict(configuration)
        self.io_configuration_loaded = True
        self.io_source_dirty = False
        self._sync_io_state_button_texts()
        self._update_do_control_texts()
        self._refresh_io_editor_state()

    def _apply_io_configuration(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED or not self.io_edit_mode:
            return
        try:
            di_planning = tuple(
                self._function_code(self.io_di_function_choices[index].get())
                | (0x0100 if self.io_di_contact_choices[index].get() == "Normally open" else 0)
                for index in range(13)
            )
            do_planning = tuple(
                self._function_code(self.io_do_function_choices[index].get())
                | (0x0100 if self.io_do_contact_choices[index].get() == "Normally open" else 0)
                for index in range(6)
            )
        except ValueError as exc:
            messagebox.showerror("Invalid I/O setup", str(exc), parent=self.root)
            return
        di_sources = sum(
            1 << index for index, choice in enumerate(self.io_di_source_choices) if choice.get() == "Software"
        )
        di_states = sum(1 << index for index, state in enumerate(self.io_di_software_states) if state.get())
        do_states = int(self.io_configuration["do_software_states"]) if self.io_configuration else 0
        for index, value in enumerate(do_planning):
            code = value & 0x00FF
            if 0x30 <= code <= 0x3F:
                bit = 1 << (code - 0x30)
                do_states = (do_states | bit) if self.io_do_software_states[index].get() else (do_states & ~bit)
        if not self.controller.simulated and not messagebox.askyesno(
            "Apply live I/O configuration?",
            "The drive must be stopped and Servo Off. Exactly one DI must be assigned EMGS as normally closed and hardware-sourced. P2 meaning/contact changes may require a drive restart before they operate normally. Apply the staged values?",
            parent=self.root,
        ):
            return
        self.io_apply_button.configure(text="APPLYING...", state="disabled")

        def worker() -> None:
            try:
                applied = self.controller.apply_io_configuration(
                    di_planning, do_planning, di_sources, di_states, do_states
                )
                self.events.put(("io_configuration_applied", applied))
            except Exception as exc:
                self.events.put(("io_configuration_apply_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="io-config-apply").start()

    def _update_io_status(self, snapshot: DriveSnapshot | None) -> None:
        if snapshot is None:
            self.io_source_dirty = False
            self.status_io.set("DISCONNECTED")
            self.status_io_detail.set("DI ---- (P4.007) • DO ---- (P4.009) • Open Motion page")
            self.io_detail_summary.set("No current readback • Sources: P4.007 and P4.009")
            for value in (*self.io_di_values, *self.io_do_values):
                value.set("-- • unavailable")
            for label in (*self.io_di_state_labels, *self.io_do_state_labels):
                label.configure(foreground=MUTED)
            for lamp in (*self.io_di_lamps, *self.io_do_lamps):
                lamp.configure(foreground="#A9B4C2")
            self._set_status_lamp("DIGITAL I/O", "unknown")
            return

        self.status_io.set(
            f"DI {snapshot.digital_inputs_on_count}/13 ON • "
            f"DO {snapshot.digital_outputs_on_count}/6 ON"
        )
        self.status_io_detail.set(
            f"DI 0x{snapshot.p4_007:04X} (P4.007) • "
            f"DO 0x{snapshot.p4_009:04X} (P4.009) • Open Motion page"
        )
        self.io_detail_summary.set(
            f"P3.006 sources: 0x{snapshot.p3_006:04X} • Combined DI: 0x{snapshot.p4_007:04X} from P4.007 • "
            f"Hardware DO: 0x{snapshot.p4_009:04X} from P4.009"
        )
        if not self.io_source_dirty:
            for channel, choice in enumerate(self.io_di_source_choices, start=1):
                choice.set("Software" if snapshot.digital_input_software_selected(channel) else "Hardware")
        for channel, value in enumerate(self.io_di_values, start=1):
            state = "ON" if snapshot.digital_input_on(channel) else "OFF"
            source = "SW" if snapshot.digital_input_software_selected(channel) else "HW"
            is_emgs = channel in snapshot.emgs_channels
            if is_emgs and snapshot.emgs_normally_closed:
                qualifier = "HEALTHY" if snapshot.emgs_hardware_healthy else "EMGS ACTIVE"
                value.set(f"{state} • {source} • {qualifier}")
                indication_color = GREEN if snapshot.emgs_hardware_healthy else RED
            else:
                value.set(f"{state} • {source}")
                indication_color = GREEN if state == "ON" else MUTED
            if channel <= len(self.io_di_state_labels):
                self.io_di_state_labels[channel - 1].configure(foreground=indication_color)
                self.io_di_lamps[channel - 1].configure(
                    foreground=indication_color if indication_color != MUTED else "#A9B4C2"
                )
        for channel, value in enumerate(self.io_do_values, start=1):
            state = "ON" if snapshot.digital_output_on(channel) else "OFF"
            value.set(f"{state} • output")
            if channel <= len(self.io_do_state_labels):
                self.io_do_state_labels[channel - 1].configure(foreground=GREEN if state == "ON" else MUTED)
                self.io_do_lamps[channel - 1].configure(foreground=GREEN if state == "ON" else "#A9B4C2")
        self._set_status_lamp("DIGITAL I/O", "info")

    def _set_disconnected_status(self) -> None:
        self.session_origin_pulses = None
        self.written_profile = None
        self.compiled_pr_program = None
        self.pr_direction_control = None
        if hasattr(self, "session_origin_text"):
            self.session_origin_text.set("Captured automatically on connection")
        if hasattr(self, "motion_compass"):
            self.motion_compass.set_values(None, None, None)
        if hasattr(self, "profile_speed_gauge"):
            self.profile_speed_gauge.set_values(None, self.speed_scale.get())
        self.pending_setpoint_rpm = None
        self.last_sent_setpoint = None
        self.header_badge.set(f"{self.mode.get().upper()} • DISCONNECTED")
        self.header_badge_label.configure(background="#E9EEF4", foreground=INK)
        self.status_drive.set("DISCONNECTED")
        self.status_drive_detail.set("SRDY -- • SON -- • Source: P0.046")
        self._update_io_status(None)
        self.io_edit_mode = False
        self.io_configuration_loaded = False
        self.io_configuration = None
        self._refresh_io_editor_state()
        self.status_alarm.set("UNKNOWN")
        self.status_alarm_detail.set("Raw: ---- • Source: P0.001")
        self.control_mode_text.set("UNKNOWN")
        self.control_mode_detail.set("Connect for S-P / DI7 mode readback")
        for title in ("DRIVE", "ALARM"):
            self._set_status_lamp(title, "unknown")
        self.feedback_text.set("--")
        self.integrated_text.set("--")
        self.tracking_error_text.set("--")
        self.platform_speed_text.set("--")
        self.motor_position_text.set("--")
        self.platform_position_text.set("--")
        self.motor_sine_text.set("--")
        self.platform_sine_text.set("--")
        self.encoder_position_text.set("--")
        self.motion_quality_text.set(f"Target {self.MOTION_SAMPLE_HZ:g} Hz • disconnected")
        self.direction_text.set("STOPPED")
        self.dashboard_gauge.set_values(None, self.speed_scale.get())
        self.motion_lock_note.set("Connect to Simulation for command testing")
        self._reset_test_preflight(clear_confirmations=True)

    def _schedule_poll(self, delay_ms: int = 0) -> None:
        if not self.polling or self.poll_inflight:
            return

        def launch() -> None:
            if not self.polling or self.poll_inflight:
                return
            # A callback queued while idle can fire after a test has started.
            # Do not let that pending heartbeat interrupt test acquisition.
            if not self.controller.simulated and (self.test_running or self.test_preparing):
                self.snapshot_poll_suspended = True
                return
            self.poll_inflight = True

            def worker() -> None:
                try:
                    snapshot = self.controller.read_snapshot()
                    self.events.put(("snapshot", snapshot))
                except Exception as exc:
                    self.events.put(("poll_error", str(exc)))

            threading.Thread(target=worker, daemon=True, name="drive-poll").start()

        self.root.after(delay_ms, launch)

    def _show_connection_heartbeat(self, healthy: bool, details: str = "") -> None:
        mode = "SIMULATION" if self.controller.simulated else "LIVE DRIVE"
        status = "CONNECTED" if healthy else "NO RESPONSE"
        self.connection_text.set(status)
        self.header_badge.set(f"{mode} • {status}")
        self.header_badge_label.configure(
            background="#DDF5E7" if healthy else "#FBE2E4",
            foreground=GREEN if healthy else RED,
        )
        self.connection_detail.set(
            f"{self.controller.config.port} • status read OK"
            if healthy else f"Status read failed • {details}"
        )

    def _resume_snapshot_polling(self) -> None:
        """Resume dashboard/status polling after a high-rate acquisition run."""
        if not self.snapshot_poll_suspended:
            return
        self.snapshot_poll_suspended = False
        if self.polling and self.controller.state is ConnectionState.CONNECTED:
            self._schedule_poll(50)

    def _test_motion_sample_hz(self) -> float:
        """Return zero for maximum-rate live RTU acquisition."""
        if self.controller.simulated:
            return min(100.0, max(self.MOTION_SAMPLE_HZ, self.sim_motion_sample_hz))
        baud = self.configured_baudrate
        for maximum_baud, rate_hz in self.TEST_SAMPLE_HZ_BY_BAUD:
            if baud <= maximum_baud:
                return rate_hz
        return 0.0

    def _desired_motion_sample_hz(self) -> float:
        active = self.test_running or self.test_preparing or self.dashboard_position_active
        return self._test_motion_sample_hz() if active else self.MOTION_SAMPLE_HZ

    def _event_pump_interval_ms(self) -> int:
        active = self.test_running or self.test_preparing or self.dashboard_position_active
        return 5 if active else 20

    def _position_control_interval_ms(self) -> int:
        effective_hz = self.motion_target_hz if self.motion_target_hz > 0 else 60.0
        return max(5, round(1000.0 / max(1.0, effective_hz)))

    def _update_running_profile_preview(self, profile: MotionProfile, cursor_s: float) -> None:
        """Advance the cursor without rebuilding the Canvas at acquisition rate."""
        self.profile_preview.profile = profile
        self.profile_preview.cursor_s = cursor_s
        now = monotonic()
        if now - self.last_profile_preview_redraw >= 1.0 / self.CHART_REFRESH_HZ:
            self.last_profile_preview_redraw = now
            self.profile_preview.redraw()

    def _start_motion_polling(self) -> None:
        """Start the high-rate P0.010/P0.011 speed/encoder acquisition loop."""
        # Generation tags let the UI discard samples from a previous worker after
        # restart. The pending-command event yields the bus after the current read;
        # it does not interrupt an RTU transaction already in progress.
        self._stop_motion_polling()
        self.motion_poll_generation += 1
        generation = self.motion_poll_generation
        stop_event = threading.Event()
        self.motion_stop_event = stop_event
        self.motion_polling = True
        self.motion_sample_times.clear()
        self.motion_late_samples = 0
        self.motion_target_hz = self._desired_motion_sample_hz()
        target_label = (
            "Bus maximum" if self.motion_target_hz <= 0 else f"Target {self.motion_target_hz:g} Hz"
        )
        self.motion_quality_text.set(f"{target_label} • acquiring")

        def worker() -> None:
            next_deadline = monotonic()
            while not stop_event.is_set():
                target_hz = self._desired_motion_sample_hz()
                self.motion_target_hz = target_hz
                period_s = 0.0 if target_hz <= 0 else 1.0 / target_hz
                while self.motion_command_pending.is_set() and not stop_event.is_set():
                    stop_event.wait(0.0005)
                try:
                    motion = self.controller.read_motion_telemetry()
                    self.events.put(("motion_sample", (generation, motion)))
                except Exception as exc:
                    if not stop_event.is_set():
                        self.events.put(("motion_poll_error", (generation, str(exc))))
                    if stop_event.wait(0.5):
                        break
                    next_deadline = monotonic()
                    continue
                if period_s <= 0:
                    # No timer delay: saturate the bus with consecutive reads.
                    # Commands at the final deadline remain protected because
                    # the stop path halts and joins this worker before writing.
                    next_deadline = monotonic()
                    continue
                next_deadline += period_s
                remaining = next_deadline - monotonic()
                if remaining < -period_s:
                    next_deadline = monotonic()
                    remaining = 0.0
                if stop_event.wait(max(0.0, remaining)):
                    break

        thread = threading.Thread(target=worker, daemon=True, name="motion-telemetry-fast")
        self.motion_poll_thread = thread
        thread.start()

    def _stop_motion_polling(self, *, wait: bool = False) -> None:
        self.motion_stop_event.set()
        self.motion_polling = False
        thread = self.motion_poll_thread
        if (
            wait
            and thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=max(1.0, self.controller.config.timeout_s + 0.5))
            if thread.is_alive():
                raise RuntimeError("Timed out waiting for telemetry to release the drive bus")

    def _run_command(self, label: str, command: Callable[[], None]) -> None:
        def worker() -> None:
            try:
                command()
                self.events.put(("command_done", label))
            except Exception as exc:
                self.events.put(("command_error", (label, str(exc))))

        threading.Thread(target=worker, daemon=True, name="drive-command").start()

    def _process_events(self) -> None:
        # This is the worker-to-Tk handoff. Keep widget access here/on Tk callbacks;
        # adding blocking bus or network calls here can delay display and position
        # control even though the dedicated speed deadline thread remains separate.
        try:
            # Never wait for a continuously-fed queue to become completely
            # empty. During a high-rate test that could keep this callback in
            # the loop indefinitely and starve Tk/Windows of paint and input
            # messages, making the application appear to have crashed.
            processed = 0
            while processed < self.MAX_EVENTS_PER_PUMP:
                event, payload = self.events.get_nowait()
                processed += 1
                if event == "connected":
                    config, simulated = payload  # type: ignore[misc]
                    self.written_profile = None
                    self.compiled_pr_program = None
                    self.pr_direction_control = None
                    self.session_origin_pulses = None
                    self.session_origin_text.set("Capturing encoder position as the connection-time session origin…")
                    self.connection_text.set("CONNECTED")
                    if simulated:
                        self.connection_detail.set("Software plant model • no physical motion")
                    else:
                        self.connection_detail.set(f"{config.port} • slave {config.slave_address} • live control disabled")
                    self._set_connection_controls(True)
                    self.header_badge.set(f"{'SIMULATION' if simulated else 'LIVE DRIVE'} • CONNECTED")
                    self.header_badge_label.configure(background="#DDF5E7", foreground=GREEN)
                    self.motion_lock_note.set("Simulation commands enabled" if simulated else "Use ENABLE LIVE CONTROL on the Dashboard for manual motion")
                    self.polling = True
                    self.snapshot_poll_suspended = False
                    self._start_motion_polling()
                    self._log("Connected", "INFO", "CONNECTION", self.connection_detail.get())
                    self._request_io_configuration()
                    self._schedule_poll()
                elif event == "connection_error":
                    self.polling = False
                    self._stop_motion_polling()
                    self.connection_text.set("FAULT")
                    self.connection_detail.set(str(payload))
                    self.header_badge.set(f"{self.mode.get().upper()} • CONNECTION FAULT")
                    self.header_badge_label.configure(background="#FBE2E4", foreground=RED)
                    self._set_connection_controls(False)
                    self._reset_test_preflight(clear_confirmations=True)
                    self._log("Connection failed", "ERROR", "CONNECTION", str(payload))
                elif event == "snapshot":
                    self.poll_inflight = False
                    if self.controller.state is not ConnectionState.CONNECTED or not self.polling:
                        self._set_disconnected_status()
                        continue
                    self._show_connection_heartbeat(True)
                    self._show_snapshot(payload)  # type: ignore[arg-type]
                    # A full live snapshot comprises many separate Modbus
                    # transactions and previously interrupted the encoder stream
                    # by about 200 ms every two seconds. Suspend dashboard
                    # snapshots during an active test/profile; the dedicated
                    # speed/position transaction and drive-side protections keep
                    # running. Completion/abort explicitly resumes snapshots.
                    if not (
                        not self.controller.simulated
                        and (self.test_running or self.test_preparing)
                    ):
                        self.snapshot_poll_suspended = False
                        interval = self.sim_poll_ms if self.controller.simulated else self.LIVE_STATUS_HEARTBEAT_MS
                        self._schedule_poll(interval)
                    else:
                        self.snapshot_poll_suspended = True
                elif event == "motion_sample":
                    generation, motion = payload  # type: ignore[misc]
                    if (
                        int(generation) == self.motion_poll_generation
                        and self.motion_polling
                        and self.controller.state is ConnectionState.CONNECTED
                    ):
                        self._show_motion_telemetry(motion)  # type: ignore[arg-type]
                elif event == "motion_poll_error":
                    generation, details = payload  # type: ignore[misc]
                    if int(generation) == self.motion_poll_generation and self.motion_polling:
                        self.motion_quality_text.set(f"Motion telemetry retrying • {details}")
                        self._log("High-rate motion read failed", "ERROR", "MODBUS", str(details))
                elif event == "poll_error":
                    self.poll_inflight = False
                    if self.controller.state is not ConnectionState.CONNECTED or not self.polling:
                        self._set_disconnected_status()
                        continue
                    self._show_connection_heartbeat(False, str(payload))
                    self.status_drive.set("READBACK FAULT")
                    self.status_drive_detail.set("SRDY -- • SON -- • Source: P0.046")
                    self.status_io.set("READBACK FAULT")
                    self.status_io_detail.set("DI ---- (P4.007) • DO ---- (P4.009) • Open Motion page")
                    self.io_detail_summary.set("Readback unavailable • Sources: P4.007 and P4.009")
                    for value in (*self.io_di_values, *self.io_do_values):
                        value.set("-- • readback fault")
                    self.status_alarm.set("READBACK FAULT")
                    self.status_alarm_detail.set("Raw: ---- • Source: P0.001")
                    for title in ("DRIVE", "DIGITAL I/O", "ALARM"):
                        self._set_status_lamp(title, "fault")
                    if self.test_running:
                        self._fail_test("Drive readback was lost during the sequence")
                    else:
                        self._reset_test_preflight(clear_confirmations=False)
                    self._log("Readback failed", "ERROR", "MODBUS", str(payload))
                    self._schedule_poll(1000 if self.controller.simulated else self.LIVE_STATUS_HEARTBEAT_MS)
                elif event == "io_configuration":
                    self._load_io_configuration(payload)  # type: ignore[arg-type]
                    self._log("I/O configuration read", "INFO", "DIGITAL I/O", "P2 planning, P3.006 and P4.006/P4.007")
                elif event == "io_configuration_error":
                    self.io_configuration_loaded = False
                    self._refresh_io_editor_state()
                    self._log("I/O configuration read failed", "ERROR", "DIGITAL I/O", str(payload))
                elif event == "io_configuration_applied":
                    self.io_edit_mode = False
                    self._load_io_configuration(payload)  # type: ignore[arg-type]
                    self.io_apply_button.configure(text="APPLY")
                    self.motion_history_tree.insert("", 0, values=(datetime.now().strftime("%H:%M:%S.%f")[:-3], "Digital I/O configuration", "Applied"))
                    self._log("I/O configuration applied", "INFO", "DIGITAL I/O", "P2/P3/P4 values accepted by transport")
                    if not self.controller.simulated:
                        messagebox.showinfo("Restart required", "Restart the drive before relying on changed P2 I/O meaning/contact assignments.", parent=self.root)
                elif event == "io_configuration_apply_error":
                    self.io_apply_button.configure(text="APPLY")
                    self._refresh_io_editor_state()
                    self._log("I/O configuration rejected", "ERROR", "DIGITAL I/O", str(payload))
                    messagebox.showerror("I/O configuration rejected", str(payload), parent=self.root)
                elif event == "profile_pr_written":
                    program, direction_control, write_count = payload  # type: ignore[misc]
                    self.compiled_pr_program = program  # type: ignore[assignment]
                    self.pr_direction_control = int(direction_control)
                    self.written_profile = self.compiled_profile
                    direction = "CCW" if self.pr_direction_control == 0 else "CW"
                    self.profile_validation_text.set(
                        f"WRITTEN + VERIFIED • {len(program.paths)} PR paths • {write_count} register values • motor + is {direction}"
                    )
                    self.profile_workflow_text.set("✓  IMPORT   →   ✓  COMPILED   →   ✓  WRITTEN / VERIFIED   →   4  START")
                    self._update_test_start_state()
                    self._log(
                        "Live PR program written and verified", "WARN", "PROFILE",
                        f"{len(program.paths)} paths; E-gear {program.egear_numerator}/{program.egear_denominator}; P1.001.Z={self.pr_direction_control}",
                    )
                    messagebox.showinfo(
                        "PR program verified",
                        "The PR program was backed up, written and read back successfully. Turn the SON selector ON before starting. The backup remains available for this connection.",
                        parent=self.root,
                    )
                elif event == "profile_pr_write_error":
                    self.compiled_pr_program = None
                    self.written_profile = None
                    self.pr_direction_control = None
                    self.profile_validation_text.set(f"WRITE FAILED • {payload}")
                    self._update_test_start_state()
                    self._log("Live PR write rejected", "ERROR", "PROFILE", str(payload))
                    messagebox.showerror("PR write failed", str(payload), parent=self.root)
                elif event == "profile_pr_armed":
                    self._schedule_armed_profile(payload, pr_mode=True)  # type: ignore[arg-type]
                elif event == "profile_speed_armed":
                    self._schedule_armed_profile(payload, pr_mode=False)  # type: ignore[arg-type]
                elif event == "profile_pr_started":
                    self._begin_live_pr_execution()
                elif event == "profile_pr_start_error":
                    self._finish_lattepanda_profile(completed=False)
                    self.test_preparing = False
                    self.test_running = False
                    self.test_status.set("NOT STARTED")
                    self.profile_validation_text.set(f"START REJECTED • {payload}")
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Live PR start rejected", "ERROR", "PROFILE", str(payload))
                    messagebox.showerror("Live PR not started", str(payload), parent=self.root)
                elif event == "profile_speed_start_error":
                    self._finish_lattepanda_profile(completed=False)
                    self.test_preparing = False
                    self.test_running = False
                    self.test_status.set("NOT STARTED")
                    self.profile_validation_text.set(f"SPEED START REJECTED • {payload}")
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Live Speed CSV start rejected", "ERROR", "PROFILE", str(payload))
                    messagebox.showerror("Live Speed CSV not started", str(payload), parent=self.root)
                elif event == "profile_speed_command_completed":
                    if self.profile_recording_active:
                        point_time_s, command_rpm = payload  # type: ignore[misc]
                        self.test_status.set(
                            f"RUNNING • CSV t={float(point_time_s):g} s • "
                            f"command {float(command_rpm):+.1f} rpm"
                        )
                elif event == "profile_speed_timeline_error":
                    if self.profile_recording_active:
                        self._abort_motion_profile(str(payload))
                elif event == "profile_safety_stop_completed":
                    if self.profile_recording_active:
                        if self.profile_run_after_id is not None:
                            try:
                                self.root.after_cancel(self.profile_run_after_id)
                            except tk.TclError:
                                pass
                            self.profile_run_after_id = None
                        self.profile_stopping_started_at = float(payload)
                        if not self.motion_polling:
                            self._start_motion_polling()
                        self._run_speed_profile_stop_tick()
                elif event == "profile_safety_stop_error":
                    if self.test_running or self.test_preparing:
                        self._abort_motion_profile(f"Independent 30-second stop failed: {payload}")
                elif event == "profile_coordinator_error":
                    self._log("LattePanda coordinator update failed", "ERROR", "SYNC", str(payload))
                    self._auto_export_profile_run()
                elif event == "profile_coordinator_health_error":
                    reason = str(payload)
                    self._log("LattePanda run health failed", "ERROR", "SYNC", reason)
                    if self.test_running or self.test_preparing or self.profile_recording_active:
                        self._abort_motion_profile(reason)
                elif event == "profile_coordinator_finished":
                    response = payload  # type: ignore[assignment]
                    self.profile_remote_metadata = response if isinstance(response, dict) else {}
                    remote_events = response.get("events", []) if isinstance(response, dict) else []
                    self.profile_remote_events = [
                        item for item in remote_events if isinstance(item, dict)
                    ]
                    self._auto_export_profile_run()
                elif event == "profile_home_armed":
                    self._run_manual_home_tick()
                elif event == "profile_home_step":
                    if self.profile_manual_home and self.test_preparing:
                        self.profile_run_after_id = self.root.after(100, self._run_manual_home_tick)
                elif event == "profile_home_complete":
                    self.profile_manual_home = False
                    self.test_preparing = False
                    self.profile_run_after_id = None
                    self.test_status.set("HOME • 0°")
                    self.dashboard_position_command_status.set(f"Home reached • error {float(payload):+.3f}°")
                    self.profile_validation_text.set(
                        f"HOME COMPLETE • platform error {float(payload):+.3f}° • motor stopped"
                    )
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Manual home complete", "INFO", "PROFILE", f"Platform error {float(payload):+.3f}°")
                elif event == "profile_home_error":
                    self.profile_manual_home = False
                    self.test_preparing = False
                    self.profile_run_after_id = None
                    try:
                        self.controller.controlled_stop()
                        if not self.controller.simulated:
                            self.controller.disarm_live_motion(request_stop=False)
                    except Exception:
                        pass
                    self.test_status.set("HOME FAILED")
                    self.dashboard_position_command_status.set(f"Home failed • {payload}")
                    self.profile_validation_text.set(f"HOME FAILED • {payload}")
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Manual home failed", "ERROR", "PROFILE", str(payload))
                    messagebox.showerror("Home failed", str(payload), parent=self.root)
                elif event == "profile_pr_status":
                    self._handle_live_pr_status(payload)  # type: ignore[arg-type]
                elif event == "profile_pr_status_error":
                    if self.test_running:
                        self._abort_motion_profile(f"PR status read failed: {payload}")
                elif event == "command_done":
                    self._log(str(payload), "INFO", "MOTION", "Accepted by transport")
                elif event == "command_error":
                    label, details = payload  # type: ignore[misc]
                    self.pending_setpoint_rpm = None
                    if self.last_snapshot is not None:
                        acknowledged = self.last_snapshot.requested_speed_rpm
                        self._suppress_slider = True
                        self.speed_scale.set(acknowledged)
                        self._suppress_slider = False
                        self.speed_entry.set(f"{acknowledged:.1f}")
                        self.dashboard_gauge.set_values(self.last_snapshot.feedback_speed_rpm, acknowledged)
                    self._log(str(label), "ERROR", "MOTION", str(details))
                    if not self.test_running:
                        messagebox.showerror("Command rejected", str(details), parent=self.root)
                elif event == "live_manual_armed":
                    self.live_manual_arming = False
                    self.connection_detail.set(
                        f"{self.controller.config.port} • live control enabled • cap ±{float(payload):g} rpm"
                    )
                    self.motion_lock_note.set(f"Live manual control enabled • cap ±{float(payload):g} rpm")
                    self._set_connection_controls(True)
                    self._log("Live manual control enabled", "WARN", "MOTION", f"Session cap ±{float(payload):g} rpm")
                elif event == "live_manual_arm_error":
                    self.live_manual_arming = False
                    self._set_connection_controls(True)
                    self._log("Live manual control rejected", "ERROR", "MOTION", str(payload))
                    messagebox.showerror("Live control not enabled", str(payload), parent=self.root)
                elif event == "live_test_armed":
                    self.live_test_arming = False
                    self.live_test_armed = True
                    self.armed_live_speed_cap = float(payload)
                    self.live_test_arm_status.set(f"ARMED for this connection • cap ±{float(payload):g} rpm")
                    self._set_connection_controls(True)
                    self._update_test_start_state()
                    self._log("Live test sequence armed", "WARN", "TEST", f"Session cap ±{float(payload):g} rpm")
                elif event == "live_test_arm_error":
                    self.live_test_arming = False
                    self.live_test_armed = False
                    self.armed_live_speed_cap = None
                    self.live_test_arm_status.set("Arming rejected")
                    self._set_connection_controls(True)
                    self._update_test_start_state()
                    self._log("Live test arming rejected", "ERROR", "TEST", str(payload))
                    messagebox.showerror("Live test not armed", str(payload), parent=self.root)
                elif event == "test_prepared":
                    values, hold, tolerance = payload  # type: ignore[misc]
                    self._begin_test(list(values), float(hold), float(tolerance))
                elif event == "test_prepare_error":
                    self.test_preparing = False
                    self.live_test_armed = False
                    self.armed_live_speed_cap = None
                    self.test_status.set("Not started")
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Test preparation rejected", "ERROR", "TEST", str(payload))
                    messagebox.showerror("Test not started", str(payload), parent=self.root)
                elif event == "test_step_commanded":
                    speed, index = payload  # type: ignore[misc]
                    if self.test_running and int(index) == self.test_index:
                        self.motion_history_tree.insert(
                            "", 0,
                            values=(datetime.now().strftime("%H:%M:%S.%f")[:-3], f"Test speed {float(speed):+.1f} rpm", "Sent"),
                        )
                        self.test_step_started_at = monotonic()
                        self.test_step_sample_count = len(self.telemetry.samples)
                        self.test_status.set(
                            f"Holding step {self.test_index + 1}/{len(self.test_values)} at {float(speed):+.1f} rpm"
                        )
                        hold_ms = max(100, round(self.test_hold_seconds * 1000.0))
                        self.test_after_id = self.root.after(hold_ms, self._complete_test_step)
                elif event == "test_step_error":
                    self._fail_test(f"Speed command failed: {payload}")
                elif event == "test_motion_stopped":
                    self.live_test_armed = False
                    self.armed_live_speed_cap = None
                    self.live_test_arming = False
                    self._set_connection_controls(self.controller.state is ConnectionState.CONNECTED)
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Test motion stopped", "INFO", "TEST", f"{payload}; zero speed requested")
                elif event == "test_stop_error":
                    outcome, details = payload  # type: ignore[misc]
                    self.live_test_armed = False
                    self.armed_live_speed_cap = None
                    self._set_connection_controls(self.controller.state is ConnectionState.CONNECTED)
                    self._update_test_start_state()
                    self._resume_snapshot_polling()
                    self._log("Test stop command failed", "ERROR", "TEST", f"{outcome}: {details}")
                    messagebox.showerror("Stop command failed", str(details), parent=self.root)
                elif event == "parameters":
                    self.parameter_values = payload  # type: ignore[assignment]
                    self.parameter_read_button.configure(state="normal")
                    self._populate_parameter_table()
                    self._log("Project parameter set read", "INFO", "PARAMETERS", f"{len(self.parameter_values)} values")
                elif event == "parameter_error":
                    self.parameter_read_button.configure(state="normal")
                    self._log("Parameter read failed", "ERROR", "PARAMETERS", str(payload))
                elif event == "parameter_written":
                    parameter, value, simulated = payload  # type: ignore[misc]
                    self.parameter_values[str(parameter)] = int(value)
                    self.parameter_write_button.configure(text=self._parameter_write_button_label())
                    self._populate_parameter_table()
                    self._show_parameter_details()
                    self._refresh_parameter_write_state()
                    self._log(
                        f"{'Simulation' if simulated else 'Live'} parameter written and verified",
                        "INFO" if simulated else "WARN",
                        "PARAMETERS",
                        f"{parameter} = {value}",
                    )
                elif event == "parameter_write_error":
                    self.parameter_write_button.configure(text=self._parameter_write_button_label())
                    self._refresh_parameter_write_state()
                    self._log("Parameter write rejected", "ERROR", "PARAMETERS", str(payload))
                    messagebox.showerror("Parameter write rejected", str(payload), parent=self.root)
                elif event == "labjack_connected":
                    rate, scale = payload  # type: ignore[misc]
                    self.labjack_status.set(
                        f"CONNECTED • {self.labjack.backend} • AIN0 at {float(rate):g} Hz • "
                        + ("raw volts only" if scale is None else f"{float(scale):g} mm/V")
                    )
                    self._set_labjack_buttons("DISCONNECT LABJACK", "normal")
                    self._save_settings_silent()
                    self._log("LabJack connected", "INFO", "LABJACK", self.labjack_status.get())
                elif event == "labjack_connection_error":
                    self.labjack_status.set(f"ERROR • {payload}")
                    self._set_labjack_buttons("CONNECT LABJACK", "normal")
                    self._log("LabJack connection failed", "ERROR", "LABJACK", str(payload))
                    messagebox.showerror("LabJack connection failed", str(payload), parent=self.root)
                elif event == "clock_sync_sample":
                    sample = payload
                    if isinstance(sample, ClockSyncSample):
                        best = self.clock_sync.best_recent_sample()
                        best_rtt = sample.round_trip_ns if best is None else best.round_trip_ns
                        best_offset = sample.remote_minus_local_ns if best is None else best.remote_minus_local_ns
                        sample_count = len(self.clock_sync.samples())
                        self.clock_sync_connection.set("● CONNECTED")
                        self.clock_sync_connection_label.configure(foreground=GREEN)
                        self.clock_sync_status.set(
                            f"Receiving replies from {self.clock_sync.resolved_ip}:{self.clock_sync.port}"
                        )
                        self.clock_sync_metrics.set(
                            f"{sample_count} exchanges • recent-best RTT "
                            f"{best_rtt / 1_000_000.0:.3f} ms • "
                            f"LattePanda−PC offset {best_offset / 1_000_000.0:+.3f} ms • "
                            f"last response {datetime.now():%H:%M:%S.%f}"[:-3]
                        )
                elif event == "clock_sync_error":
                    if self.clock_sync.running:
                        self.clock_sync_connection.set("● WAITING")
                        self.clock_sync_connection_label.configure(foreground=ORANGE)
                        self.clock_sync_status.set(f"WAITING / RETRYING • {payload}")
                        self.clock_sync_metrics.set("No valid response received yet")
                    else:
                        self.clock_sync_connection.set("● ERROR")
                        self.clock_sync_connection_label.configure(foreground=RED)
                        self.clock_sync_status.set(f"ERROR • {payload}")
                        self.clock_sync_button_text.set("START CLOCK SYNC")
                elif event == "dashboard_position_ready":
                    if self.dashboard_position_active:
                        speed_rpm, deceleration_rpm_s = payload  # type: ignore[misc]
                        self.dashboard_position_deceleration_rpm_s = float(deceleration_rpm_s)
                        self._run_dashboard_position_tick(float(speed_rpm))
                elif event == "dashboard_position_error":
                    self._stop_dashboard_position_command(f"Position command failed: {payload}")
        except queue.Empty:
            pass
        except Exception as exc:
            # A rendering or widget-state error must not permanently stop the
            # queue: connection workers may already have completed and posted
            # their result, which would otherwise leave the HMI on CONNECTING.
            try:
                self._log("UI event queue recovered", "ERROR", "SYSTEM", str(exc))
            except Exception:
                pass
        finally:
            try:
                self._refresh_labjack_display()
                self.root.after(self._event_pump_interval_ms(), self._process_events)
            except tk.TclError:
                pass

    def _refresh_labjack_display(self) -> None:
        """Refresh AIN0 independently of servo connection and Modbus polling."""
        sample = self.labjack.latest_before(monotonic())
        if self.labjack.last_error and not self.labjack.connected:
            self.labjack_status.set(f"ERROR • {self.labjack.last_error}")
        if sample is None:
            return
        self.drawstring_position_text.set(
            f"{sample.voltage_v:+.4f} V"
            if sample.extension_mm is None
            else f"{sample.extension_mm:+.2f} mm ({sample.voltage_v:+.4f} V)"
        )
        self.dashboard_displacement_mm_text.set(
            "Calibration unavailable" if sample.extension_mm is None else f"{sample.extension_mm:+.2f} mm"
        )
        self.dashboard_displacement_voltage_text.set(f"{sample.voltage_v:+.4f} V")

    def _show_motion_telemetry(self, motion: MotionTelemetry) -> None:
        """Record and display one timestamped high-rate motion sample."""
        # The analog value is associated from a separate USB history, not sampled
        # simultaneously with Modbus. See latest_before for its early-history fallback.
        if self.session_origin_pulses is None:
            self.session_origin_pulses = motion.encoder_position_pulses
            if hasattr(self, "profile_preview"):
                self.profile_preview.set_position_origin(
                    self.session_origin_pulses * 360.0 / (1 << 24)
                )
            self.session_origin_text.set(
                f"Session 0° captured at encoder {motion.encoder_position_pulses:+d} pulse • camera = North"
            )
        requested = self.controller.cached_speed_command_rpm
        ain0 = self.labjack.latest_before(motion.sampled_monotonic_s)
        sample = self.telemetry.append_motion(
            motion,
            requested,
            None if ain0 is None else ain0.voltage_v,
            None if ain0 is None else ain0.extension_mm,
        )
        if self.profile_recording_active:
            self.profile_recorded_samples.append(sample)
        previous_sample_time = self.motion_sample_times[-1] if self.motion_sample_times else None
        self.motion_sample_times.append(motion.sampled_monotonic_s)
        if previous_sample_time is not None:
            latest_interval = motion.sampled_monotonic_s - previous_sample_time
            target_period = (
                motion.acquisition_duration_s
                if self.motion_target_hz <= 0
                else 1.0 / max(1.0, self.motion_target_hz)
            )
            if latest_interval > target_period * 1.5:
                self.motion_late_samples += 1

        if self.last_snapshot is not None:
            self.last_snapshot = replace(
                self.last_snapshot,
                requested_speed_rpm=requested,
                integrated_command_rpm=motion.integrated_command_rpm,
                feedback_speed_rpm=motion.feedback_speed_rpm,
                encoder_position_pulses=motion.encoder_position_pulses,
            )
        # Keep every acquired sample above for export, position control and
        # safety decisions, but do not repaint multiple Canvas widgets at the
        # Modbus acquisition rate (up to 200 Hz). Tk drawing is intentionally
        # capped independently of data acquisition.
        now = monotonic()
        if now - self.last_motion_ui_refresh < 1.0 / self.UI_TELEMETRY_REFRESH_HZ:
            return
        self.last_motion_ui_refresh = now
        if len(self.motion_sample_times) >= 2:
            sample_times = list(self.motion_sample_times)
            intervals = [
                later - earlier
                for earlier, later in zip(sample_times, sample_times[1:])
                if later > earlier
            ]
            if intervals:
                achieved_hz = 1.0 / (sum(intervals) / len(intervals))
                jitter_ms = pstdev(intervals) * 1000.0 if len(intervals) > 1 else 0.0
                read_ms = motion.acquisition_duration_s * 1000.0
                rate_text = (
                    f"{achieved_hz:.1f} Hz (bus maximum)"
                    if self.motion_target_hz <= 0
                    else f"{achieved_hz:.1f}/{self.motion_target_hz:g} Hz"
                )
                self.motion_quality_text.set(
                    f"{rate_text} • jitter {jitter_ms:.1f} ms • "
                    f"read {read_ms:.1f} ms • late {self.motion_late_samples}"
                )
        feedback = motion.feedback_speed_rpm
        integrated = motion.integrated_command_rpm
        self.feedback_text.set(f"{feedback:+.1f} rpm")
        self.integrated_text.set(f"{integrated:+.1f} rpm")
        self.tracking_error_text.set(f"{integrated - feedback:+.1f} rpm")
        self.platform_speed_text.set(
            "--" if sample.platform_speed_rpm is None else f"{sample.platform_speed_rpm:+.1f} rpm"
        )
        self.motor_position_text.set(
            "--" if sample.motor_position_deg is None else f"{sample.motor_position_deg:+.2f}°"
        )
        self.platform_position_text.set(
            "--" if sample.platform_position_deg is None else f"{sample.platform_position_deg:+.2f}°"
        )
        self.drawstring_position_text.set(
            "--"
            if sample.drawstring_voltage_v is None
            else (
                f"{sample.drawstring_voltage_v:+.4f} V"
                if sample.drawstring_extension_mm is None
                else f"{sample.drawstring_extension_mm:+.2f} mm ({sample.drawstring_voltage_v:+.4f} V)"
            )
        )
        self.motor_sine_text.set(
            "--" if sample.motor_position_deg is None else f"{sin(radians(sample.motor_position_deg)):+.4f}"
        )
        self.platform_sine_text.set(
            "--" if sample.platform_position_deg is None else f"{sin(radians(sample.platform_position_deg)):+.4f}"
        )
        self.encoder_position_text.set(f"{motion.encoder_position_pulses:+d} pulse")
        self.direction_text.set("FORWARD" if feedback > 0.05 else "REVERSE" if feedback < -0.05 else "STOPPED")
        rms = self.telemetry.rms_tracking_error()
        peak = self.telemetry.peak_tracking_error()
        self.rms_error_text.set("--" if rms is None else f"{rms:.2f} rpm")
        self.peak_error_text.set("--" if peak is None else f"{peak:.2f} rpm")
        displayed_setpoint = self.pending_setpoint_rpm if self.pending_setpoint_rpm is not None else requested
        self.dashboard_gauge.set_values(feedback, displayed_setpoint)
        if hasattr(self, "profile_speed_gauge"):
            self.profile_speed_gauge.set_values(feedback, displayed_setpoint)
        if hasattr(self, "motion_compass"):
            motor_angle, platform_angle = self._session_angles()
            target = None
            if self.test_running and self.written_profile is not None and self.written_profile.kind == "position" and self.profile_cursor_s is not None:
                target = interpolate_profile(self.written_profile, self.profile_cursor_s)
            self.motion_compass.set_values(motor_angle, platform_angle, target)
        if hasattr(self, "dashboard_position_compass"):
            motor_angle, platform_angle = self._session_angles()
            if platform_angle is not None:
                platform_angle *= self._platform_command_sign()
            self.dashboard_platform_position_text.set(
                "--" if platform_angle is None else f"{platform_angle:+.2f}° • {platform_angle / 360.0:+.4f} rev"
            )
            self.dashboard_position_compass.set_values(
                motor_angle,
                platform_angle,
                self.dashboard_position_goal_deg if self.dashboard_position_active else None,
            )
        if now - self.last_motion_chart_redraw >= 1.0 / self.CHART_REFRESH_HZ:
            self.last_motion_chart_redraw = now
            self._redraw_charts()
        self.log_summary["Telemetry samples"].set(str(len(self.telemetry.samples)))

    def _show_snapshot(self, snapshot: DriveSnapshot) -> None:
        self.last_snapshot = snapshot
        sample = (
            self.telemetry.samples[-1]
            if self.motion_polling and self.telemetry.samples
            else self.telemetry.append(snapshot)
        )
        feedback = snapshot.feedback_speed_rpm
        integrated = snapshot.integrated_command_rpm
        error = snapshot.tracking_error_rpm
        platform = snapshot.platform_speed_rpm(self.telemetry.platform_ratio)
        motor_position = sample.motor_position_deg
        platform_position = sample.platform_position_deg
        self.feedback_text.set("--" if feedback is None else f"{feedback:+.1f} rpm")
        self.integrated_text.set("--" if integrated is None else f"{integrated:+.1f} rpm")
        self.tracking_error_text.set("--" if error is None else f"{error:+.1f} rpm")
        self.platform_speed_text.set("--" if platform is None else f"{platform:+.1f} rpm")
        self.motor_position_text.set("--" if motor_position is None else f"{motor_position:+.2f}°")
        self.platform_position_text.set("--" if platform_position is None else f"{platform_position:+.2f}°")
        self.drawstring_position_text.set(
            "--"
            if sample.drawstring_voltage_v is None
            else (
                f"{sample.drawstring_voltage_v:+.4f} V"
                if sample.drawstring_extension_mm is None
                else f"{sample.drawstring_extension_mm:+.2f} mm ({sample.drawstring_voltage_v:+.4f} V)"
            )
        )
        self.motor_sine_text.set("--" if motor_position is None else f"{sin(radians(motor_position)):+.4f}")
        self.platform_sine_text.set("--" if platform_position is None else f"{sin(radians(platform_position)):+.4f}")
        self.encoder_position_text.set(
            "--" if snapshot.encoder_position_pulses is None else f"{snapshot.encoder_position_pulses:+d} pulse"
        )
        direction_value = feedback or 0.0
        self.direction_text.set("FORWARD" if direction_value > 0.05 else "REVERSE" if direction_value < -0.05 else "STOPPED")
        rms = self.telemetry.rms_tracking_error()
        peak = self.telemetry.peak_tracking_error()
        self.rms_error_text.set("--" if rms is None else f"{rms:.2f} rpm")
        self.peak_error_text.set("--" if peak is None else f"{peak:.2f} rpm")
        displayed_setpoint = snapshot.requested_speed_rpm
        if self.pending_setpoint_rpm is not None:
            if abs(snapshot.requested_speed_rpm - self.pending_setpoint_rpm) < 0.049:
                self.pending_setpoint_rpm = None
            else:
                displayed_setpoint = self.pending_setpoint_rpm
        focused_widget = self.root.focus_get()
        speed_editors = (
            self.dashboard_speed_entry,
            self.motion_speed_entry,
        )
        if self.pending_setpoint_rpm is None and focused_widget not in speed_editors:
            self.speed_entry.set(f"{snapshot.requested_speed_rpm:.1f}")
            self._suppress_slider = True
            self.speed_scale.set(snapshot.requested_speed_rpm)
            self._suppress_slider = False
        self.dashboard_gauge.set_values(feedback, displayed_setpoint)

        self.status_drive.set("READY" if snapshot.drive_ready_indication else "NOT READY")
        self.status_drive_detail.set(
            f"SRDY {'ON' if snapshot.servo_ready else 'OFF'} • "
            f"SON {'ON' if snapshot.servo_on else 'OFF'} • Source: P0.046"
        )
        self.status_alarm.set("ACTIVE" if snapshot.alarm_active else "CLEAR")
        self.status_alarm_detail.set(f"Raw: 0x{snapshot.alarm_code:04X} • Source: P0.001")
        speed_mode = snapshot.speed_mode_selected
        self.control_mode_text.set("SPEED" if speed_mode else "PR")
        self.control_mode_detail.set(
            f"{'Speed' if speed_mode else 'PR'} selected • S-P / DI7 {'OFF' if speed_mode else 'ON'}"
        )
        self._update_io_status(snapshot)
        self._set_status_lamp("DRIVE", "ok" if snapshot.drive_ready_indication else "fault" if snapshot.alarm_active else "warn")
        self._set_status_lamp("ALARM", "fault" if snapshot.alarm_active else "ok")
        checks = self._current_preflight_checks()
        self.preflight_state = checks
        for label, passed in checks.items():
            self.preflight_labels[label].configure(text=f"{'✓' if passed else '○'}  {label}", foreground=GREEN if passed else MUTED)
        if not snapshot.simulated and self.live_test_armed and not snapshot.hardware_motion_enabled:
            self.live_test_armed = False
            self.armed_live_speed_cap = None
            self.live_test_arm_status.set("Live sequence disarmed by drive state")
            self._log("Live sequence automatically disarmed", "WARN", "TEST", "Drive state no longer met the armed conditions")
        if not snapshot.simulated and not snapshot.hardware_motion_enabled:
            self._set_connection_controls(True)
        self._update_test_start_state()
        if self.test_running:
            self._monitor_running_test(snapshot)
        if snapshot.configuration_issues:
            issue_text = "; ".join(snapshot.configuration_issues)
            self.motion_lock_note.set(issue_text)

        self._redraw_charts()
        self.log_summary["Telemetry samples"].set(str(len(self.telemetry.samples)))

    def _on_speed_scale(self, value: str) -> None:
        if self._suppress_slider:
            return
        speed = round(float(value), 1)
        self.pending_setpoint_rpm = speed
        self.speed_entry.set(f"{speed:.1f}")
        self.dashboard_gauge.set_values(
            self.last_snapshot.feedback_speed_rpm if self.last_snapshot else None,
            speed,
        )
        if self.speed_after_id is not None:
            self.root.after_cancel(self.speed_after_id)
        self.speed_after_id = self.root.after(140, lambda: self._send_speed(speed, silent=True))

    def _nudge_speed(self, delta_rpm: float) -> None:
        try:
            current = float(self.speed_entry.get())
        except ValueError:
            current = self.speed_scale.get()
        speed = round(
            max(
                self.applied_min_speed_rpm,
                min(self.applied_max_speed_rpm, current + delta_rpm),
            ),
            1,
        )
        self.pending_setpoint_rpm = speed
        self.speed_entry.set(f"{speed:.1f}")
        self._suppress_slider = True
        self.speed_scale.set(speed)
        self._suppress_slider = False
        self.dashboard_gauge.set_values(
            self.last_snapshot.feedback_speed_rpm if self.last_snapshot else None,
            speed,
        )
        if self.speed_after_id is not None:
            self.root.after_cancel(self.speed_after_id)
        self.speed_after_id = self.root.after(80, lambda: self._send_speed(speed, silent=True))

    def _send_speed_from_entry(self, silent: bool = False) -> None:
        try:
            speed = float(self.speed_entry.get())
        except ValueError:
            if not silent:
                messagebox.showerror("Invalid speed", "Enter a numeric speed in rpm.", parent=self.root)
            return
        if not self.applied_min_speed_rpm <= speed <= self.applied_max_speed_rpm:
            if not silent:
                messagebox.showerror(
                    "Invalid speed",
                    f"Speed must be between {self.applied_min_speed_rpm:g} and {self.applied_max_speed_rpm:+g} rpm.",
                    parent=self.root,
                )
            return
        self._suppress_slider = True
        self.speed_scale.set(speed)
        self._suppress_slider = False
        self.speed_entry.set(f"{speed:.1f}")
        if self.speed_after_id is not None:
            self.root.after_cancel(self.speed_after_id)
            self.speed_after_id = None
        self._send_speed(speed, silent=silent)

    def _send_speed(self, speed: float, silent: bool = False) -> None:
        self.speed_after_id = None
        if not self.applied_min_speed_rpm <= speed <= self.applied_max_speed_rpm:
            self.pending_setpoint_rpm = None
            self._log(
                "Speed command rejected",
                "WARN",
                "MOTION",
                f"{speed:+.1f} rpm is outside operator limits "
                f"[{self.applied_min_speed_rpm:g}, {self.applied_max_speed_rpm:+g}] rpm",
            )
            if not silent:
                messagebox.showerror("Operator limit", "The requested speed is outside the applied operator limits.", parent=self.root)
            return
        if self.controller.state is not ConnectionState.CONNECTED:
            if not silent:
                messagebox.showinfo("Not connected", "Connect to Simulation before sending commands.", parent=self.root)
            return
        self.pending_setpoint_rpm = speed
        if self.last_sent_setpoint is not None and abs(self.last_sent_setpoint - speed) < 0.049:
            return
        self.last_sent_setpoint = speed
        self._run_command(f"Speed setpoint {speed:+.1f} rpm", lambda: self.controller.command_speed_rpm(speed))
        self.motion_history_tree.insert("", 0, values=(datetime.now().strftime("%H:%M:%S.%f")[:-3], f"Speed {speed:+.1f} rpm", "Sent"))
        children = self.motion_history_tree.get_children()
        for item in children[20:]:
            self.motion_history_tree.delete(item)

    def _controlled_stop(self) -> None:
        self._abort_test(log=False)
        self._suppress_slider = True
        self.speed_scale.set(0.0)
        self._suppress_slider = False
        self.speed_entry.set("0.0")
        self.last_sent_setpoint = None
        self._send_speed(0.0)

    def _apply_operator_limits(self) -> None:
        try:
            magnitude = float(self.speed_limit_magnitude.get())
            ratio = float(self.platform_ratio.get())
        except ValueError:
            messagebox.showerror("Invalid limits", "Speed range and ratio must be numeric.", parent=self.root)
            return
        if not 0 < magnitude <= self.controller.MAX_SPEED_RPM or ratio <= 0.0:
            messagebox.showerror(
                "Invalid limits",
                "Use a symmetric speed range greater than 0 and no more than ±1000 rpm, and set a ratio greater than 0.",
                parent=self.root,
            )
            return
        minimum = -magnitude
        maximum = magnitude
        try:
            current = float(self.speed_entry.get())
        except ValueError:
            messagebox.showerror("Invalid P1.009 command", "Finish editing the numeric speed command before applying limits.", parent=self.root)
            return
        if not minimum <= current <= maximum:
            messagebox.showerror(
                "Command outside new limits",
                "Stop or move P1.009 inside the proposed range before applying narrower limits.",
                parent=self.root,
            )
            return

        self._refresh_settings_dependent_widgets()
        if self.motion_profile is not None:
            self._load_motion_profile_path(str(self.motion_profile.source))
        self._save_settings_silent()
        self._log(
            "Operator limits applied",
            "INFO",
            "SETTINGS",
            f"{minimum:g} to {maximum:+g} rpm • motor-to-platform {ratio:g}:1",
        )

    def _apply_ramps(self) -> None:
        try:
            acceleration = float(self.acceleration_s.get())
            deceleration = float(self.deceleration_s.get())
            acceleration_ms, deceleration_ms = self.controller.ramp_parameter_values(
                acceleration,
                deceleration,
                self.applied_speed_limit_rpm,
            )
        except ValueError:
            messagebox.showerror(
                "Invalid ramps",
                "Use positive numeric ramp times that convert within the P1.034/P1.035 range.",
                parent=self.root,
            )
            return
        self._draw_ramp_preview()
        self.written_profile = None
        self.compiled_pr_program = None
        self.pr_direction_control = None
        if self.motion_profile is not None:
            self._compile_motion_profile()
        self._save_settings_silent()
        if self.controller.state is not ConnectionState.CONNECTED:
            self._log("Ramp settings staged", "INFO", "SETTINGS", "Connect to Simulation or Live Drive to apply")
            return
        if not self.controller.simulated:
            confirmed = messagebox.askyesno(
                "Apply live drive ramps?",
                (
                    f"Write and verify these live drive parameters?\n\n"
                    f"P1.034 acceleration = {acceleration_ms} ms\n"
                    f"P1.035 deceleration = {deceleration_ms} ms\n\n"
                    f"These values give {acceleration:g} s acceleration and {deceleration:g} s "
                    f"deceleration over 0 to {self.applied_speed_limit_rpm:g} rpm.\n\n"
                    "The command must be 0 rpm, the drive must report zero speed, and Servo On must be OFF."
                ),
                parent=self.root,
            )
            if not confirmed:
                return
        mode_label = "Simulator" if self.controller.simulated else "Live drive"
        self._run_command(
            (
                f"{mode_label} ramps {acceleration:g} s / {deceleration:g} s "
                f"(P1.034={acceleration_ms}, P1.035={deceleration_ms})"
            ),
            lambda: self.controller.apply_ramps(
                acceleration,
                deceleration,
                self.applied_speed_limit_rpm,
            ),
        )

    def _apply_simulation_settings(self) -> None:
        try:
            sample_rate = float(self.sample_rate.get())
            noise = float(self.feedback_noise.get())
            if not 1 <= sample_rate <= 100 or not 0 <= noise <= 10:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid simulator settings", "Use sample rate 1-100 Hz and noise 0-10 rpm.", parent=self.root)
            return
        self.sim_poll_ms = round(1000.0 / sample_rate)
        self.sim_motion_sample_hz = sample_rate
        if self.controller.state is ConnectionState.CONNECTED and self.controller.simulated:
            self._run_command(
                f"Simulator feedback noise {noise:g} rpm",
                lambda: self.controller.set_simulation_feedback_noise(noise),
            )
        self._save_settings_silent()
        self._log("Simulation settings applied", "INFO", "SETTINGS", f"{sample_rate:g} Hz, noise {noise:g} rpm")

    def _toggle_clock_sync(self) -> None:
        if self.clock_sync.running:
            self.clock_sync.stop()
            self.clock_sync_status.set("DISCONNECTED")
            self.clock_sync_connection.set("● DISCONNECTED")
            self.clock_sync_connection_label.configure(foreground=MUTED)
            self.clock_sync_metrics.set("No clock samples received")
            self.clock_sync_button_text.set("START CLOCK SYNC")
            self._log("UDP clock sync stopped", "INFO", "SYNC")
            return
        try:
            host = self.clock_sync_host.get().strip()
            port = int(self.clock_sync_port.get())
            rate = float(self.clock_sync_rate.get())
            self.clock_sync.clear()
            self.clock_sync.start(host, port, rate)
        except (OSError, TypeError, ValueError) as exc:
            self.clock_sync_status.set(f"ERROR • {exc}")
            self.clock_sync_connection.set("● ERROR")
            self.clock_sync_connection_label.configure(foreground=RED)
            messagebox.showerror("Clock sync failed", str(exc), parent=self.root)
            return
        self.clock_sync_status.set(f"WAITING • UDP {host}:{port} at {rate:g} Hz")
        self.clock_sync_connection.set("● WAITING")
        self.clock_sync_connection_label.configure(foreground=ORANGE)
        self.clock_sync_metrics.set("Waiting for the first valid clock response")
        self.clock_sync_button_text.set("STOP CLOCK SYNC")
        self._save_settings_silent()
        self._log("UDP clock sync started", "INFO", "SYNC", f"{host}:{port} at {rate:g} Hz")

    def _toggle_labjack(self) -> None:
        if self.labjack.connected:
            self.labjack.stop()
            self.labjack_status.set("DISCONNECTED")
            self._set_labjack_buttons("CONNECT LABJACK", "normal")
            self._log("LabJack disconnected", "INFO", "LABJACK", "AIN0 sampling stopped")
            return
        try:
            rate = float(self.labjack_rate.get())
            zero = float(self.labjack_zero_voltage.get())
            scale_text = self.labjack_mm_per_volt.get().strip()
            scale = None if not scale_text else float(scale_text)
            if not 1 <= rate <= 1000:
                raise ValueError("Sample rate must be between 1 and 1000 Hz")
        except (TypeError, ValueError) as exc:
            self.labjack_status.set(f"ERROR • {exc}")
            messagebox.showerror("LabJack connection failed", str(exc), parent=self.root)
            return
        self.labjack_status.set("CONNECTING • searching for a LabJack device…")
        self._set_labjack_buttons("CONNECTING…", "disabled")

        def worker() -> None:
            try:
                self.labjack.start(
                    sample_rate_hz=rate,
                    zero_voltage_v=zero,
                    mm_per_volt=scale,
                )
                self.events.put(("labjack_connected", (rate, scale)))
            except Exception as exc:
                self.events.put(("labjack_connection_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="labjack-connect").start()

    def _set_labjack_buttons(self, text: str, state: str) -> None:
        for name in ("labjack_connect_button", "dashboard_labjack_connect_button"):
            button = getattr(self, name, None)
            if button is not None:
                button.configure(text=text, state=state)

    def _apply_connection_profile(self) -> None:
        try:
            config = self._serial_config_from_form()
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Invalid connection profile", str(exc), parent=self.root)
            return
        self._save_settings_silent()
        self._log("Connection profile validated", "INFO", "SETTINGS", f"{config.port}, slave {config.slave_address}, {config.baudrate}, 8-{config.parity}-{config.stopbits}")
        messagebox.showinfo("Profile saved", "The connection profile is valid and will be restored at the next application start.", parent=self.root)

    def _draw_ramp_preview(self) -> None:
        limit = self.applied_speed_limit_rpm
        max_acceleration_s = P1_034.maximum / 1000.0 * limit / 3000.0
        max_deceleration_s = P1_035.maximum / 1000.0 * limit / 3000.0
        self.ramp_limits_text.set(
            f"At {limit:g} rpm: max accel {max_acceleration_s:.6g} s; "
            f"max decel {max_deceleration_s:.6g} s"
        )
        canvas = self.ramp_canvas
        canvas.delete("all")
        width = max(260, canvas.winfo_width())
        height = max(130, canvas.winfo_height())
        left, right, top, bottom = 34, width - 18, 18, height - 28
        try:
            acceleration = max(0.1, float(self.acceleration_s.get()))
            deceleration = max(0.1, float(self.deceleration_s.get()))
        except ValueError:
            acceleration = deceleration = 2.0
        hold = 1.0
        total = acceleration + hold + deceleration
        x1 = left + (right - left) * acceleration / total
        x2 = left + (right - left) * (acceleration + hold) / total
        canvas.create_line(left, bottom, right, bottom, fill="#AAB5C2")
        canvas.create_line(left, bottom, x1, top, x2, top, right, bottom, fill=BLUE, width=3)
        canvas.create_text((left + x1) / 2, bottom + 15, text=f"{acceleration:g}s accel", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text((x2 + right) / 2, bottom + 15, text=f"{deceleration:g}s decel", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(left, top, text=f"{self.applied_max_speed_rpm:+g} rpm", anchor="sw", fill=INK, font=("Segoe UI", 8))

    def _generate_test_values(self) -> tuple[list[float], float, float]:
        start = float(self.test_start.get())
        end = float(self.test_end.get())
        step = float(self.test_step.get())
        hold = float(self.test_hold.get())
        cycles = int(self.test_cycles.get())
        tolerance = float(self.test_tolerance.get())
        if step <= 0 or hold <= 0 or cycles < 1 or tolerance < 0:
            raise ValueError("Step and hold must be positive, cycles at least 1, and tolerance zero or greater")
        if not self.applied_min_speed_rpm <= start <= self.applied_max_speed_rpm:
            raise ValueError("Start speed is outside the applied operator limit")
        if not self.applied_min_speed_rpm <= end <= self.applied_max_speed_rpm:
            raise ValueError("End speed is outside the applied operator limit")
        test_type = self.test_type.get()
        if test_type == "Step response":
            values = [start, end] * cycles
        elif test_type == "Direction reversal":
            magnitude = max(abs(start), abs(end), step)
            values = [magnitude, -magnitude] * cycles
        elif test_type == "Repeatability cycle":
            values = [0.0, end, 0.0, start, 0.0] * cycles
        elif test_type == "Speed sweep":
            values = speed_sweep_values(start, end, step, cycles)
        else:
            raise ValueError("Select a supported test type")
        if any(not self.applied_min_speed_rpm <= value <= self.applied_max_speed_rpm for value in values):
            raise ValueError("The generated sequence exceeds the applied operator limit")
        return values, hold, tolerance

    def _draw_sequence_preview(self) -> None:
        canvas = self.sequence_canvas
        canvas.delete("all")
        width = max(360, canvas.winfo_width())
        height = max(160, canvas.winfo_height())
        left, right, top, bottom = 44, width - 16, 22, height - 31
        middle = (top + bottom) / 2
        canvas.create_line(left, middle, right, middle, fill="#AAB5C2")
        try:
            values, _, _ = self._generate_test_values()
        except (TypeError, ValueError) as exc:
            canvas.create_text((left + right) / 2, middle, text=f"Enter valid sequence settings: {exc}", fill=MUTED, font=("Segoe UI", 9))
            return
        shown = values[:40]
        maximum = max(abs(self.applied_min_speed_rpm), abs(self.applied_max_speed_rpm), 1.0)
        coordinates: list[tuple[float, float]] = []
        for index, value in enumerate(shown):
            x = left + index * (right - left) / max(1, len(shown) - 1)
            y = middle - max(-maximum, min(maximum, value)) / (2 * maximum) * (bottom - top)
            coordinates.append((x, y))
        if coordinates:
            points: list[float] = [coordinates[0][0], coordinates[0][1]]
            for previous, current in zip(coordinates, coordinates[1:]):
                points.extend((current[0], previous[1], current[0], current[1]))
            if len(points) >= 4:
                canvas.create_line(*points, fill=BLUE, width=3)
            for x, y in coordinates:
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=BLUE, outline=BLUE)
        canvas.create_text(left, top, text=f"{self.applied_max_speed_rpm:+g}", anchor="w", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(left, bottom, text=f"{self.applied_min_speed_rpm:g}", anchor="w", fill=MUTED, font=("Segoe UI", 8))
        suffix = " (first 40 points)" if len(values) > len(shown) else ""
        canvas.create_text(right, bottom + 18, text=f"{self.test_type.get()} • {len(values)} commands{suffix}", anchor="e", fill=MUTED, font=("Segoe UI", 8))

    def _reset_test_preflight(self, clear_confirmations: bool) -> None:
        self.preflight_state = {}
        self.live_test_armed = False
        self.armed_live_speed_cap = None
        self.live_test_arming = False
        if clear_confirmations:
            for confirmation in self.live_test_confirmations:
                confirmation.set(False)
        if hasattr(self, "preflight_labels"):
            for label, widget in self.preflight_labels.items():
                widget.configure(text=f"○  {label}", foreground=MUTED)
        if hasattr(self, "live_test_arm_status"):
            self.live_test_arm_status.set("Live sequence not armed")
        if hasattr(self, "test_start_button"):
            self._update_test_start_state()

    def _automatic_preflight_ready(self) -> bool:
        checks = self._current_preflight_checks()
        self.preflight_state = checks
        return bool(checks) and all(checks.values())

    def _current_preflight_checks(self) -> dict[str, bool]:
        """Evaluate motion interlocks from the freshest command and encoder data."""
        snapshot = self.last_snapshot
        if snapshot is None:
            return {}
        latest = self.telemetry.samples[-1] if self.telemetry.samples else None
        feedback = (
            latest.encoder_feedback_rpm
            if latest is not None and latest.encoder_feedback_rpm is not None
            else snapshot.feedback_speed_rpm
        )
        encoder_available = (
            latest is not None
            and latest.motor_position_deg is not None
            and snapshot.encoder_position_pulses is not None
        )
        good_monitor = (
            snapshot.monitor_1_selection == 9
            and snapshot.monitor_2_selection == 7
            and snapshot.monitor_3_selection == 3
        )
        command = self.controller.cached_speed_command_rpm
        return {
            "Drive ready and Servo On": snapshot.drive_ready_indication,
            "EMGS hardware input healthy": snapshot.emgs_hardware_healthy,
            "No active alarm": not snapshot.alarm_active,
            "Encoder monitor available": good_monitor and encoder_available and feedback is not None,
            "Zero command and zero speed": (
                abs(command) < 0.05
                and snapshot.zero_speed
                and feedback is not None
                and abs(feedback) < 0.5
            ),
            "Telemetry recording ready": latest is not None and self.polling,
        }

    def _preflight_failure_details(self) -> list[str]:
        snapshot = self.last_snapshot
        if snapshot is None:
            return ["No drive status snapshot has been received yet."]
        checks = self._current_preflight_checks()
        latest = self.telemetry.samples[-1] if self.telemetry.samples else None
        feedback = (
            latest.encoder_feedback_rpm
            if latest is not None and latest.encoder_feedback_rpm is not None
            else snapshot.feedback_speed_rpm
        )
        details: list[str] = []
        if not checks.get("Drive ready and Servo On", False):
            details.append(
                f"Drive state: SRDY {'ON' if snapshot.servo_ready else 'OFF'}, "
                f"SON {'ON' if snapshot.servo_on else 'OFF'}."
            )
        if not checks.get("EMGS hardware input healthy", False):
            channel_text = ", ".join(f"DI{channel}" for channel in snapshot.emgs_channels) or "not configured"
            details.append(f"EMGS interlock is not healthy (channel {channel_text}).")
        if not checks.get("No active alarm", False):
            details.append(f"Drive alarm is active: 0x{snapshot.alarm_code:04X}.")
        if not checks.get("Encoder monitor available", False):
            details.append(
                "Encoder feedback is unavailable or monitor mapping is not P0.017=9, P0.018=7, P0.019=3."
            )
        if not checks.get("Zero command and zero speed", False):
            feedback_text = "unavailable" if feedback is None else f"{feedback:+.2f} rpm"
            details.append(
                f"Motion is not confirmed stopped: command {self.controller.cached_speed_command_rpm:+.1f} rpm, "
                f"feedback {feedback_text}, ZSPD {'ON' if snapshot.zero_speed else 'OFF'}."
            )
        if not checks.get("Telemetry recording ready", False):
            details.append("Live telemetry is not running or no encoder sample has arrived yet.")
        return details

    def _show_preflight_error(self, title: str) -> None:
        details = self._preflight_failure_details()
        message = "Motion remains locked because:\n\n" + "\n".join(f"• {detail}" for detail in details)
        messagebox.showerror(title, message, parent=self.root)

    def _update_test_start_state(self) -> None:
        if not hasattr(self, "test_start_button"):
            return
        connected = self.controller.state is ConnectionState.CONNECTED
        busy = self.test_running or self.test_preparing
        if hasattr(self, "profile_write_button"):
            self.profile_write_button.configure(
                state="normal" if connected and self.compiled_profile is not None and not busy else "disabled"
            )
        if hasattr(self, "profile_home_button"):
            can_home = (
                connected
                and self.session_origin_pulses is not None
                and self._automatic_preflight_ready()
                and not busy
            )
            self.profile_home_button.configure(state="normal" if can_home else "disabled")
        can_start = (
            connected
            and self.written_profile is not None
            and self._automatic_preflight_ready()
            and not busy
            and (
                self.controller.simulated
                or self.compiled_pr_program is not None
                or self.written_profile.kind in ("speed", "position")
            )
        )
        if busy:
            self.test_start_button.configure(text="ABORT", style="Stop.TButton", state="normal")
        else:
            self.test_start_button.configure(
                text=(
                    "START"
                    if self.controller.simulated
                    else (
                        (
                            "START SPEED-CONTROLLED POSITION"
                            if self.written_profile is not None and self.written_profile.kind == "position"
                            else "START LIVE SPEED CSV"
                        )
                        if self.written_profile is not None and self.written_profile.kind in ("speed", "position")
                        else "START LIVE PR"
                    )
                ),
                style="Primary.TButton",
                state="normal" if can_start else "disabled",
            )

    def _arm_live_test(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED or self.controller.simulated:
            return
        try:
            values, _, _ = self._generate_test_values()
            cap = float(self.live_test_speed_cap.get())
            if not 0 < cap <= self.controller.MAX_SPEED_RPM:
                raise ValueError("Live speed cap must be greater than 0 and no more than 1000 rpm")
            if max(abs(value) for value in values) > cap:
                raise ValueError("The generated sequence exceeds the live speed cap")
            confirmations = tuple(variable.get() for variable in self.live_test_confirmations)
            if not all(confirmations):
                raise ValueError("Complete every physical commissioning confirmation before arming")
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Cannot arm live test", str(exc), parent=self.root)
            return
        self.live_test_arming = True
        self.live_test_arm_status.set("Checking live drive parameters and status...")
        self._update_test_start_state()

        def worker() -> None:
            try:
                self.controller.arm_live_motion(cap, confirmations)
                self.events.put(("live_test_armed", cap))
            except Exception as exc:
                self.events.put(("live_test_arm_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="live-test-arm").start()

    def _start_test(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED:
            messagebox.showinfo("Not connected", "Connect to Simulation or Live Drive before running a test.", parent=self.root)
            return
        try:
            values, hold, tolerance = self._generate_test_values()
            if not self._automatic_preflight_ready():
                raise ValueError("Every automatic pre-flight check must pass before the sequence starts")
            live_cap = 0.0
            acceleration_s = 0.0
            deceleration_s = 0.0
            if self.controller.simulated:
                acceleration_s = float(self.acceleration_s.get())
                deceleration_s = float(self.deceleration_s.get())
                if acceleration_s <= 0 or deceleration_s <= 0:
                    raise ValueError("Acceleration and deceleration must be greater than 0")
            else:
                if not self.live_test_armed or not self.controller.hardware_motion_enabled:
                    raise ValueError("Arm the live sequence for this connection before starting")
                if not all(variable.get() for variable in self.live_test_confirmations):
                    raise ValueError("Every physical confirmation must remain selected")
                if self.armed_live_speed_cap is None:
                    raise ValueError("The live speed cap has not been armed")
                live_cap = self.armed_live_speed_cap
                if max(abs(value) for value in values) > live_cap:
                    raise ValueError("The generated sequence exceeds the armed live speed cap")
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Invalid test", str(exc), parent=self.root)
            return
        self.test_preparing = True
        self.test_status.set("Preparing")
        self._update_test_start_state()

        def worker() -> None:
            try:
                if self.controller.simulated:
                    self.controller.set_simulation_ramps(
                        acceleration_s,
                        deceleration_s,
                        self.applied_speed_limit_rpm,
                    )
                else:
                    issues = self.controller.live_motion_readiness_issues(live_cap)
                    if issues:
                        raise RuntimeError("; ".join(issues))
                self.events.put(("test_prepared", (values, hold, tolerance)))
            except Exception as exc:
                if not self.controller.simulated:
                    try:
                        self.controller.disarm_live_motion(request_stop=True)
                    except Exception:
                        pass
                self.events.put(("test_prepare_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="test-prepare").start()

    def _begin_test(self, values: list[float], hold: float, tolerance: float) -> None:
        self.test_preparing = False
        self.test_running = True
        self.test_values = values
        self.test_hold_seconds = hold
        self.test_tolerance_rpm = tolerance
        self.test_index = 0
        self.test_current_target = None
        self.test_failures.clear()
        self.test_status.set("Running")
        self.test_completed.set(f"0 / {len(values)}")
        self.test_step_result.set("--")
        self.telemetry.clear()
        self.labjack.clear()
        self._update_test_start_state()
        self._log(
            f"{self.test_type.get()} started",
            "WARN" if not self.controller.simulated else "INFO",
            "TEST",
            f"{len(values)} commands • hold {hold:g} s • tolerance ±{tolerance:g} rpm",
        )
        self._advance_test()

    def _advance_test(self) -> None:
        if not self.test_running:
            return
        if self.test_index >= len(self.test_values):
            self._finish_test()
            return
        speed = self.test_values[self.test_index]
        step_index = self.test_index
        self.test_current_target = speed
        self.test_status.set(f"Sending step {self.test_index + 1}/{len(self.test_values)}: {speed:+.1f} rpm")

        def worker() -> None:
            try:
                self.controller.command_speed_rpm(speed)
                self.events.put(("test_step_commanded", (speed, step_index)))
            except Exception as exc:
                self.events.put(("test_step_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="test-step").start()

    def _complete_test_step(self) -> None:
        self.test_after_id = None
        if not self.test_running or self.test_current_target is None:
            return
        elapsed = monotonic() - (self.test_step_started_at or monotonic())
        if len(self.telemetry.samples) <= self.test_step_sample_count:
            maximum_wait = self.test_hold_seconds + (2.5 * (self.sim_poll_ms if self.controller.simulated else self.LIVE_POLL_MS) / 1000.0)
            if elapsed < maximum_wait:
                self.test_after_id = self.root.after(100, self._complete_test_step)
                return
            self._fail_test("No fresh encoder sample was received after the speed command")
            return
        snapshot = self.last_snapshot
        if snapshot is None or snapshot.feedback_speed_rpm is None:
            self._fail_test("Encoder feedback is unavailable at the end of the hold")
            return
        error = snapshot.feedback_speed_rpm - self.test_current_target
        passed = abs(error) <= self.test_tolerance_rpm
        self.test_step_result.set(
            f"{'PASS' if passed else 'FAIL'} • {snapshot.feedback_speed_rpm:+.1f} rpm • error {error:+.1f}"
        )
        if not passed:
            self._fail_test(
                f"Step {self.test_index + 1} encoder error {error:+.2f} rpm exceeds ±{self.test_tolerance_rpm:g} rpm"
            )
            return
        self.test_index += 1
        self.test_completed.set(f"{self.test_index} / {len(self.test_values)}")
        self._advance_test()

    def _monitor_running_test(self, snapshot: DriveSnapshot) -> None:
        if not self.test_running:
            return
        reasons: list[str] = []
        if not snapshot.drive_ready_indication:
            reasons.append("SRDY or SON became false")
        if not snapshot.emgs_hardware_healthy:
            reasons.append("EMGS hardware input became unhealthy")
        if snapshot.alarm_active:
            reasons.append(f"alarm 0x{snapshot.alarm_code:04X} became active")
        if snapshot.feedback_speed_rpm is None:
            reasons.append("encoder feedback became unavailable")
        if not snapshot.simulated and not snapshot.hardware_motion_enabled:
            reasons.append("live motion permission was automatically removed")
        if reasons:
            self._fail_test("; ".join(reasons))

    def _finish_test(self) -> None:
        if not self.test_running:
            return
        self.test_running = False
        self.test_after_id = None
        self.test_status.set("PASS • returning to zero")
        self.live_test_armed = False
        self.armed_live_speed_cap = None
        self._update_test_start_state()
        self._log(f"{self.test_type.get()} passed", "INFO", "TEST", self.test_completed.get())
        self._stop_test_motion("PASS")

    def _fail_test(self, reason: str) -> None:
        if not self.test_running and not self.test_preparing:
            return
        self.test_running = False
        self.test_preparing = False
        if self.test_after_id is not None:
            self.root.after_cancel(self.test_after_id)
            self.test_after_id = None
        self.test_failures.append(reason)
        self.test_status.set("FAILED • returning to zero")
        self.test_step_result.set(f"FAIL • {reason}")
        self.live_test_armed = False
        self.armed_live_speed_cap = None
        self._update_test_start_state()
        self._log("Test failed", "ERROR", "TEST", reason)
        self._stop_test_motion("FAILED")

    def _stop_test_motion(self, outcome: str) -> None:
        if self.controller.state is not ConnectionState.CONNECTED:
            return

        def worker() -> None:
            try:
                if self.controller.simulated:
                    self.controller.controlled_stop()
                else:
                    self.controller.disarm_live_motion(request_stop=True)
                self.events.put(("test_motion_stopped", outcome))
            except Exception as exc:
                self.events.put(("test_stop_error", (outcome, str(exc))))

        threading.Thread(target=worker, daemon=True, name="test-stop").start()

    def _abort_test(self, log: bool = True) -> None:
        was_active = self.test_running or self.test_preparing
        self.test_running = False
        self.test_preparing = False
        if self.test_after_id is not None:
            self.root.after_cancel(self.test_after_id)
            self.test_after_id = None
        if was_active:
            self.test_status.set("ABORTED • returning to zero")
            self.test_step_result.set("ABORTED")
            self.live_test_armed = False
            self.armed_live_speed_cap = None
            self._stop_test_motion("ABORTED")
        self._update_test_start_state()
        if was_active and log:
            self._log("Test aborted", "WARN", "TEST", "Zero speed requested; live permission removed")

    def _toggle_trend_pause(self, channel: str) -> None:
        if channel == "speed":
            self.speed_trends_paused = not self.speed_trends_paused
            paused = self.speed_trends_paused
            indicator = self.speed_live_indicator
        elif channel == "position":
            self.position_trends_paused = not self.position_trends_paused
            paused = self.position_trends_paused
            indicator = self.position_live_indicator
        else:
            raise ValueError(f"Unknown trend channel: {channel}")
        indicator.configure(
            text="● Paused" if paused else "● Live",
            foreground=MUTED if paused else GREEN,
        )
        if not paused:
            self._redraw_charts()

    def _clear_telemetry(self) -> None:
        self.telemetry.clear()
        self.motion_sample_times.clear()
        self.motion_late_samples = 0
        self.motion_quality_text.set(f"Target {self._desired_motion_sample_hz():g} Hz • acquiring")
        self.rms_error_text.set("--")
        self.peak_error_text.set("--")
        self._redraw_charts(force=True)
        self._log("Telemetry cleared", "INFO", "DATA")

    def _redraw_charts(self, force: bool = False) -> None:
        samples = self._downsample_for_display(self.telemetry.recent(60.0))
        self.dashboard_trend.set_samples(samples)
        if self.motion_profile is not None:
            profile_samples = self._downsample_for_display(self.profile_recorded_samples)
            self.profile_preview.set_recorded_samples(profile_samples)
            self.profile_displacement_chart.set_samples(
                profile_samples,
                self.motion_profile.duration_s,
            )
        if force or not self.speed_trends_paused:
            self.trends_chart.set_samples(samples)
        if force or not self.position_trends_paused:
            self.position_chart.set_samples(samples)

    @classmethod
    def _downsample_for_display(cls, samples: list | deque) -> list:
        """Bound Canvas work while retaining the first and latest samples.

        The original full-resolution collections remain untouched and are used
        for CSV export and calculations. This helper affects display only.
        """
        count = len(samples)
        limit = cls.MAX_CHART_DISPLAY_SAMPLES
        if count <= limit:
            return list(samples)
        step = (count - 1) / (limit - 1)
        return [samples[round(index * step)] for index in range(limit)]

    def _update_trend_channels(self) -> None:
        self.trends_chart.set_channels(
            self.channel_command.get(),
            self.channel_feedback.get(),
            self.channel_error.get(),
            self.channel_platform.get(),
        )

    def _update_position_channels(self) -> None:
        self.position_chart.set_channels(
            self.channel_motor_position.get(),
            self.channel_platform_position.get(),
        )

    def _export_telemetry(self, channel_group: str = "all") -> None:
        suffix = "speed" if channel_group == "speed" else "position" if channel_group == "position" else "all"
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="Export telemetry",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"),),
            initialfile=f"servo_telemetry_{suffix}_{datetime.now():%Y%m%d_%H%M%S}.csv",
        )
        if not path:
            return
        try:
            self.telemetry.export_csv(path, channel_group)
            self._log(f"{suffix.title()} telemetry exported", "INFO", "DATA", path)
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc), parent=self.root)

    def _request_parameter_read(self) -> None:
        if self.controller.state is not ConnectionState.CONNECTED:
            messagebox.showinfo("Not connected", "Connect before reading parameters.", parent=self.root)
            return
        self.parameter_read_button.configure(state="disabled")

        def worker() -> None:
            try:
                self.events.put(("parameters", self.controller.read_project_parameters()))
            except Exception as exc:
                self.events.put(("parameter_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="parameter-read").start()

    def _refresh_parameter_write_state(self) -> None:
        if not hasattr(self, "parameter_write_button"):
            return
        register = self.selected_parameter
        connected = self.controller.state is ConnectionState.CONNECTED
        writable = register is not None and register.persistence is not Persistence.READ_ONLY
        state = "normal" if connected and writable else "disabled"
        self.parameter_edit_entry.configure(state=state)
        self.parameter_write_button.configure(
            state=state,
            text=self._parameter_write_button_label(),
        )
        if register is None:
            return
        base_note = register.note or "Manual-verified project parameter."
        if register.persistence is Persistence.READ_ONLY:
            mode_note = "This parameter is read-only in both modes."
        elif connected and self.controller.simulated:
            mode_note = "Simulation write enabled; the real drive will not be changed."
        elif connected:
            mode_note = (
                "Live write enabled with confirmation, range checking and readback. "
                "The controller may still reject an unsafe drive state or command register."
            )
        else:
            mode_note = "Connect to Simulation or Live Drive to edit and verify this value."
        self.parameter_note.set(f"{base_note} {mode_note}")

    def _parameter_write_button_label(self) -> str:
        if self.controller.state is not ConnectionState.CONNECTED:
            return "WRITE PARAMETER"
        return "WRITE TO SIMULATOR" if self.controller.simulated else "WRITE TO LIVE DRIVE"

    @staticmethod
    def _parameter_edit_text(register: Register, value: int | None) -> str:
        if value is None:
            return ""
        if register.value_format == "HEX":
            width = register.words * 4
            unsigned = value & ((1 << (register.words * 16)) - 1)
            return f"0x{unsigned:0{width}X}"
        if register.unit == "0.1 rpm":
            return f"{value / 10.0:g}"
        return str(value)

    @staticmethod
    def _parse_parameter_edit_value(register: Register, text: str) -> int:
        cleaned = text.strip()
        if not cleaned:
            raise ValueError("Enter a new parameter value")
        if register.unit == "0.1 rpm":
            engineering_value = float(cleaned)
            raw_value = engineering_value * 10.0
            rounded = round(raw_value)
            if abs(raw_value - rounded) > 1e-9:
                raise ValueError(f"{register.parameter} accepts 0.1 rpm increments")
            return rounded
        if register.value_format == "HEX":
            return int(cleaned, 16) if cleaned.lower().startswith("0x") else int(cleaned, 10)
        return int(cleaned, 0)

    def _write_selected_parameter(self) -> None:
        register = self.selected_parameter
        if register is None:
            return
        if self.controller.state is not ConnectionState.CONNECTED:
            messagebox.showinfo(
                "Connection required",
                "Connect to Simulation or Live Drive before editing a parameter.",
                parent=self.root,
            )
            return
        try:
            value = self._parse_parameter_edit_value(register, self.parameter_edit_value.get())
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Invalid parameter value", str(exc), parent=self.root)
            return
        simulated = self.controller.simulated
        if not simulated:
            current = self.parameter_values.get(register.parameter)
            current_text = self._format_parameter_value(register, current)
            new_text = self._format_parameter_value(register, value)
            caution = ""
            if register.parameter.startswith("P3.00"):
                caution = "\n\nCommunication parameters can interrupt or change the current connection."
            if register.persistence is Persistence.POWER_CYCLE:
                caution += "\n\nThis parameter requires a drive power cycle before it becomes active."
            if not messagebox.askyesno(
                "Write live drive parameter?",
                f"Write {register.parameter} on the connected drive?\n\nCurrent: {current_text}\nNew: {new_text}\nPersistence: {register.persistence.value}{caution}\n\nThe drive must be stopped and Servo Off.",
                parent=self.root,
            ):
                return
        self.parameter_write_button.configure(text="WRITING...", state="disabled")
        self.parameter_edit_entry.configure(state="disabled")

        def worker() -> None:
            try:
                readback = self.controller.write_parameter(register.parameter, value)
                self.events.put(("parameter_written", (register.parameter, readback, simulated)))
            except Exception as exc:
                self.events.put(("parameter_write_error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="parameter-write").start()

    @staticmethod
    def _format_parameter_value(register: Register, value: int | None) -> str:
        if value is None:
            return "--"
        if register.value_format == "HEX":
            width = register.words * 4
            unsigned = value & ((1 << (register.words * 16)) - 1)
            return f"0x{unsigned:0{width}X}"
        if register.unit == "0.1 rpm":
            return f"{value / 10.0:+.1f} rpm"
        return f"{value}{(' ' + register.unit) if register.unit else ''}"

    def _populate_parameter_table(self) -> None:
        selected = self.parameter_tree.selection()
        selected_parameter = self.parameter_tree.item(selected[0], "values")[0] if selected else None
        for item in self.parameter_tree.get_children():
            self.parameter_tree.delete(item)
        query = self.parameter_search.get().strip().lower()
        selected_group = self.parameter_group.get()
        restored_item: str | None = None
        for register in PROJECT_PARAMETER_CATALOG:
            if selected_group != "All groups" and not register.parameter.startswith(f"{selected_group}."):
                continue
            if query and query not in register.parameter.lower() and query not in register.description.lower():
                continue
            drive_value = self.parameter_values.get(register.parameter)
            match = register.matches_project(drive_value) if drive_value is not None else None
            if register.project_value is None:
                status = "Runtime"
                tag = "runtime"
            elif drive_value is None:
                status = "Not read"
                tag = "runtime"
            elif match:
                status = "Match"
                tag = "match"
            else:
                status = "Mismatch"
                tag = "mismatch"
            item = self.parameter_tree.insert(
                "",
                "end",
                values=(
                    register.parameter,
                    register.description,
                    self._format_parameter_value(register, drive_value),
                    self._format_parameter_value(register, register.project_value),
                    status,
                ),
                tags=(tag,),
            )
            if register.parameter == selected_parameter:
                restored_item = item
        if restored_item:
            self.parameter_tree.selection_set(restored_item)

    def _show_parameter_details(self, _event: object = None) -> None:
        selected = self.parameter_tree.selection()
        if not selected:
            return
        parameter = str(self.parameter_tree.item(selected[0], "values")[0])
        register = next(item for item in PROJECT_PARAMETER_CATALOG if item.parameter == parameter)
        self.selected_parameter = register
        drive_value = self.parameter_values.get(parameter)
        self.parameter_detail_title.set(f"{register.parameter} • {register.description}")
        self.parameter_detail_rows["Drive value"].set(self._format_parameter_value(register, drive_value))
        self.parameter_detail_rows["Project profile"].set(self._format_parameter_value(register, register.project_value))
        self.parameter_detail_rows["Unit"].set(register.unit or "Raw value")
        if register.minimum is None and register.maximum is None:
            range_text = "Manual-defined / status"
        else:
            range_text = f"{register.minimum if register.minimum is not None else '—'} to {register.maximum if register.maximum is not None else '—'}"
        self.parameter_detail_rows["Allowed range"].set(range_text)
        self.parameter_detail_rows["Persistence"].set(register.persistence.value)
        address_text = f"0x{register.address:04X}"
        if register.words == 2:
            address_text += f" / 0x{register.address + 1:04X}"
        self.parameter_detail_rows["Address"].set(address_text)
        editable_value = drive_value if drive_value is not None else register.project_value
        self.parameter_edit_value.set(self._parameter_edit_text(register, editable_value))
        self.parameter_edit_unit.set("rpm" if register.unit == "0.1 rpm" else register.unit or "raw")
        self._refresh_parameter_write_state()

    def _log(self, event: str, level: str = "INFO", source: str = "SYSTEM", details: str = "") -> None:
        self.history.add(level, source, event, details)
        self._refresh_log_trees()

    def _refresh_log_trees(self) -> None:
        query = self.log_filter.get().strip().lower() if hasattr(self, "log_filter") else ""
        if hasattr(self, "logs_tree"):
            for item in self.logs_tree.get_children():
                self.logs_tree.delete(item)
            for record in reversed(self.history.records):
                searchable = f"{record.level} {record.source} {record.event} {record.details}".lower()
                if query and query not in searchable:
                    continue
                self.logs_tree.insert(
                    "",
                    "end",
                    values=(record.timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3], record.level, record.source, record.event, record.details),
                    tags=(record.level,),
                )
        if hasattr(self, "dashboard_log_tree"):
            for item in self.dashboard_log_tree.get_children():
                self.dashboard_log_tree.delete(item)
            for record in list(self.history.records)[-3:][::-1]:
                self.dashboard_log_tree.insert("", "end", values=(record.timestamp.strftime("%H:%M:%S"), record.level, f"{record.event} {record.details}".strip()))
        if hasattr(self, "log_summary"):
            records = list(self.history.records)
            self.log_summary["Records"].set(str(len(records)))
            self.log_summary["Warnings"].set(str(sum(record.level == "WARN" for record in records)))
            self.log_summary["Errors"].set(str(sum(record.level == "ERROR" for record in records)))
            self.log_summary["Telemetry samples"].set(str(len(self.telemetry.samples)))

    def _clear_logs(self) -> None:
        # Clear is intentionally a view/session action; exported files are untouched.
        self.history.records.clear()
        self._log("Log cleared", "INFO", "SYSTEM")

    def _export_logs(self) -> None:
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="Export event log",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"),),
            initialfile=f"servo_events_{datetime.now():%Y%m%d_%H%M%S}.csv",
        )
        if not path:
            return
        try:
            self.history.export_csv(path)
            self._log("Event log exported", "INFO", "DATA", path)
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc), parent=self.root)

    def _on_close(self) -> None:
        # Shutdown crosses several worker lifecycles. Preserve cancellation and
        # stop ordering when adding services so a late callback cannot restart work.
        self.polling = False
        self._cancel_speed_profile_execution()
        self._cancel_profile_safety_stop_timer()
        self.profile_coordinator_monitor_stop.set()
        self._stop_motion_polling()
        self.labjack.stop()
        self.clock_sync.stop()
        self._save_settings_silent()
        if self.speed_after_id is not None:
            self.root.after_cancel(self.speed_after_id)
        if self.test_after_id is not None:
            self.root.after_cancel(self.test_after_id)
        if self.profile_run_after_id is not None:
            self.root.after_cancel(self.profile_run_after_id)
        if self.dashboard_position_after_id is not None:
            self.root.after_cancel(self.dashboard_position_after_id)
        try:
            self.controller.disconnect()
        finally:
            self.root.destroy()
