"""
Standalone LattePanda clock, run, EVK4-recording and trigger service.

Run this file on the Windows 11 LattePanda.

Features:
1. UDP clock responder on port 49152, compatible with the existing PC clock_sync.py.
2. UDP discovery responder on port 49153 so the PC can find the LattePanda if its IP changes.
3. UDP run coordinator on port 49154 that selects the authoritative test T0.
4. EVK4 opened, biased and streaming at service startup.
5. EVK4 RAW pre-roll recording and scheduled 1 Hz UART 0x00 writes from T0.

EVK4 wiring:
    LattePanda SIO UART TX  -> EVK4 TRIG_IN_P
    LattePanda GND          -> EVK4 TRIG_IN_N
    LattePanda RX           -> not connected

The deployed LattePanda camera, bias, recording-directory and COM1 settings are
defaults so a double-click starts the complete physical service. Command-line
options remain available for testing and overrides.


Maintenance boundary: this file is deployed standalone, so do not introduce
imports from the Motor-PC servo_control package. Keep clock structures and
STRG_RUN_V1 messages compatible with clock_sync.py/run_coordinator.py.
Camera pumping, serial deadlines and the watchdog have separate lifecycles;
finalize a recording only after its UART worker stops writing its CSV.
The service watchdog owns camera/UART cleanup, not the physical motor stop.
Embedded wiring/default paths describe this deployment, not a generic EVK4
electrical specification; revalidate them when moving to another rig.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import struct
import sys
import threading
import traceback
import csv
from datetime import datetime
from pathlib import Path
from time import perf_counter_ns, sleep, time_ns


MAGIC = b"STRG"
VERSION = 1

CLOCK_PORT = 49152
DISCOVERY_PORT = 49153
COORDINATOR_PORT = 49154
RUN_PROTOCOL = "STRG_RUN_V1"

REQUEST = struct.Struct("!4sBQQQ")
RESPONSE = struct.Struct("!4sBQQQQQQQ")

DISCOVERY_REQUEST = b"STRG_DISCOVER_V1"
DISCOVERY_RESPONSE_PREFIX = "STRG_RESPONDER_V1"

# LattePanda installation defaults. These make a double-click launch use the
# complete physical configuration without requiring a prepared terminal.
METAVISION_SITE_PACKAGES = Path(
    r"C:\Program Files\Prophesee\lib\python3\site-packages"
)
METAVISION_HAL_PLUGINS = Path(
    r"C:\Program Files\Prophesee\lib\metavision\hal\plugins"
)
DEFAULT_BIAS_FILE = Path(
    r"C:\Users\LattePanda\Documents\metavision\biases\Settings1.bias"
)
DEFAULT_RECORDINGS_DIR = Path(
    r"C:\Users\LattePanda\Documents\metavision\recordings"
)
DEFAULT_WATCHDOG_GRACE_S = 10.0
DEFAULT_MINIMUM_FREE_GIB = 2.0

if METAVISION_SITE_PACKAGES.is_dir():
    site_packages = str(METAVISION_SITE_PACKAGES)
    if site_packages not in sys.path:
        sys.path.insert(0, site_packages)
os.environ.setdefault("MV_HAL_PLUGIN_PATH", str(METAVISION_HAL_PLUGINS))


def load_bias_file(device, bias_file: Path) -> dict[str, int]:
    """Apply the legacy Metavision ``value % name`` bias-file format."""
    if not bias_file.is_file():
        raise FileNotFoundError(f"Bias file not found: {bias_file}")
    facility = device.get_i_ll_biases()
    if facility is None:
        raise RuntimeError("The selected camera does not expose low-level biases")
    requested: dict[str, int] = {}
    for line_number, raw_line in enumerate(
        bias_file.read_text(encoding="utf-8-sig").splitlines(), start=1,
    ):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split("%", 1)
        if len(fields) != 2:
            raise ValueError(f"Invalid bias-file line {line_number}: {raw_line!r}")
        try:
            value = int(fields[0].strip())
        except ValueError as exc:
            raise ValueError(f"Invalid bias value on line {line_number}: {raw_line!r}") from exc
        name = fields[1].strip()
        if not name:
            raise ValueError(f"Missing bias name on line {line_number}")
        if not facility.set(name, value):
            raise RuntimeError(f"Camera rejected {name}={value} from {bias_file}")
        requested[name] = value
    if not requested:
        raise ValueError(f"Bias file contains no bias settings: {bias_file}")
    return {name: int(facility.get(name)) for name in requested}


TRIGGER_EXTRACTION_CHUNK_US = 10_000
TRIGGER_EXTRACTION_MAX_EVENTS = 20_000_000


def export_raw_trigger_events(raw_path: Path, output_path: Path, reader_factory=None) -> int:
    """Decode all external-trigger events from a finalized RAW recording."""
    if reader_factory is None:
        try:
            from metavision_core.event_io.raw_reader import RawReader
        except ImportError as exc:
            raise RuntimeError("Metavision RawReader is unavailable") from exc
        reader_factory = RawReader

    # RawReader uses a rolling CD-event buffer even though this function only
    # needs external-trigger events. Keep each requested time slice short so a
    # high-rate recording cannot accumulate a full second of CD events before
    # the reader returns. The larger-than-default buffer also leaves headroom
    # for a dense decoder batch from the EVK4.
    reader = reader_factory(
        str(raw_path),
        max_events=TRIGGER_EXTRACTION_MAX_EVENTS,
        do_time_shifting=False,
    )
    events: dict[tuple[int, int, int], tuple[int, int, int]] = {}
    while not reader.is_done():
        reader.load_delta_t(TRIGGER_EXTRACTION_CHUNK_US)
        for event in reader.get_ext_trigger_events():
            try:
                timestamp = int(event["t"])
                polarity = int(event["p"])
                channel_id = int(event["id"])
            except (IndexError, KeyError, TypeError):
                timestamp = int(event.t)
                polarity = int(event.p)
                channel_id = int(event.id)
            events[(timestamp, polarity, channel_id)] = (timestamp, polarity, channel_id)
        clear = getattr(reader, "clear_ext_trigger_events", None)
        if clear is not None:
            clear()

    ordered = sorted(events.values())
    with output_path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("trigger_index", "raw_timestamp_us", "polarity", "channel_id"))
        for index, (timestamp, polarity, channel_id) in enumerate(ordered):
            writer.writerow((index, timestamp, polarity, channel_id))
    return len(ordered)


class Evk4ArmedCamera:
    """Open, configure and stream the EVK4 for the lifetime of the responder."""

    def __init__(
        self, output_dir: Path, camera: str, bias_file: Path | None,
        minimum_free_bytes: int = 0,
    ) -> None:
        try:
            from metavision_core.event_io.raw_reader import initiate_device
            from metavision_hal import I_TriggerIn
        except ImportError as exc:
            raise RuntimeError(
                "Metavision SDK Python modules are unavailable; run the responder "
                "with the Metavision-configured Python environment"
            ) from exc

        self.output_dir = output_dir
        self.camera = camera
        self.bias_file = bias_file
        self.minimum_free_bytes = minimum_free_bytes
        self._api_lock = threading.Lock()
        self._stop = threading.Event()
        self._healthy = threading.Event()
        self._failure_reason = "camera has not started"
        self._recording: dict[str, object] | None = None
        self.paths: tuple[Path, Path, Path, Path] | None = None

        print("[CAMERA] Opening EVK4...", flush=True)
        self._device = initiate_device(camera, do_time_shifting=True)
        self.applied_biases: dict[str, int] = {}
        if bias_file is not None:
            self.applied_biases = load_bias_file(self._device, bias_file)
            print(f"[CAMERA] Applied biases from {bias_file}", flush=True)
            for name, value in self.applied_biases.items():
                print(f"[CAMERA]   {name}={value}", flush=True)

        trigger_in = self._device.get_i_trigger_in()
        if trigger_in is None:
            raise RuntimeError("The selected camera does not expose Trigger In")
        if not trigger_in.enable(I_TriggerIn.Channel.MAIN):
            raise RuntimeError("Could not enable the EVK4 MAIN Trigger In channel")

        self._event_stream = self._device.get_i_events_stream()
        if self._event_stream is None:
            raise RuntimeError("The selected EVK4 has no event stream")
        self._event_stream.start()
        self._healthy.set()
        self._failure_reason = ""
        self._pump_thread = threading.Thread(
            target=self._pump, daemon=True, name="evk4-raw-pump",
        )
        self._pump_thread.start()
        print("[CAMERA] READY - open, streaming, biased and Trigger In enabled", flush=True)

    def _pump(self) -> None:
        try:
            while not self._stop.is_set():
                with self._api_lock:
                    result = self._event_stream.poll_buffer()
                    if result > 0:
                        # This call also commits the buffer to the active RAW log.
                        self._event_stream.get_latest_raw_data()
                if result < 0:
                    self._mark_failed("event stream returned an error or stopped")
                    return
                if result == 0:
                    self._stop.wait(0.001)
        except Exception as exc:
            self._mark_failed(f"event-stream pump failed: {exc}")

    def _mark_failed(self, reason: str) -> None:
        self._failure_reason = reason
        self._healthy.clear()
        print(f"[CAMERA] FAILED - {reason}", flush=True)

    @property
    def is_healthy(self) -> bool:
        return self._healthy.is_set() and self._pump_thread.is_alive()

    @property
    def health_reason(self) -> str:
        return "healthy" if self.is_healthy else self._failure_reason

    @property
    def is_recording(self) -> bool:
        return self._recording is not None

    def start_recording(self, run_id: str) -> tuple[Path, Path, Path, Path]:
        # The camera is already streaming; ARM starts logging without paying SDK
        # open latency. The returned trigger-events path is reserved linkage only:
        # normal extraction occurs later on the Motor PC.
        if not self.is_healthy:
            raise RuntimeError(f"EVK4 is not healthy: {self.health_reason}")
        if self._recording is not None:
            raise RuntimeError("EVK4 is already recording")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        free_bytes = shutil.disk_usage(self.output_dir).free
        if free_bytes < self.minimum_free_bytes:
            required_gib = self.minimum_free_bytes / (1024 ** 3)
            available_gib = free_bytes / (1024 ** 3)
            raise RuntimeError(
                f"Insufficient recording space: {available_gib:.2f} GiB free; "
                f"at least {required_gib:.2f} GiB is required"
            )
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        stem = f"{run_id}_{stamp}"
        raw_path = self.output_dir / f"{stem}.raw"
        writes_path = self.output_dir / f"{stem}_0x00_writes.csv"
        trigger_events_path = self.output_dir / f"{stem}_trigger_events.csv"
        metadata_path = self.output_dir / f"{stem}_metadata.json"
        if any(path.exists() for path in (raw_path, writes_path, trigger_events_path, metadata_path)):
            raise FileExistsError(f"Recording output already exists for run {run_id}")

        writes_stream = writes_path.open("x", newline="", encoding="utf-8")
        writer = csv.writer(writes_stream)
        writer.writerow((
            "sequence", "scheduled_lattepanda_monotonic_ns",
            "write_requested_lattepanda_monotonic_ns",
            "write_completed_lattepanda_monotonic_ns", "time_from_t0_s",
            "schedule_error_us", "write_call_duration_us", "status", "error",
        ))
        writes_stream.flush()
        with self._api_lock:
            if not self._event_stream.log_raw_data(str(raw_path)):
                writes_stream.close()
                raise RuntimeError(f"Could not start RAW recording: {raw_path}")
        started_ns = perf_counter_ns()
        self.paths = (raw_path, writes_path, trigger_events_path, metadata_path)
        self._recording = {
            "run_id": run_id,
            "started_ns": started_ns,
            "started_utc_ns": time_ns(),
            "writes_stream": writes_stream,
            "writes_writer": writer,
            "write_count": 0,
        }
        print(f"[CAMERA] Recording started: {raw_path}", flush=True)
        return self.paths

    def record_write(
        self, sequence: int, scheduled_ns: int, requested_ns: int,
        completed_ns: int, status: str, error: str = "",
    ) -> None:
        # Flush each attempt for diagnostics if later acquisition fails. Requested
        # and completed bracket the API call, not guaranteed physical TX-edge bounds.
        recording = self._recording
        if recording is None:
            return
        writer = recording["writes_writer"]
        writer.writerow((
            sequence, scheduled_ns, requested_ns, completed_ns,
            f"{(scheduled_ns - self.t0_ns) / 1_000_000_000:.9f}",
            f"{(requested_ns - scheduled_ns) / 1_000:.3f}",
            f"{(completed_ns - requested_ns) / 1_000:.3f}", status, error,
        ))
        recording["writes_stream"].flush()
        recording["write_count"] = int(recording["write_count"]) + 1

    def set_t0(self, t0_ns: int) -> None:
        self.t0_ns = t0_ns

    def stop_recording(self, reason: str = "requested") -> None:
        # Metadata describes this logging session even when it stopped abnormally.
        # Leave trigger count/status deferred until an extractor has inspected RAW.
        recording = self._recording
        if recording is None:
            return
        with self._api_lock:
            self._event_stream.stop_log_raw_data()
        ended_ns = perf_counter_ns()
        self._recording = None
        recording["writes_stream"].close()
        raw_path, writes_path, trigger_events_path, metadata_path = self.paths
        metadata = {
            "format_version": 3,
            "run_id": recording["run_id"],
            "hostname": socket.gethostname(),
            "raw_file": raw_path.name,
            "uart_writes_file": writes_path.name,
            "trigger_events_file": trigger_events_path.name,
            "session_start_lattepanda_monotonic_ns": recording["started_ns"],
            "session_start_utc_ns": recording["started_utc_ns"],
            "session_end_lattepanda_monotonic_ns": ended_ns,
            "lattepanda_t0_monotonic_ns": getattr(self, "t0_ns", None),
            "uart_write_count": recording["write_count"],
            "trigger_event_count": None,
            "trigger_extraction_status": "deferred_to_motor_pc",
            "trigger_extraction_error": "",
            "bias_file": None if self.bias_file is None else str(self.bias_file),
            "applied_biases": self.applied_biases,
            "trigger_in_channel": "MAIN",
            "stop_reason": reason,
            "camera_healthy_at_finalize": self.is_healthy,
            "camera_health_details": self.health_reason,
            "uart_timestamp_semantics": (
                "requested is immediately before pyserial.write; completed is after "
                "write and flush; the EVK4 external-trigger event is the physical-edge observation"
            ),
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[CAMERA] Recording saved: {raw_path}", flush=True)

    def close(self) -> None:
        self.stop_recording()
        self._stop.set()
        self._pump_thread.join(timeout=2.0)
        with self._api_lock:
            self._event_stream.stop()


class ScheduledSerialTrigger:
    """Keep COM1 open and issue 0x00 at T0 and every second after."""

    def __init__(self, port: str, baudrate: int) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for the scheduled COM1 trigger") from exc
        self._serial_module = serial
        self.port = port
        self.baudrate = baudrate
        self._cancel: threading.Event | None = None
        self._thread: threading.Thread | None = None
        self._uart = None
        self._healthy = False
        self._active = False
        self._failure_reason = "serial port has not opened"

    def open(self) -> None:
        serial = self._serial_module
        self._uart = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._healthy = True
        self._failure_reason = ""
        print(f"[UART] READY - {self.port}@{self.baudrate}, TX idle high", flush=True)

    @property
    def is_healthy(self) -> bool:
        return (
            self._healthy
            and self._uart is not None
            and bool(getattr(self._uart, "is_open", True))
        )

    @property
    def health_reason(self) -> str:
        return "healthy" if self.is_healthy else self._failure_reason

    @property
    def is_active(self) -> bool:
        return self._active and self._thread is not None and self._thread.is_alive()

    def arm(self, t0_ns: int, record_write) -> None:
        # Absolute T0+n-second deadlines avoid accumulating each write's latency.
        # The camera trigger event, not pyserial completion, observes the actual edge.
        if self._uart is None:
            self.open()
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("UART trigger is already armed")
        cancel = threading.Event()
        self._cancel = cancel
        self._active = True

        def worker() -> None:
            sequence = 0
            period_ns = 1_000_000_000
            while not cancel.is_set():
                deadline_ns = t0_ns + sequence * period_ns
                while not cancel.is_set():
                    remaining_ns = deadline_ns - perf_counter_ns()
                    if remaining_ns <= 0:
                        break
                    cancel.wait(min(remaining_ns / 1_000_000_000.0, 0.05))
                if cancel.is_set():
                    return
                requested_ns = perf_counter_ns()
                try:
                    written = self._uart.write(b"\x00")
                    if written != 1:
                        raise IOError(f"serial write accepted {written!r} bytes instead of 1")
                    self._uart.flush()
                    completed_ns = perf_counter_ns()
                    record_write(
                        sequence, deadline_ns, requested_ns, completed_ns, "completed", "",
                    )
                except Exception as exc:
                    failed_ns = perf_counter_ns()
                    self._healthy = False
                    self._failure_reason = f"0x00 write failed: {exc}"
                    self._active = False
                    record_write(
                        sequence, deadline_ns, requested_ns, failed_ns, "failed", str(exc),
                    )
                    return
                sequence += 1

        self._thread = threading.Thread(target=worker, daemon=True, name="scheduled-com1-trigger")
        self._thread.start()

    def stop_run(self) -> None:
        if self._cancel is not None:
            self._cancel.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._active = False
        self._thread = None
        self._cancel = None

    def close(self) -> None:
        self.stop_run()
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._healthy = False
        self._failure_reason = "serial port closed"


class LattePandaRunCoordinator:
    def __init__(
        self, output_dir: Path, camera: str, bias_file: Path | None,
        camera_enabled: bool, serial_enabled: bool, serial_port: str, baudrate: int,
        watchdog_grace_s: float = DEFAULT_WATCHDOG_GRACE_S, minimum_free_bytes: int = 0,
    ) -> None:
        self.output_dir = output_dir
        self.camera = camera
        self.bias_file = bias_file
        self.camera_enabled = camera_enabled
        self.serial_enabled = serial_enabled
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.watchdog_grace_s = watchdog_grace_s
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._run: dict[str, object] | None = None
        self._camera_recorder: Evk4ArmedCamera | None = None
        self._serial_trigger: ScheduledSerialTrigger | None = None
        self._terminal_responses: dict[str, dict[str, object]] = {}
        self._watchdog_cancel = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        if self.camera_enabled:
            self._camera_recorder = Evk4ArmedCamera(
                output_dir, camera, bias_file, minimum_free_bytes,
            )
        if self.serial_enabled:
            self._serial_trigger = ScheduledSerialTrigger(serial_port, baudrate)
            self._serial_trigger.open()

    def handle(self, request: dict[str, object]) -> dict[str, object]:
        request_id = request.get("request_id")
        if request.get("protocol") != RUN_PROTOCOL:
            return self._error(request_id, "unsupported protocol")
        action = str(request.get("action", "")).upper()
        try:
            if action == "ARM":
                return self._arm(request)
            if action == "STATUS":
                return self._status(request)
            if action in ("CANCEL", "COMPLETE"):
                return self._finish(request, action)
            return self._error(request_id, f"unsupported action {action!r}")
        except Exception as exc:
            self._stop_run_hardware()
            return self._error(request_id, str(exc))

    def _arm(self, request: dict[str, object]) -> dict[str, object]:
        with self._lifecycle_lock:
            return self._arm_locked(request)

    def _arm_locked(self, request: dict[str, object]) -> dict[str, object]:
        # ARM retry semantics must retain the existing run/T0 rather than start a
        # second RAW or pulse train. Camera readiness and free space precede motion.
        run_id = str(request.get("run_id", "")).strip()
        pre_roll_s = float(request.get("pre_roll_s", 0.0))
        margin_s = float(request.get("preparation_margin_s", 1.0))
        duration_s = float(request.get("expected_duration_s") or 60.0)
        if not run_id:
            raise ValueError("run_id is required")
        if pre_roll_s < 0.0:
            raise ValueError("pre_roll_s must be nonnegative")
        if margin_s < 0.25:
            raise ValueError("preparation_margin_s must be at least 0.25 seconds")
        if duration_s <= 0:
            raise ValueError("expected_duration_s must be greater than zero")
        with self._lock:
            existing = self._run
        if existing is not None:
            if existing["run_id"] != run_id:
                return self._error(request.get("request_id"), f"coordinator busy with run {existing['run_id']}")
            return self._response(request, existing)

        camera_recorder = self._camera_recorder
        serial_trigger = self._serial_trigger
        if camera_recorder is not None:
            if not camera_recorder.is_healthy:
                raise RuntimeError(f"EVK4 is not ready: {camera_recorder.health_reason}")
        if serial_trigger is not None and not serial_trigger.is_healthy:
            raise RuntimeError(f"UART trigger is not ready: {serial_trigger.health_reason}")
        if camera_recorder is not None:
            camera_recorder.start_recording(run_id)
        # Camera and COM1 were opened at service startup. ARM only starts the
        # RAW log and schedules T0, so device discovery/opening cannot consume
        # the pre-roll window.
        armed_at_ns = perf_counter_ns()
        t0_ns = armed_at_ns + round((pre_roll_s + margin_s) * 1_000_000_000)
        if camera_recorder is not None:
            camera_recorder.set_t0(t0_ns)
        if serial_trigger is not None:
            recorder = (
                camera_recorder.record_write if camera_recorder is not None
                else lambda *_args, **_kwargs: None
            )
            serial_trigger.arm(t0_ns, recorder)
        run = {
            "run_id": run_id,
            "profile_name": str(request.get("profile_name", "")),
            "armed_at_ns": armed_at_ns,
            "t0_ns": t0_ns,
            "pre_roll_s": pre_roll_s,
            "preparation_margin_s": margin_s,
            "expected_duration_s": duration_s,
            "watchdog_deadline_ns": (
                t0_ns + round((duration_s + self.watchdog_grace_s) * 1_000_000_000)
            ),
            "camera_files": (
                [] if camera_recorder is None or camera_recorder.paths is None
                else [str(path) for path in camera_recorder.paths]
            ),
        }
        with self._lock:
            self._run = run
        self._start_watchdog(run)
        return self._response(request, run)

    def _start_watchdog(self, run: dict[str, object]) -> None:
        cancel = threading.Event()
        self._watchdog_cancel = cancel
        run_id = str(run["run_id"])
        deadline_ns = int(run["watchdog_deadline_ns"])

        def worker() -> None:
            remaining_s = max(0.0, (deadline_ns - perf_counter_ns()) / 1_000_000_000.0)
            if cancel.wait(remaining_s):
                return
            self._expire_watchdog(run_id)

        self._watchdog_thread = threading.Thread(
            target=worker, daemon=True, name="run-safety-watchdog",
        )
        self._watchdog_thread.start()

    def _expire_watchdog(self, run_id: str) -> None:
        with self._lifecycle_lock:
            with self._lock:
                run = self._run
            if run is None or str(run["run_id"]) != run_id:
                return
            print(
                f"[WATCHDOG] Run {run_id} exceeded its safety deadline; "
                "stopping UART writes and finalizing the recording",
                flush=True,
            )
            finalization_error = ""
            try:
                self._stop_run_hardware("watchdog_timeout")
            except Exception as exc:
                finalization_error = str(exc)
                print(f"[WATCHDOG] Finalization error: {exc}", flush=True)
            run["finalization_error"] = finalization_error
            request = {"request_id": None}
            state = "WATCHDOG_TIMEOUT" if not finalization_error else "WATCHDOG_TIMEOUT_WITH_ERRORS"
            response = self._response(request, run, forced_state=state)
            with self._lock:
                self._run = None
                self._terminal_responses[run_id] = response

    def _status(self, request: dict[str, object]) -> dict[str, object]:
        with self._lock:
            run = self._run
            cached = self._terminal_responses.get(str(request.get("run_id", "")))
        if run is None or run["run_id"] != request.get("run_id"):
            if cached is not None:
                return {**cached, "request_id": request.get("request_id")}
            return self._error(request.get("request_id"), "unknown run")
        return self._response(request, run)

    def _finish(self, request: dict[str, object], state: str) -> dict[str, object]:
        with self._lifecycle_lock:
            return self._finish_locked(request, state)

    def _finish_locked(self, request: dict[str, object], state: str) -> dict[str, object]:
        with self._lock:
            run = self._run
            cached = self._terminal_responses.get(str(request.get("run_id", "")))
        if run is None or run["run_id"] != request.get("run_id"):
            if cached is not None:
                return {**cached, "request_id": request.get("request_id")}
            return self._error(request.get("request_id"), "unknown run")
        self._watchdog_cancel.set()
        finalization_error = ""
        try:
            self._stop_run_hardware(state.lower())
        except Exception as exc:
            finalization_error = str(exc)
        run["finalization_error"] = finalization_error
        terminal_state = state if not finalization_error else f"{state}_WITH_ERRORS"
        response = self._response(request, run, forced_state=terminal_state)
        with self._lock:
            self._run = None
            self._terminal_responses[str(run["run_id"])] = response
        return response

    def _stop_run_hardware(self, reason: str = "requested") -> None:
        # Stop UART callbacks before closing the recording/write CSV they target.
        # This cleanup has no direct control of the Motor-PC servo drive.
        if self._serial_trigger is not None:
            self._serial_trigger.stop_run()
        if self._camera_recorder is not None:
            # Preserve a short tail after the final UART operation so the raw
            # pump can commit the corresponding EVK4 trigger buffer.
            sleep(0.1)
            self._camera_recorder.stop_recording(reason)

    def shutdown(self) -> None:
        self._watchdog_cancel.set()
        self._stop_run_hardware("responder_shutdown")
        if self._serial_trigger is not None:
            self._serial_trigger.close()
        if self._camera_recorder is not None:
            self._camera_recorder.close()

    def camera_self_test(self, duration_s: float) -> tuple[Path, Path, Path, Path]:
        if self._camera_recorder is None:
            raise RuntimeError("Camera recording is disabled")
        paths = self._camera_recorder.start_recording("camera-test")
        self._camera_recorder.set_t0(perf_counter_ns())
        print(f"[TEST] Recording for {duration_s:.3f} seconds...", flush=True)
        sleep(duration_s)
        self._camera_recorder.stop_recording("camera_self_test_complete")
        return paths

    def camera_trigger_self_test(self, duration_s: float) -> tuple[Path, Path, Path, Path]:
        if self._camera_recorder is None:
            raise RuntimeError("Camera recording is disabled")
        if self._serial_trigger is None:
            raise RuntimeError("UART trigger is disabled")
        paths = self._camera_recorder.start_recording("trigger-test")
        t0_ns = perf_counter_ns() + 1_000_000_000
        self._camera_recorder.set_t0(t0_ns)
        self._serial_trigger.arm(t0_ns, self._camera_recorder.record_write)
        print(
            f"[TEST] One-second pre-roll, then 1 Hz 0x00 writes for "
            f"{duration_s:.3f} seconds...",
            flush=True,
        )
        sleep(1.0 + duration_s)
        self._stop_run_hardware("camera_trigger_self_test_complete")
        return paths

    def _response(
        self, request: dict[str, object], run: dict[str, object], forced_state: str | None = None,
    ) -> dict[str, object]:
        now_ns = perf_counter_ns()
        state = forced_state or ("ARMED" if now_ns < int(run["t0_ns"]) else "RUNNING")
        return {
            "protocol": RUN_PROTOCOL,
            "request_id": request.get("request_id"),
            "ok": True,
            "run_id": run["run_id"],
            "state": state,
            "lattepanda_now_monotonic_ns": now_ns,
            "lattepanda_t0_monotonic_ns": run["t0_ns"],
            "pre_roll_s": run["pre_roll_s"],
            "preparation_margin_s": run["preparation_margin_s"],
            "expected_duration_s": run.get("expected_duration_s"),
            "watchdog_deadline_lattepanda_monotonic_ns": run.get("watchdog_deadline_ns"),
            "watchdog_grace_s": self.watchdog_grace_s,
            "finalization_error": run.get("finalization_error", ""),
            "camera_enabled": self.camera_enabled,
            "camera_healthy": (
                False if self._camera_recorder is None else self._camera_recorder.is_healthy
            ),
            "camera_health_details": (
                "disabled" if self._camera_recorder is None else self._camera_recorder.health_reason
            ),
            "camera_recording_active": (
                False if self._camera_recorder is None else self._camera_recorder.is_recording
            ),
            "serial_trigger_enabled": self.serial_enabled,
            "serial_trigger_healthy": (
                False if self._serial_trigger is None else self._serial_trigger.is_healthy
            ),
            "serial_trigger_health_details": (
                "disabled" if self._serial_trigger is None else self._serial_trigger.health_reason
            ),
            "serial_trigger_active": (
                False if self._serial_trigger is None else self._serial_trigger.is_active
            ),
            "camera_files": run.get("camera_files", []),
        }

    @staticmethod
    def _error(request_id, message: str) -> dict[str, object]:
        return {
            "protocol": RUN_PROTOCOL,
            "request_id": request_id,
            "ok": False,
            "state": "FAILED",
            "error": message,
        }


def serve_run_coordinator(
    host: str, port: int, stop_event: threading.Event, coordinator: LattePandaRunCoordinator,
    ready_event: threading.Event | None = None,
) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        sock.settimeout(0.5)
        if ready_event is not None:
            ready_event.set()
        print(f"[RUN] Listening on UDP {host}:{port}", flush=True)
        while not stop_event.is_set():
            try:
                packet, address = sock.recvfrom(8192)
            except socket.timeout:
                continue
            try:
                request = json.loads(packet.decode("utf-8"))
                if not isinstance(request, dict):
                    raise ValueError("request must be a JSON object")
                response = coordinator.handle(request)
            except Exception as exc:
                response = {"protocol": RUN_PROTOCOL, "ok": False, "state": "FAILED", "error": str(exc)}
            sock.sendto(json.dumps(response, separators=(",", ":")).encode("utf-8"), address)


def serve_clock(
    host: str,
    port: int,
    stop_event: threading.Event,
) -> None:
    """Respond to NTP-style clock requests from the main PC."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        sock.settimeout(0.5)

        print(f"[CLOCK] Listening on UDP {host}:{port}", flush=True)

        while not stop_event.is_set():
            try:
                packet, address = sock.recvfrom(REQUEST.size)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop_event.is_set():
                    print(f"[CLOCK] Socket error: {exc}", flush=True)
                continue

            remote_receive_mono = perf_counter_ns()
            remote_receive_utc = time_ns()

            try:
                (
                    magic,
                    version,
                    sequence,
                    local_send_mono,
                    local_send_utc,
                ) = REQUEST.unpack(packet)
            except struct.error:
                continue

            if magic != MAGIC or version != VERSION:
                continue

            remote_send_mono = perf_counter_ns()
            remote_send_utc = time_ns()

            response = RESPONSE.pack(
                MAGIC,
                VERSION,
                sequence,
                local_send_mono,
                local_send_utc,
                remote_receive_mono,
                remote_receive_utc,
                remote_send_mono,
                remote_send_utc,
            )

            try:
                sock.sendto(response, address)
            except OSError as exc:
                if not stop_event.is_set():
                    print(f"[CLOCK] Send error: {exc}", flush=True)


def serve_discovery(
    host: str,
    discovery_port: int,
    clock_port: int,
    stop_event: threading.Event,
) -> None:
    """
    Respond to discovery broadcasts from the PC.

    Request:
        STRG_DISCOVER_V1

    Response:
        STRG_RESPONDER_V1|<hostname>|<clock_port>

    The PC learns the LattePanda IP from the source address of the response.
    """
    hostname = socket.gethostname()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, discovery_port))
        sock.settimeout(0.5)

        print(
            f"[DISCOVERY] Listening on UDP {host}:{discovery_port} as {hostname}",
            flush=True,
        )

        response = (
            f"{DISCOVERY_RESPONSE_PREFIX}|{hostname}|{clock_port}"
        ).encode("utf-8")

        while not stop_event.is_set():
            try:
                packet, address = sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop_event.is_set():
                    print(f"[DISCOVERY] Socket error: {exc}", flush=True)
                continue

            if packet.strip() != DISCOVERY_REQUEST:
                continue

            try:
                sock.sendto(response, address)
                print(
                    f"[DISCOVERY] Replied to {address[0]}:{address[1]}",
                    flush=True,
                )
            except OSError as exc:
                if not stop_event.is_set():
                    print(f"[DISCOVERY] Send error: {exc}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "LattePanda UDP clock responder with automatic discovery "
            "and optional EVK4 UART timing pulses"
        )
    )

    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Local interface to listen on (default: all interfaces)",
    )
    parser.add_argument(
        "--clock-port",
        type=int,
        default=CLOCK_PORT,
        help=f"UDP clock-sync port (default: {CLOCK_PORT})",
    )
    parser.add_argument(
        "--discovery-port",
        type=int,
        default=DISCOVERY_PORT,
        help=f"UDP discovery port (default: {DISCOVERY_PORT})",
    )
    parser.add_argument(
        "--coordinator-port",
        type=int,
        default=COORDINATOR_PORT,
        help=f"UDP run-coordinator port (default: {COORDINATOR_PORT})",
    )
    parser.add_argument(
        "--trigger",
        dest="trigger",
        action="store_true",
        help="Enable scheduled 1 Hz UART writes (enabled by default)",
    )
    parser.add_argument(
        "--no-trigger",
        dest="trigger",
        action="store_false",
        help="Disable scheduled UART writes",
    )
    parser.add_argument(
        "--serial",
        default="COM1",
        help="LattePanda SIO/debug UART COM port (default: COM1)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="UART baud rate (default: 115200)",
    )
    parser.add_argument(
        "--camera-recording",
        dest="camera_recording",
        action="store_true",
        help="Open and arm the EVK4 (enabled by default)",
    )
    parser.add_argument(
        "--no-camera-recording",
        dest="camera_recording",
        action="store_false",
        help="Disable EVK4 opening and recording",
    )
    parser.add_argument(
        "--camera",
        default="",
        help="EVK4 serial number; blank selects the first camera",
    )
    parser.add_argument(
        "--recordings-dir",
        type=Path,
        default=DEFAULT_RECORDINGS_DIR,
        help="LattePanda directory for RAW, UART-write and metadata files",
    )
    parser.add_argument(
        "--bias-file",
        type=Path,
        default=DEFAULT_BIAS_FILE,
        help="Legacy Metavision .bias file applied when the EVK4 opens",
    )
    parser.add_argument(
        "--camera-test-seconds",
        type=float,
        help="Open/configure the EVK4, make a test RAW recording, then exit",
    )
    parser.add_argument(
        "--trigger-test-seconds",
        type=float,
        help="Record the EVK4 with 1 Hz COM1 0x00 writes, then exit",
    )
    parser.add_argument(
        "--extract-trigger-events",
        type=Path,
        help="Decode Trigger In events from one existing RAW file, then exit",
    )
    parser.add_argument(
        "--trigger-events-output",
        type=Path,
        help="Optional output CSV for --extract-trigger-events",
    )
    parser.add_argument(
        "--watchdog-grace-seconds",
        type=float,
        default=DEFAULT_WATCHDOG_GRACE_S,
        help="Maximum extra run time after the expected duration (default: 10 s)",
    )
    parser.add_argument(
        "--minimum-free-gib",
        type=float,
        default=DEFAULT_MINIMUM_FREE_GIB,
        help="Free disk space required before recording (default: 2 GiB)",
    )
    parser.set_defaults(trigger=True, camera_recording=True)

    args = parser.parse_args()

    if args.extract_trigger_events is not None:
        raw_path = args.extract_trigger_events.expanduser().resolve()
        if not raw_path.is_file():
            raise FileNotFoundError(f"RAW recording not found: {raw_path}")
        output_path = (
            args.trigger_events_output.expanduser().resolve()
            if args.trigger_events_output is not None
            else raw_path.with_name(f"{raw_path.stem}_trigger_events.csv")
        )
        if output_path.exists():
            raise FileExistsError(f"Trigger-event output already exists: {output_path}")
        count = export_raw_trigger_events(raw_path, output_path)
        print(f"[DONE] Extracted {count} Trigger In events: {output_path}", flush=True)
        return
    if args.trigger_events_output is not None:
        raise ValueError("--trigger-events-output requires --extract-trigger-events")

    if not (1 <= args.clock_port <= 65535):
        raise ValueError("Clock port must be between 1 and 65535")
    if not (1 <= args.discovery_port <= 65535):
        raise ValueError("Discovery port must be between 1 and 65535")
    if args.clock_port == args.discovery_port:
        raise ValueError("Clock and discovery ports must be different")
    if not (1 <= args.coordinator_port <= 65535):
        raise ValueError("Coordinator port must be between 1 and 65535")
    if len({args.clock_port, args.discovery_port, args.coordinator_port}) != 3:
        raise ValueError("Clock, discovery and coordinator ports must be different")
    if args.camera_test_seconds is not None and args.camera_test_seconds <= 0:
        raise ValueError("Camera test duration must be greater than zero")
    if args.trigger_test_seconds is not None and args.trigger_test_seconds <= 0:
        raise ValueError("Trigger test duration must be greater than zero")
    if args.camera_test_seconds is not None and args.trigger_test_seconds is not None:
        raise ValueError("Choose either camera test or trigger test, not both")
    if args.watchdog_grace_seconds < 5:
        raise ValueError("Watchdog grace must be at least 5 seconds")
    if args.minimum_free_gib < 0:
        raise ValueError("Minimum free disk space cannot be negative")

    stop_event = threading.Event()

    clock_thread = threading.Thread(
        target=serve_clock,
        args=(args.host, args.clock_port, stop_event),
        daemon=True,
        name="clock-responder",
    )

    discovery_thread = threading.Thread(
        target=serve_discovery,
        args=(
            args.host,
            args.discovery_port,
            args.clock_port,
            stop_event,
        ),
        daemon=True,
        name="discovery-responder",
    )
    coordinator = LattePandaRunCoordinator(
        output_dir=args.recordings_dir.expanduser().resolve(),
        camera=args.camera,
        bias_file=(None if args.bias_file is None else args.bias_file.expanduser().resolve()),
        camera_enabled=(
            args.camera_recording
            or args.camera_test_seconds is not None
            or args.trigger_test_seconds is not None
        ),
        serial_enabled=(
            args.trigger
            and (args.camera_test_seconds is None or args.trigger_test_seconds is not None)
        ),
        serial_port=args.serial,
        baudrate=args.baud,
        watchdog_grace_s=args.watchdog_grace_seconds,
        minimum_free_bytes=round(args.minimum_free_gib * 1024 ** 3),
    )

    if args.camera_test_seconds is not None:
        try:
            paths = coordinator.camera_self_test(args.camera_test_seconds)
            print("[TEST] PASS - files finalized:", flush=True)
            for path in paths:
                print(f"[TEST]   {path}", flush=True)
        finally:
            coordinator.shutdown()
        return

    if args.trigger_test_seconds is not None:
        try:
            paths = coordinator.camera_trigger_self_test(args.trigger_test_seconds)
            print("[TEST] PASS - trigger-test files finalized:", flush=True)
            for path in paths:
                print(f"[TEST]   {path}", flush=True)
        finally:
            coordinator.shutdown()
        return

    coordinator_thread = threading.Thread(
        target=serve_run_coordinator,
        args=(args.host, args.coordinator_port, stop_event, coordinator),
        daemon=True,
        name="run-coordinator",
    )

    clock_thread.start()
    discovery_thread.start()
    coordinator_thread.start()

    print()
    print("LattePanda synchronisation service running.", flush=True)
    print(f"Hostname:       {socket.gethostname()}", flush=True)
    print(f"Clock UDP:      {args.clock_port}", flush=True)
    print(f"Discovery UDP:  {args.discovery_port}", flush=True)
    print(f"Coordinator UDP:{args.coordinator_port}", flush=True)
    print(
        f"EVK4 recording: {'ENABLED' if args.camera_recording else 'disabled'}",
        flush=True,
    )
    print(f"T0 serial write:{'ENABLED' if args.trigger else 'disabled'}", flush=True)
    print("Press Ctrl+C to stop.", flush=True)
    print()

    try:
        while not stop_event.wait(1.0):
            pass
    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
        stop_event.set()

    coordinator.shutdown()

    clock_thread.join(timeout=2.0)
    discovery_thread.join(timeout=2.0)
    coordinator_thread.join(timeout=2.0)



if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        if len(sys.argv) == 1:
            input("\nStartup failed. Press Enter to close...")
        raise SystemExit(1)
