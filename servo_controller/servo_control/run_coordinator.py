"""UDP protocol for scheduling a LattePanda-authoritative test T0.

The Motor-PC client and protocol-only state machine live here. The deployed
camera-aware implementation is LattePandaRunCoordinator in the standalone
responder; do not mistake this lightweight state machine for hardware startup.
ARM retries for an active run retain its T0. request_id matches a UDP exchange;
run_id identifies the full acquisition. Network retries must not restart motion
or choose a second origin after an acknowledgement is lost.
"""

from __future__ import annotations

import json
import socket
import threading
import uuid
from dataclasses import dataclass
from time import perf_counter_ns
from typing import Callable


COORDINATOR_PORT = 49154
PROTOCOL = "STRG_RUN_V1"


@dataclass(frozen=True)
class RunArmAcknowledgement:
    run_id: str
    lattepanda_t0_monotonic_ns: int
    lattepanda_now_monotonic_ns: int
    state: str
    pre_roll_s: float
    preparation_margin_s: float
    camera_enabled: bool
    serial_trigger_enabled: bool
    serial_trigger_healthy: bool
    camera_healthy: bool
    camera_files: tuple[str, ...]


@dataclass(frozen=True)
class CoordinatorRun:
    run_id: str
    profile_name: str
    expected_duration_s: float | None
    pre_roll_s: float
    preparation_margin_s: float
    armed_at_ns: int
    t0_ns: int

    def state_at(self, now_ns: int) -> str:
        return "ARMED" if now_ns < self.t0_ns else "RUNNING"


class RunCoordinator:
    """Thread-safe LattePanda run state machine, independent of networking."""

    def __init__(self, clock_ns: Callable[[], int] = perf_counter_ns) -> None:
        self._clock_ns = clock_ns
        self._lock = threading.Lock()
        self._run: CoordinatorRun | None = None
        self._terminal: dict[str, str] = {}

    def handle(self, request: dict[str, object]) -> dict[str, object]:
        if request.get("protocol") != PROTOCOL:
            return self._error(request, "unsupported protocol")
        action = str(request.get("action", "")).upper()
        try:
            if action == "ARM":
                return self._arm(request)
            if action == "STATUS":
                return self._status(request)
            if action == "CANCEL":
                return self._cancel(request)
            if action == "COMPLETE":
                return self._finish(request, "COMPLETE")
            return self._error(request, f"unsupported action {action!r}")
        except (TypeError, ValueError) as exc:
            return self._error(request, str(exc))

    def _arm(self, request: dict[str, object]) -> dict[str, object]:
        run_id = str(request.get("run_id", "")).strip()
        if not run_id:
            raise ValueError("run_id is required")
        pre_roll_s = float(request.get("pre_roll_s", 0.0))
        margin_s = float(request.get("preparation_margin_s", 1.0))
        duration_value = request.get("expected_duration_s")
        duration_s = None if duration_value is None else float(duration_value)
        if pre_roll_s < 0.0:
            raise ValueError("pre_roll_s must be nonnegative")
        if margin_s < 0.25:
            raise ValueError("preparation_margin_s must be at least 0.25 seconds")
        if duration_s is not None and duration_s <= 0:
            raise ValueError("expected_duration_s must be greater than zero")
        now_ns = self._clock_ns()
        with self._lock:
            if self._run is not None:
                if self._run.run_id != run_id:
                    return self._error(request, f"coordinator busy with run {self._run.run_id}")
                run = self._run
            else:
                run = CoordinatorRun(
                    run_id=run_id,
                    profile_name=str(request.get("profile_name", "")),
                    expected_duration_s=duration_s,
                    pre_roll_s=pre_roll_s,
                    preparation_margin_s=margin_s,
                    armed_at_ns=now_ns,
                    t0_ns=now_ns + round((pre_roll_s + margin_s) * 1_000_000_000),
                )
                self._run = run
                self._terminal.pop(run_id, None)
        return self._response(request, True, run.state_at(now_ns), now_ns, run)

    def _status(self, request: dict[str, object]) -> dict[str, object]:
        run_id = str(request.get("run_id", "")).strip()
        now_ns = self._clock_ns()
        with self._lock:
            run = self._run
            terminal = self._terminal.get(run_id)
        if run is not None and run.run_id == run_id:
            return self._response(request, True, run.state_at(now_ns), now_ns, run)
        if terminal is not None:
            return self._response(request, True, terminal, now_ns, None, run_id)
        return self._error(request, f"unknown run {run_id!r}")

    def _cancel(self, request: dict[str, object]) -> dict[str, object]:
        return self._finish(request, "CANCELLED")

    def _finish(self, request: dict[str, object], terminal_state: str) -> dict[str, object]:
        run_id = str(request.get("run_id", "")).strip()
        now_ns = self._clock_ns()
        with self._lock:
            run = self._run
            if run is None or run.run_id != run_id:
                return self._error(request, f"unknown run {run_id!r}")
            self._run = None
            self._terminal[run_id] = terminal_state
        return self._response(request, True, terminal_state, now_ns, run)

    @staticmethod
    def _response(
        request: dict[str, object], ok: bool, state: str, now_ns: int,
        run: CoordinatorRun | None, run_id: str | None = None,
    ) -> dict[str, object]:
        return {
            "protocol": PROTOCOL,
            "request_id": request.get("request_id"),
            "ok": ok,
            "run_id": run.run_id if run is not None else run_id,
            "state": state,
            "lattepanda_now_monotonic_ns": now_ns,
            "lattepanda_t0_monotonic_ns": None if run is None else run.t0_ns,
            "pre_roll_s": None if run is None else run.pre_roll_s,
            "preparation_margin_s": None if run is None else run.preparation_margin_s,
        }

    @staticmethod
    def _error(request: dict[str, object], message: str) -> dict[str, object]:
        return {
            "protocol": PROTOCOL,
            "request_id": request.get("request_id"),
            "ok": False,
            "state": "FAILED",
            "error": message,
        }


class UdpRunCoordinatorClient:
    def __init__(
        self, host: str, port: int = COORDINATOR_PORT,
        timeout_s: float = 1.0, retries: int = 3,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.retries = retries

    def arm(
        self, run_id: str, profile_name: str, expected_duration_s: float | None,
        pre_roll_s: float = 0.0, preparation_margin_s: float = 1.0,
    ) -> RunArmAcknowledgement:
        response = self._exchange({
            "action": "ARM",
            "run_id": run_id,
            "profile_name": profile_name,
            "expected_duration_s": expected_duration_s,
            "pre_roll_s": pre_roll_s,
            "preparation_margin_s": preparation_margin_s,
        })
        return RunArmAcknowledgement(
            run_id=str(response["run_id"]),
            lattepanda_t0_monotonic_ns=int(response["lattepanda_t0_monotonic_ns"]),
            lattepanda_now_monotonic_ns=int(response["lattepanda_now_monotonic_ns"]),
            state=str(response["state"]),
            pre_roll_s=float(response["pre_roll_s"]),
            preparation_margin_s=float(response["preparation_margin_s"]),
            camera_enabled=bool(response.get("camera_enabled", False)),
            serial_trigger_enabled=bool(response.get("serial_trigger_enabled", False)),
            serial_trigger_healthy=bool(response.get("serial_trigger_healthy", False)),
            camera_healthy=bool(response.get("camera_healthy", False)),
            camera_files=tuple(str(item) for item in response.get("camera_files", [])),
        )

    def status(self, run_id: str) -> dict[str, object]:
        return self._exchange({"action": "STATUS", "run_id": run_id})

    def cancel(self, run_id: str) -> dict[str, object]:
        return self._exchange({"action": "CANCEL", "run_id": run_id})

    def complete(self, run_id: str) -> dict[str, object]:
        return self._exchange({"action": "COMPLETE", "run_id": run_id})

    def _exchange(self, payload: dict[str, object]) -> dict[str, object]:
        request_id = uuid.uuid4().hex
        request = {"protocol": PROTOCOL, "request_id": request_id, **payload}
        packet = json.dumps(request, separators=(",", ":")).encode("utf-8")
        if self.retries < 1:
            raise ValueError("retries must be at least 1")
        response_packet = None
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout_s)
            for _attempt in range(self.retries):
                sock.sendto(packet, (self.host, self.port))
                try:
                    response_packet, _address = sock.recvfrom(8192)
                    break
                except socket.timeout:
                    continue
        if response_packet is None:
            raise TimeoutError(
                f"LattePanda run coordinator did not reply after {self.retries} attempts"
            )
        response = json.loads(response_packet.decode("utf-8"))
        if response.get("protocol") != PROTOCOL or response.get("request_id") != request_id:
            raise RuntimeError("invalid run-coordinator response")
        if not response.get("ok"):
            raise RuntimeError(str(response.get("error", "run coordinator rejected request")))
        return response


def serve_run_coordinator(
    host: str, port: int, stop_event: threading.Event,
    coordinator: RunCoordinator | None = None,
) -> None:
    coordinator = coordinator or RunCoordinator()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        sock.settimeout(0.5)
        print(f"[RUN] Listening on UDP {host}:{port}", flush=True)
        while not stop_event.is_set():
            try:
                packet, address = sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                if stop_event.is_set():
                    break
                continue
            try:
                request = json.loads(packet.decode("utf-8"))
                if not isinstance(request, dict):
                    raise ValueError("request must be a JSON object")
                response = coordinator.handle(request)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                response = {"protocol": PROTOCOL, "ok": False, "state": "FAILED", "error": str(exc)}
            sock.sendto(json.dumps(response, separators=(",", ":")).encode("utf-8"), address)
