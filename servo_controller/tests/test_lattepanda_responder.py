"""Regression tests: Standalone responder lifecycle with hardware substitutes.
Camera/trigger finalization and health failures are tested without opening the
commissioned devices.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import tempfile
import json
import socket
import sys
import threading
import time
from types import SimpleNamespace
import unittest
import csv
from pathlib import Path
from unittest.mock import patch

import lattepanda_clock_responder as responder
from servo_control.run_coordinator import UdpRunCoordinatorClient


class FakeCamera:
    def __init__(self, output_dir, camera, bias_file, minimum_free_bytes=0):
        self.output_dir = output_dir
        self.paths = None
        self.t0_ns = None
        self.recorded_writes = []
        self.is_healthy = True
        self.health_reason = "healthy"
        self.is_recording = False

    def start_recording(self, run_id):
        self.is_recording = True
        self.paths = (
            self.output_dir / f"{run_id}.raw",
            self.output_dir / f"{run_id}_0x00_writes.csv",
            self.output_dir / f"{run_id}_trigger_events.csv",
            self.output_dir / f"{run_id}_metadata.json",
        )
        return self.paths

    def set_t0(self, t0_ns):
        self.t0_ns = t0_ns

    def record_write(self, *values):
        self.recorded_writes.append(values)

    def stop_recording(self, reason="requested"):
        self.is_recording = False

    def close(self):
        pass


class FakeSerialTrigger:
    def __init__(self, port, baudrate):
        self.port = port
        self.is_healthy = False
        self.health_reason = "not open"
        self.is_active = False

    def open(self):
        self.is_healthy = True
        self.health_reason = "healthy"

    def arm(self, t0_ns, record_write):
        self.is_active = True
        record_write(0, t0_ns, t0_ns, t0_ns + 200_000, "completed", "")

    def stop_run(self):
        self.is_active = False

    def close(self):
        self.is_healthy = False
        self.is_active = False


class LattePandaResponderTests(unittest.TestCase):
    def request(self, action, **values):
        return {
            "protocol": responder.RUN_PROTOCOL,
            "request_id": f"request-{action}",
            "action": action,
            **values,
        }

    def test_single_file_coordinator_preopens_camera_and_returns_compact_response(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            responder, "Evk4ArmedCamera", FakeCamera
        ), patch.object(responder, "ScheduledSerialTrigger", FakeSerialTrigger):
            coordinator = responder.LattePandaRunCoordinator(
                output_dir=Path(directory),
                camera="",
                bias_file=None,
                camera_enabled=True,
                serial_enabled=True,
                serial_port="COM1",
                baudrate=115200,
            )
            armed = coordinator.handle(self.request(
                "ARM",
                run_id="run-1",
                profile_name="profile.csv",
                expected_duration_s=2.0,
                pre_roll_s=0.0,
                preparation_margin_s=1.0,
            ))
            self.assertTrue(armed["ok"])
            self.assertEqual(armed["pre_roll_s"], 0.0)
            self.assertEqual(armed["preparation_margin_s"], 1.0)
            self.assertTrue(armed["camera_enabled"])
            self.assertTrue(armed["serial_trigger_enabled"])
            self.assertEqual(len(armed["camera_files"]), 4)
            self.assertNotIn("events", armed)

            completed = coordinator.handle(self.request("COMPLETE", run_id="run-1"))
            self.assertEqual(completed["state"], "COMPLETE")
            self.assertNotIn("events", completed)
            duplicate = coordinator.handle({
                **self.request("COMPLETE", run_id="run-1"),
                "request_id": "retry-complete",
            })
            self.assertEqual(duplicate["state"], "COMPLETE")
            self.assertEqual(duplicate["request_id"], "retry-complete")

    def test_watchdog_finalizes_run_and_caches_terminal_state(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            responder, "Evk4ArmedCamera", FakeCamera
        ), patch.object(responder, "ScheduledSerialTrigger", FakeSerialTrigger):
            coordinator = responder.LattePandaRunCoordinator(
                output_dir=Path(directory), camera="", bias_file=None,
                camera_enabled=True, serial_enabled=True,
                serial_port="COM1", baudrate=115200,
            )
            armed = coordinator.handle(self.request(
                "ARM", run_id="watchdog-run", profile_name="profile.csv",
                expected_duration_s=2.0, pre_roll_s=3.0, preparation_margin_s=1.0,
            ))
            self.assertTrue(armed["ok"])
            coordinator._expire_watchdog("watchdog-run")
            status = coordinator.handle(self.request("STATUS", run_id="watchdog-run"))
            self.assertEqual(status["state"], "WATCHDOG_TIMEOUT")
            self.assertFalse(status["camera_recording_active"])

    def test_unhealthy_camera_rejects_arm(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            responder, "Evk4ArmedCamera", FakeCamera
        ):
            coordinator = responder.LattePandaRunCoordinator(
                output_dir=Path(directory), camera="", bias_file=None,
                camera_enabled=True, serial_enabled=False,
                serial_port="COM1", baudrate=115200,
            )
            coordinator._camera_recorder.is_healthy = False
            coordinator._camera_recorder.health_reason = "USB disconnected"
            response = coordinator.handle(self.request(
                "ARM", run_id="bad-camera", profile_name="profile.csv",
                expected_duration_s=2.0, pre_roll_s=3.0, preparation_margin_s=1.0,
            ))
            self.assertFalse(response["ok"])
            self.assertIn("USB disconnected", response["error"])

    def test_script_has_no_project_package_dependency(self):
        source = Path(responder.__file__).read_text(encoding="utf-8")
        self.assertNotIn("from servo_control", source)
        self.assertNotIn("import servo_control", source)

    def test_trigger_events_are_exported_from_raw_reader(self):
        readers = []

        class FakeReader:
            def __init__(self, _path, max_events, do_time_shifting):
                self.calls = 0
                self.max_events = max_events
                self.do_time_shifting = do_time_shifting
                self.delta_t_values = []
                readers.append(self)

            def is_done(self):
                return self.calls >= 2

            def load_delta_t(self, delta):
                self.delta_t_values.append(delta)
                self.calls += 1

            def get_ext_trigger_events(self):
                if self.calls == 1:
                    return [
                        {"t": 1000, "p": 0, "id": 0},
                        {"t": 1100, "p": 1, "id": 0},
                    ]
                return [{"t": 2000, "p": 0, "id": 0}]

            def clear_ext_trigger_events(self):
                pass

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "test.raw"
            raw.write_bytes(b"raw")
            output = root / "test_trigger_events.csv"
            count = responder.export_raw_trigger_events(raw, output, FakeReader)
            with output.open("r", newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(count, 3)
        self.assertEqual(rows[0]["raw_timestamp_us"], "1000")
        self.assertEqual(rows[-1]["polarity"], "0")
        self.assertEqual(readers[0].max_events, responder.TRIGGER_EXTRACTION_MAX_EVENTS)
        self.assertEqual(
            readers[0].delta_t_values,
            [responder.TRIGGER_EXTRACTION_CHUNK_US] * 2,
        )

    def test_legacy_bias_file_is_applied_and_read_back(self):
        class FakeBiases:
            def __init__(self):
                self.values = {}

            def set(self, name, value):
                self.values[name] = value
                return True

            def get(self, name):
                return self.values[name]

        biases = FakeBiases()
        device = SimpleNamespace(get_i_ll_biases=lambda: biases)
        with tempfile.TemporaryDirectory() as directory:
            bias_path = Path(directory) / "camera.bias"
            bias_path.write_text(
                "0 % bias_diff\n120 % bias_diff_on # tuned value\n",
                encoding="utf-8",
            )
            applied = responder.load_bias_file(device, bias_path)

        self.assertEqual(applied, {"bias_diff": 0, "bias_diff_on": 120})

    def test_real_trigger_scheduler_writes_at_t0_then_one_hz(self):
        writes = []
        events = []

        class FakeUart:
            def write(self, value):
                writes.append((responder.perf_counter_ns(), value))
                return len(value)

            def flush(self):
                pass

            def close(self):
                pass

        serial_module = SimpleNamespace(
            EIGHTBITS=8,
            PARITY_NONE="N",
            STOPBITS_ONE=1,
            Serial=lambda **_kwargs: FakeUart(),
        )

        def record_write(sequence, scheduled_ns, requested_ns, completed_ns, status, error):
            events.append({
                "sequence": sequence,
                "scheduled_ns": scheduled_ns,
                "requested_ns": requested_ns,
                "completed_ns": completed_ns,
                "status": status,
                "error": error,
            })

        with patch.dict(sys.modules, {"serial": serial_module}):
            trigger = responder.ScheduledSerialTrigger("COM1", 115200)
            trigger.open()
            t0_ns = responder.perf_counter_ns() + 50_000_000
            trigger.arm(t0_ns, record_write)
            time.sleep(1.18)
            trigger.close()

        self.assertEqual([value for _timestamp, value in writes], [b"\x00", b"\x00"])
        self.assertEqual([event["sequence"] for event in events], [0, 1])
        self.assertEqual(
            events[1]["scheduled_ns"] - events[0]["scheduled_ns"],
            1_000_000_000,
        )

    def test_trigger_write_failure_marks_uart_unhealthy(self):
        events = []

        class FailingUart:
            is_open = True

            def write(self, _value):
                raise IOError("simulated COM1 failure")

            def close(self):
                self.is_open = False

        serial_module = SimpleNamespace(
            EIGHTBITS=8,
            PARITY_NONE="N",
            STOPBITS_ONE=1,
            Serial=lambda **_kwargs: FailingUart(),
        )

        with patch.dict(sys.modules, {"serial": serial_module}):
            trigger = responder.ScheduledSerialTrigger("COM1", 115200)
            trigger.open()
            trigger.arm(
                responder.perf_counter_ns() + 10_000_000,
                lambda *values: events.append(values),
            )
            time.sleep(0.05)
            self.assertFalse(trigger.is_healthy)
            self.assertIn("simulated COM1 failure", trigger.health_reason)
            self.assertEqual(events[0][4], "failed")
            trigger.close()

    def test_motor_pc_client_completes_udp_workflow_against_standalone_service(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            responder, "Evk4ArmedCamera", FakeCamera
        ), patch.object(responder, "ScheduledSerialTrigger", FakeSerialTrigger):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            coordinator = responder.LattePandaRunCoordinator(
                output_dir=Path(directory), camera="", bias_file=None,
                camera_enabled=True, serial_enabled=True,
                serial_port="COM1", baudrate=115200,
            )
            stop_event = threading.Event()
            ready_event = threading.Event()
            thread = threading.Thread(
                target=responder.serve_run_coordinator,
                args=("127.0.0.1", port, stop_event, coordinator, ready_event),
                daemon=True,
            )
            thread.start()
            self.assertTrue(ready_event.wait(2.0))
            try:
                client = UdpRunCoordinatorClient("127.0.0.1", port=port)
                acknowledgement = client.arm("udp-run", "profile.csv", 2.0)
                self.assertEqual(acknowledgement.state, "ARMED")
                self.assertTrue(acknowledgement.camera_enabled)
                self.assertTrue(acknowledgement.serial_trigger_enabled)
                self.assertTrue(acknowledgement.serial_trigger_healthy)
                self.assertTrue(acknowledgement.camera_healthy)
                status = client.status("udp-run")
                self.assertTrue(status["camera_recording_active"])
                completed = client.complete("udp-run")
                self.assertEqual(completed["state"], "COMPLETE")
                self.assertNotIn("events", completed)
            finally:
                stop_event.set()
                thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
