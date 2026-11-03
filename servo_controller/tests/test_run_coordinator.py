"""Regression tests: Run protocol/state transitions and scheduling with controlled
clock inputs. This tests message semantics, not real network latency or camera
readiness.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import unittest

from servo_control.run_coordinator import PROTOCOL, RunCoordinator


class RunCoordinatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now_ns = 20_000_000_000
        self.coordinator = RunCoordinator(lambda: self.now_ns)

    def request(self, action: str, **values: object) -> dict[str, object]:
        return self.coordinator.handle({
            "protocol": PROTOCOL,
            "request_id": "request-1",
            "action": action,
            **values,
        })

    def test_arm_selects_future_t0_after_preroll_and_margin(self) -> None:
        response = self.request(
            "ARM", run_id="run-1", profile_name="profile.csv",
            expected_duration_s=12.0, pre_roll_s=3.0, preparation_margin_s=1.0,
        )
        self.assertTrue(response["ok"])
        self.assertEqual(response["state"], "ARMED")
        self.assertEqual(response["lattepanda_t0_monotonic_ns"], 24_000_000_000)

    def test_duplicate_arm_is_idempotent(self) -> None:
        first = self.request("ARM", run_id="run-1", pre_roll_s=3.0, preparation_margin_s=1.0)
        self.now_ns += 500_000_000
        second = self.request("ARM", run_id="run-1", pre_roll_s=3.0, preparation_margin_s=1.0)
        self.assertEqual(first["lattepanda_t0_monotonic_ns"], second["lattepanda_t0_monotonic_ns"])

    def test_state_changes_to_running_at_t0(self) -> None:
        self.request("ARM", run_id="run-1", pre_roll_s=3.0, preparation_margin_s=1.0)
        self.now_ns = 24_000_000_000
        response = self.request("STATUS", run_id="run-1")
        self.assertEqual(response["state"], "RUNNING")

    def test_other_run_is_rejected_while_busy(self) -> None:
        self.request("ARM", run_id="run-1", pre_roll_s=3.0, preparation_margin_s=1.0)
        response = self.request("ARM", run_id="run-2", pre_roll_s=3.0, preparation_margin_s=1.0)
        self.assertFalse(response["ok"])
        self.assertIn("busy", str(response["error"]))

    def test_cancel_is_terminal(self) -> None:
        self.request("ARM", run_id="run-1", pre_roll_s=3.0, preparation_margin_s=1.0)
        response = self.request("CANCEL", run_id="run-1")
        self.assertEqual(response["state"], "CANCELLED")
        status = self.request("STATUS", run_id="run-1")
        self.assertEqual(status["state"], "CANCELLED")

    def test_default_zero_preroll_keeps_one_second_margin(self) -> None:
        response = self.request("ARM", run_id="run-1")
        self.assertTrue(response["ok"])
        self.assertEqual(response["pre_roll_s"], 0.0)
        self.assertEqual(response["preparation_margin_s"], 1.0)
        self.assertEqual(response["lattepanda_t0_monotonic_ns"], 21_000_000_000)

    def test_rejects_negative_preroll(self) -> None:
        response = self.request("ARM", run_id="run-1", pre_roll_s=-0.1, preparation_margin_s=1.0)
        self.assertFalse(response["ok"])
        self.assertIn("nonnegative", str(response["error"]))

    def test_complete_releases_coordinator_for_next_run(self) -> None:
        self.request("ARM", run_id="run-1", pre_roll_s=3.0, preparation_margin_s=1.0)
        response = self.request("COMPLETE", run_id="run-1")
        self.assertEqual(response["state"], "COMPLETE")
        next_run = self.request("ARM", run_id="run-2", pre_roll_s=3.0, preparation_margin_s=1.0)
        self.assertTrue(next_run["ok"])


if __name__ == "__main__":
    unittest.main()
