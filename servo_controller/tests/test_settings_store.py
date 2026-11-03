"""Regression tests: Versioned settings shape and file round trips. Hardware
connection and motion permission must remain outside persistence.

Run from code/servo_controller with python -m unittest discover -s tests -v.
Use the project virtual environment; preserve these regression contracts when
refactoring the implementation described in README_DETAILED.md.
"""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from servo_control.settings_store import default_settings_path, migrate_legacy_settings, read_settings, write_settings


class SettingsStoreTests(unittest.TestCase):
    def test_default_path_is_beside_app(self) -> None:
        self.assertEqual(default_settings_path(), Path(__file__).resolve().parents[1] / "settings.json")

    def test_legacy_settings_copied_without_overwriting_local_settings(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            legacy = base / "ServoTestRig" / "settings.json"
            local = base / "project" / "settings.json"
            write_settings(legacy, {"connection": {"port": "COM9"}})
            with patch.dict("os.environ", {"APPDATA": str(base)}):
                migrate_legacy_settings(local)
                self.assertEqual(read_settings(local), read_settings(legacy))
                write_settings(local, {"connection": {"port": "COM8"}})
                migrate_legacy_settings(local)
            self.assertEqual(read_settings(local)["connection"]["port"], "COM8")
            self.assertTrue(legacy.exists())

    def test_linux_legacy_settings_use_home_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            legacy = home / ".config" / "ServoTestRig" / "settings.json"
            write_settings(legacy, {})
            local = home / "project" / "settings.json"
            with patch.dict("os.environ", {"APPDATA": ""}), patch("pathlib.Path.home", return_value=home):
                migrate_legacy_settings(local)
            self.assertEqual(read_settings(local)["version"], 1)

    def test_json_settings_round_trip_adds_version(self) -> None:
        payload = {
            "connection": {"mode": "Simulation", "port": "COM6"},
            "motion": {"operator_limit_rpm": 100.0},
            "simulator": {"sample_rate_hz": 10.0},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            write_settings(path, payload)
            loaded = read_settings(path)
        self.assertEqual(loaded["version"], 1)
        self.assertEqual(loaded["connection"]["port"], "COM6")

    def test_non_object_settings_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text("[]", encoding="utf-8")
            with self.assertRaises(ValueError):
                read_settings(path)


if __name__ == "__main__":
    unittest.main()
