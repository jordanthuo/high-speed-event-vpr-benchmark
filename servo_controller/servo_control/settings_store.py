"""Versioned JSON persistence used by the GUI settings workflow.

This layer checks document shape/version and performs temporary-file replace;
field validation/application belongs to ui.py. Saving configuration must not
persist live motion permission or implicitly connect hardware. Replacement
reduces partial-write exposure but is not a backup/version history.
"""

import json
import os
from pathlib import Path
from typing import Any


SETTINGS_VERSION = 1


def default_settings_path() -> Path:
    """Keep configuration with the application, regardless of launch directory."""
    return Path(__file__).resolve().parent.parent / "settings.json"


def migrate_legacy_settings(destination: Path) -> None:
    """Copy valid user-profile settings once; retain the original as a backup."""
    if destination.exists():
        return
    base = Path(os.environ.get("APPDATA") or Path.home() / ".config")
    legacy = base / "ServoTestRig" / "settings.json"
    if legacy.exists():
        write_settings(destination, read_settings(legacy))


def read_settings(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError("Settings file must contain a JSON object")
    if payload.get("version") != SETTINGS_VERSION:
        raise ValueError(f"Unsupported settings version: {payload.get('version')!r}")
    return payload


def write_settings(path: str | Path, payload: dict[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    document = dict(payload)
    document["version"] = SETTINGS_VERSION
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")
    temporary.replace(destination)
