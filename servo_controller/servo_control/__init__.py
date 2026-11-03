"""Servo rig application package.

Start at ui.py for operator workflows, controller.py for guarded drive access,
and transport.py for register I/O. Offline EVK4 tools live beside app.py.
Keep hardware startup out of this package initializer so importing parsers,
models and tests does not connect to the rig.
"""

from .config import SerialConfig
from .controller import DriveController

__all__ = ["DriveController", "SerialConfig"]

