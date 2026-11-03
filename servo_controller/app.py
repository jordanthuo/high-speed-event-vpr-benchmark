"""Windows entry point for the servo HMI.

Launch this file, rather than ui.py, so the project virtual environment and
crash handlers are installed before Tk and hardware modules are imported.
Importing this module has side effects: it can relaunch the interpreter and
open the crash log. Tests should import servo_control components directly.
CLI page selection changes presentation only; --simulate also connects the
simulator. Physical motion still goes through DriveController's live guards.
"""

import argparse
import faulthandler
import subprocess
import sys
import threading
import traceback
from datetime import datetime
from pathlib import Path


_crash_log_stream = None


def _ensure_project_python() -> None:
    """Relaunch with the project's virtual environment when it is available.

    Windows file associations and ``python app.py`` may otherwise select a
    system interpreter that does not contain the hardware dependencies even
    though they are installed in ``.venv``.
    """
    project_dir = Path(__file__).resolve().parent
    scripts_dir = project_dir / ".venv" / "Scripts"
    executable_name = "pythonw.exe" if Path(sys.executable).name.lower() == "pythonw.exe" else "python.exe"
    project_python = scripts_dir / executable_name

    if not project_python.is_file():
        return

    try:
        current = Path(sys.executable).resolve()
        expected = project_python.resolve()
    except OSError:
        current = Path(sys.executable)
        expected = project_python

    if str(current).casefold() == str(expected).casefold():
        return

    completed = subprocess.run(
        [str(project_python), str(Path(__file__).resolve()), *sys.argv[1:]],
        cwd=str(project_dir),
        check=False,
    )
    raise SystemExit(completed.returncode)


_ensure_project_python()


def _configure_crash_logging() -> Path | None:
    """Persist otherwise invisible pythonw/Tk and worker-thread failures."""
    global _crash_log_stream
    log_path = Path(__file__).resolve().parent / "servo_gui_crash.log"
    try:
        _crash_log_stream = log_path.open("a", encoding="utf-8", buffering=1)
    except OSError:
        return None

    def write_exception(exc_type, exc_value, exc_traceback) -> None:
        assert _crash_log_stream is not None
        _crash_log_stream.write(
            f"\n[{datetime.now().astimezone().isoformat()}] "
            f"Unhandled exception\n"
        )
        traceback.print_exception(
            exc_type,
            exc_value,
            exc_traceback,
            file=_crash_log_stream,
        )
        _crash_log_stream.flush()

    sys.excepthook = write_exception

    def write_thread_exception(args: threading.ExceptHookArgs) -> None:
        write_exception(args.exc_type, args.exc_value, args.exc_traceback)

    threading.excepthook = write_thread_exception
    try:
        faulthandler.enable(file=_crash_log_stream, all_threads=True)
    except (OSError, RuntimeError):
        pass
    return log_path


_configure_crash_logging()

from servo_control.ui import ServoControllerApp


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Delta ASDA-B3 event-camera rig HMI")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="start in Simulation mode and connect automatically",
    )
    parser.add_argument(
        "--page",
        choices=ServoControllerApp.NAV_ITEMS,
        default="Dashboard",
        help="page to show when the application opens",
    )
    args = parser.parse_args()
    ServoControllerApp(auto_connect_simulation=args.simulate, initial_page=args.page).run()
