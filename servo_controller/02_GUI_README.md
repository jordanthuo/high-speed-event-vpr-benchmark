# GUI and coordinated recording

Use this guide to run the Motor-PC interface and the LattePanda camera service. Start with [the folder map](01_README.md) if you are unfamiliar with the project.

## 1. What runs where

| Computer | Entry point | Role |
|---|---|---|
| Motor PC, Windows | `app.py` | GUI, servo commands, motor telemetry, optional scene-height measurements and experiment scheduling. |
| LattePanda, Metavision environment | `lattepanda_clock_responder.py` | EVK4 RAW recording, UART trigger writes, clock replies and run coordination. |
| Offline processing computer | `vpr_media.py` / `sad_vpr.py` | Preparation, visualisation and recognition; see the other two guides. |

The Motor PC talks to the servo drive over Modbus RTU and to the LattePanda over UDP. The LattePanda records locally. The GUI does not automatically copy camera files back to the Motor PC.

## 2. Install and launch the GUI

Run these PowerShell commands from `code/servo_controller`. Create the environment only if `.venv` does not already exist:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Windows Python must include Tk. Live drive access also needs the USB-to-RS485 driver; the optional LabJack needs its native driver. Metavision is installed separately on the LattePanda.

```powershell
# Open the simulator and connect it automatically.
.\.venv\Scripts\python.exe app.py --simulate

# Open the simulator directly on the Tests page.
.\.venv\Scripts\python.exe app.py --simulate --page Tests

# Open normally; choose the connection mode in the interface.
.\.venv\Scripts\python.exe app.py
```

`python app.py` also works: when a local Windows `.venv` exists, the launcher relaunches itself with that interpreter. Simulation uses the standard-library controller and Tk; it does not require a connected drive or camera.

## 3. Find your way around

| Page | Use it for |
|---|---|
| Dashboard | Connect, enable live manual control, issue speed/position commands, return towards the session zero, and monitor feedback/status. |
| Motion | Apply operator limits and ramps; inspect or configure digital I/O. |
| Tests | Import, stage and execute a CSV profile; inspect the preview and recorded feedback. |
| Trends | Inspect speed and position history and export the retained buffer. Pausing a chart does not stop motion. |
| Parameters | Read the project register catalogue and perform guarded parameter writes. |
| Logs | Inspect and export application events. |
| Settings | Set serial communication, motor/platform ratio, simulator settings, LabJack calibration and clock-service address. Import/export settings JSON. |

The current physical CSV workflow uses **Speed mode**, including position waypoints approached using encoder feedback. The retained PR compiler is not the live execution path.

## 4. Connection and settings

`settings.json` is stored beside `app.py`, regardless of the launch directory. Saved settings override source defaults. Importing settings does not connect the drive or grant live-motion permission.

| Setting | Source default / meaning |
|---|---|
| Servo serial link | COM6, slave 1, 115200 baud, 8-N-2, 1 s timeout. Check against the installed drive. |
| Motor/platform ratio | 2:1. Commands use motor rpm; derived platform speed and angle divide by the ratio. |
| Operator speed limit | Source default is 100 motor rpm; saved settings can differ. Check the applied value before use. |
| Clock service | Host is configured in Settings; UDP 49152. Discovery uses UDP 49153. |
| Run coordinator | UDP 49154. |

For physical operation, check the commissioned direction, clearance, protection settings and wired stop circuit. Live-control permission is granted per connection. A software zero-speed request is not an emergency stop or proof that torque has been removed.

## 5. Prepare the LattePanda

Copy `lattepanda_clock_responder.py` to the LattePanda. It is standalone and does not need the `servo_control/` package. Use the Python environment configured for the Metavision SDK, install `pyserial` there, and close Metavision Studio before opening the camera.

The following paths are examples; replace them with the deployed bias file and recording directory:

```powershell
python -m pip install pyserial
python lattepanda_clock_responder.py --help

# Camera-only recording check; no servo commands are sent.
python lattepanda_clock_responder.py --camera-test-seconds 5 --bias-file "C:\EVK4\test.bias" --recordings-dir "C:\EVK4\recordings"

# Camera plus UART-trigger check; no servo commands are sent.
python lattepanda_clock_responder.py --trigger-test-seconds 5 --bias-file "C:\EVK4\test.bias" --recordings-dir "C:\EVK4\recordings" --serial COM1

# Start the recording/clock/coordinator service.
python lattepanda_clock_responder.py --camera-recording --trigger --bias-file "C:\EVK4\test.bias" --recordings-dir "C:\EVK4\recordings" --serial COM1
```

The service opens the EVK4 and COM1 at startup, applies the biases and enables MAIN Trigger In. Allow the three UDP ports through the relevant firewall. Open test recordings in Studio to check them, then close Studio before restarting the service.

Without arguments, the script uses its deployed `C:\Users\LattePanda\Documents\metavision\...` paths. Override them when deploying elsewhere. `--no-camera-recording` and `--no-trigger` are diagnostic modes; the live GUI profile workflow requires both capabilities.

## 6. Make a motion CSV

### Timed speed commands

Save a file such as `trial.csv` with these exact headers:

```csv
time_s,motor_rpm
0,0
0.1,30
10,0
15,0
```

This requests 30 motor rpm, or 15 platform rpm at a 2:1 ratio. Times specify when setpoints are issued; drive ramps determine the physical response. Use at least two rows, start at time zero and increase time strictly. Physical speed profiles must end with a zero-speed row. The applied operator limit can be lower than the parser's absolute +/-1000 rpm bound.

### Position waypoints

```csv
platform_position_deg,motor_speed_rpm,hold_s
0,0,0
90,5,1
180,5,1
```

Targets are platform angles relative to the run origin. The speed is a nonnegative motor-speed magnitude; movement direction follows the target change and configured command sign. The first position and speed must both be zero. A changed target requires positive speed, and an unchanged target requires zero speed. Holds must be between 0 and 32.767 s. Estimated preview timing is not a measured trajectory.

## 7. Run and collect an experiment

1. Start the LattePanda service and confirm it is ready.
2. Connect the Motor-PC GUI and check the serial, ratio, speed-limit and ramp settings.
3. Start clock exchanges using the configured LattePanda address. Connect the LabJack if required.
4. In Tests, import the CSV and inspect its preview. Select **STAGE / VERIFY** to stage the execution plan; final live-drive checks occur when starting the run.
5. Check run readiness and the physical confirmations, then select Start.
6. Wait for the armed countdown. On ARM, the LattePanda begins RAW logging and selects a future T0. The current GUI requests the default **zero additional pre-roll plus a one-second preparation margin**, not a fixed three-second pre-roll.
7. Monitor the run. The UART writes `0x00` at T0 and every second thereafter. Physical trigger edges are recorded in the RAW stream; software write timestamps are diagnostic timing records.
8. Wait for completion or confirmed abort handling. Copy the camera files back and retain all associated motor exports.

Timed speed rows run on a worker with absolute T0-relative deadlines. A separate timer requests the final zero setpoint; the speed-profile completion path continues acquiring feedback until speed is within 0.5 motor rpm for 0.25 s. Position-profile completion uses its own feedback/waypoint logic.

The Motor PC monitors the camera coordinator. The LattePanda watchdog finalizes recording after `T0 + expected duration + 10 s` if completion/cancellation is not received. That watchdog does not directly control the servo motor.

## 8. Files produced by a run

| Location | Output | Contents |
|---|---|---|
| Beside the imported profile | `<profile>_<timestamp>_recorded.csv` | Motor feedback, timing and optional scene measurements. |
| Same location | `<recorded-stem>_events.csv` | Motor-PC command/release and coordination events. |
| Same location | `<recorded-stem>_metadata.json` | Run ID, source profile, clock mapping, application build and remote finalization information. |
| LattePanda recording directory | `<run-id>_<timestamp>.raw` | Camera events. |
| Same directory | `<run-id>_<timestamp>_0x00_writes.csv` | UART attempts, sequence numbers, deadlines, durations and outcomes. |
| Same directory | `<run-id>_<timestamp>_metadata.json` | Camera session times, applied biases and file association. |

The external-trigger CSV is normally extracted later during trigger-based preparation. Motor event sidecars do not replace the separate UART-write CSV.

Speed-profile exports include `run_id`, `lattepanda_time_s`, `motor_pc_monotonic_ns`, feedback speeds, encoder pulses and platform angles. **Position-profile exports have a different schema and cannot be passed directly into the current VPR preparation loader.** Use [VPR Media](03_VPR_MEDIA_README.md) for supported preparation.

High-rate telemetry timestamps bracket a serial transaction and use its midpoint; they are not hardware-latched event times. Trends uses a bounded buffer, while profile recording retains its own samples. Preserve the run metadata and command-event sidecars when interpreting timing and command changes.

## 9. Optional scene-height measurement

The LabJack worker reads AIN0 independently of the servo link. Settings holds the sample rate, zero voltage and signed conversion slope:

```text
displacement_mm = (voltage_v - zero_voltage_v) * mm_per_volt
```

The project defaults use 3.620 V at 0 mm and 1.671 V at +125 mm, giving a slope of approximately -64.135454 mm/V. Lower voltage indicates greater positive displacement. Voltage and displacement are associated with motor samples from the separate USB history, not sampled simultaneously. Review missing/stale readings and calibration before treating them as measurements.

## 10. Source-file map

These are imported modules, not scripts to launch individually:

| File | Responsibility |
|---|---|
| `servo_control/ui.py` | Tk pages, operator callbacks, workers, profile execution and run exports. |
| `servo_control/controller.py` | Drive operations, register guards, arming, speed commands and readback. |
| `servo_control/transport.py` | Modbus communication, register-word encoding and simulator. |
| `servo_control/registers.py` | Project register addresses, types, limits and intended configuration. |
| `servo_control/config.py` | Serial configuration and validation. |
| `servo_control/models.py` | Status, telemetry and connection-state data structures. |
| `servo_control/motion_profile.py` | CSV parsing, profile representation, previews and waypoint helpers. |
| `servo_control/pr_program.py` | Retained PR compiler/status helpers; still imported, but not the current live CSV path. |
| `servo_control/clock_sync.py` | UDP clock client, offset samples and frozen run-time mapping. |
| `servo_control/run_coordinator.py` | Motor-PC UDP run client and protocol-only coordinator model. The deployed camera-aware server is in the standalone responder. |
| `servo_control/telemetry.py` | Measurement buffers, encoder-counter unwrapping, derived angles and CSV/event exports. |
| `servo_control/labjack_input.py` | Optional AIN0 acquisition via LJM or LabJackPython. |
| `servo_control/scene_calibration.py` | Voltage conversion defaults and migration of the old calibration. |
| `servo_control/settings_store.py` | JSON persistence and migration of older user-profile settings. |
| `servo_control/widgets.py` | Gauges, compasses and plotted Tk widgets. |
| `servo_control/__init__.py` | Package exports. |

## 11. Common problems

| Symptom | Check |
|---|---|
| GUI will not open | Windows Tk installation and the interpreter selected by the local `.venv`. |
| Cannot connect to drive | COM port, slave, baud/parity/stop bits, USB driver and other software holding the port. |
| Start remains unavailable | Drive readiness, SON, EMGS, alarm state, monitor selection, zero feedback, staged profile and fresh clock exchanges. |
| Camera service fails at startup | Metavision environment, bias-file path, Studio still open, camera/COM1 access and free disk space. |
| Coordinator does not respond | Configured host and UDP ports 49152-49154; network/firewall configuration. |
| Recording is incomplete | Both computers' sidecars, remote finalization state, camera health and available storage. |
| GUI exception | Inspect the regenerated `servo_gui_crash.log` beside `app.py`. |

For software regression tests, see [Testing in the main README](01_README.md#testing). Those tests use substitutes and do not commission the physical rig.
