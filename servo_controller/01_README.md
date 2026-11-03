# Servo controller and event-camera tools

This folder contains three connected workflows: control and record the rotary rig, prepare and visualise recordings, then evaluate event-based visual place recognition (VPR).

## Folder structure

```text
servo_controller/
|-- 01_README.md                         Start here: structure and workflow
|-- 02_GUI_README.md                     GUI and LattePanda recording guide
|-- 03_VPR_MEDIA_README.md               Preparation and visualisation guide
|-- 04_SAD_VPR_README.md                 Recognition and evaluation guide
|-- RIG_EVENT_EVALUATION_README.md    Short pointer to the evaluation section
|-- THIRD_PARTY_VPR.md                Research attribution and upstream licence
|-- app.py                            Motor-PC GUI launcher
|-- lattepanda_clock_responder.py     Standalone LattePanda recording service
|-- settings.json                     Saved GUI configuration
|-- requirements.txt                 GUI/hardware/media Python dependencies
|-- requirements-vpr.txt             Offline numerical-analysis dependencies
|-- servo_control/                   GUI, drive, measurement and scheduling modules
|-- vpr_media.py                      Main preparation and media command
|-- raw_timing_preview.py             RAW preview before alignment
|-- prepare_experiment.py             File discovery and preparation helper
|-- generate_vpr_time_bins.py          Encoder-to-camera time-bin helper/CLI
|-- evt3_trigger_extract.py            External-trigger decoder helper
|-- trigger_sync.py                   Trigger clock-fitting helper
|-- build_angle_reference_csv.py       Angle-crossing index helper/CLI
|-- make_angle_video.py                Annotated-video helper/CLI
|-- make_angle_synchronised_video.py   Common-angle comparison helper
|-- make_angle_comparison_video.py    Angle-instance MP4 helper/CLI
|-- make_angle_comparison_gif.py      Angle-instance GIF helper/CLI
|-- steady_revolutions.py             Steady-turn screening command
|-- sad_vpr.py                        Main recognition/evaluation command
|-- rig_vpr/                         Recognition, datasets and result-analysis modules
|-- evaluate_rig.py                   Motion and continuous-event analysis
|-- run_rig_event_pipeline.py          Combined motion/event/pre-motion analysis
|-- run_vpr_window_comparison.py       Paired 1 ms / 2 ms study runner
|-- tests/                           Software regression tests
|-- design/                          Historical handbook-building utility
`-- .venv/                           Local Python environment, if installed
```

Historical design images, diagnostics and the old detailed handbook were moved to `../../archive/servo_controller_cleanup_20261103/`. The retained `design/build_handbook.py` expects the archived handbook sources at their old locations; it is historical documentation tooling, not part of normal operation.

## Choose a task

| I want to... | Start with | Main command |
|---|---|---|
| Run the GUI, command motion or record a new experiment | [GUI and recording](02_GUI_README.md) | `python app.py` on the Motor PC; the responder runs separately on the LattePanda. |
| Align files, inspect RAW recordings, extract views or make comparisons | [VPR Media](03_VPR_MEDIA_README.md) | `python vpr_media.py --help` |
| Measure recognition, compare representations or evaluate motion/event output | [SAD VPR](04_SAD_VPR_README.md) | `python sad_vpr.py --help` |

The three guides include the file maps for their modules, example commands, expected inputs and outputs, and common problems. The former rig-evaluation README is now a short pointer into the SAD guide. [Third-party attribution](THIRD_PARTY_VPR.md) records the original SAD research/tutorial and its licence.

## How the workflows connect

```text
1. GUI + LattePanda service
   -> motor CSV + command events + run metadata + camera RAW/UART metadata

2. VPR Media preparation
   -> experiment_setup.csv + outputs/time_bins/ + outputs/vpr_runs.csv
   -> previews, angle-index CSVs, images and videos

3. SAD VPR
   -> frozen protocol + numeric event-count frames + distances + recall
   -> reference/normalisation/window comparisons + motion/event diagnostics
```

Use the shared `run_id` to associate acquisitions. Human labels such as `run1` identify runs within an experiment; they are not globally unique. Rendered videos and overlays are inspection outputs. SAD extracts numeric counts from RAW recordings rather than reading those videos.

## Installation and command conventions

Use Python 3.10 or newer. The GUI requires Windows Python with Tk. LattePanda camera recording requires a separately installed Metavision SDK environment; pip requirements do not install that SDK. Offline RAW decoding requires a working EventCV backend on the chosen computer.

All guide commands assume your current directory is this `servo_controller` folder. The interpreter must belong to the environment containing the relevant dependencies:

```powershell
# Windows: use an existing project environment.
.\.venv\Scripts\python.exe app.py --simulate
```

```bash
# Linux: use an existing offline-analysis environment.
./.venv/bin/python sad_vpr.py --help
```

`requirements.txt` contains Modbus, serial, LabJack, EventCV, OpenCV and Pillow dependencies. `requirements-vpr.txt` adds the NumPy/SciPy/Matplotlib analysis stack and EventCV. Follow the task guide instead of installing every dependency on every computer. No script automatically transfers recordings between computers.

## Where data belongs

Keep recordings and generated results outside this source folder. The Windows project uses the sibling `../Experiments/` directory. Linux deployments can use the same relative layout or an explicitly supplied path.

```text
code/
|-- servo_controller/       Source and configuration
`-- Experiments/            Acquisition folders and separate analysis directories
```

GUI exports go beside the imported motion-profile CSV, so place that CSV in the intended experiment location. Media preparation moves files within the supplied experiment folder and updates its setup/catalogue; review the [preparation procedure](03_VPR_MEDIA_README.md#3-prepare-an-experiment) before running it on accepted data.

For the actual data layout and completed analyses, see [Experiments](../Experiments/EXPERIMENTS_README.md). Printable targets are described in [Scenes](../Scenes/SCENES_README.md).

The current SAD study is specialised to `Low-Speed`, `Medium-Speed` and `High-Speed`, with three acquisitions named `run1`-`run3` in each. It assumes the recorded 2:1 reduction. The GUI and media tools are more general. Changing the study design requires reviewing the analysis constants and coverage checks, not merely renaming folders.

## Testing

From this directory:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

For one area only:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_controller.py" -v
```

| Tests | Coverage |
|---|---|
| `test_controller.py`, `test_ui_sequence.py` | Drive guards, register access and GUI execution paths. Tk-dependent cases can be skipped if Tk is unavailable. |
| `test_motion_profile.py`, `test_profile_export.py` | CSV parsing, position/speed helpers and run-export schemas. |
| `test_telemetry.py`, `test_settings_store.py`, `test_scene_calibration.py` | Measurements, configuration persistence and voltage conversion. |
| `test_run_coordinator.py`, `test_lattepanda_responder.py` | Protocol and camera/UART lifecycle using substitutes. |
| `test_prepare_experiment.py`, `test_vpr_time_bins.py`, `test_trigger_sync.py`, `test_evt3_trigger_extract.py` | Preparation, time mapping and trigger decoding/fitting. |
| `test_raw_timing_preview.py`, `test_make_angle_video.py`, `test_vpr_media.py` | RAW preview and media workflows. |
| `test_angle_reference_csv.py`, `test_angle_synchronised_video.py`, `test_angle_comparison_gif.py`, `test_steady_revolutions.py` | Angle selection, comparison timing and steady-turn screening. |
| `test_sad_vpr.py`, `test_vpr_window_comparison.py` | Recognition, ranking, normalisation and study consistency. |
| `test_evaluate_rig.py`, `test_rig_event_pipeline.py` | Motion/event metrics and evaluation outputs; plotting tests require Matplotlib. |

These tests do not move the physical rig or establish electrical/mechanical commissioning. Passing mocks is also not evidence that the installed camera SDK, wiring or RAW backend works on the deployment computer.

Local `.venv/`, `__pycache__/` and `servo_gui_crash.log` are environment/runtime files. They can reappear after use and are not source deliverables.
