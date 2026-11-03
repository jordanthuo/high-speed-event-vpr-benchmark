# VPR Media: prepare, inspect and compare recordings

This guide takes the files produced by [GUI acquisition](02_GUI_README.md), associates camera time with motor measurements, and produces inspection media. For recognition scores, continue to [SAD VPR](04_SAD_VPR_README.md).

## 1. Files and responsibilities

| File | What it does / how to use it |
|---|---|
| `vpr_media.py` | Main CLI. Prepare experiments, register runs, render videos, query angles and compare views. Use this for normal media work. |
| `raw_timing_preview.py` | Standalone preview when encoder alignment is not ready. Helps identify visible motion onset. |
| `prepare_experiment.py` | Imported by `prepare-experiment`; discovers runs, organises files, writes setup/time bins/catalogue and optionally extracts/fits triggers. No standalone CLI. |
| `generate_vpr_time_bins.py` | Creates camera intervals associated with encoder rows. Imported by preparation; has a lower-level CLI for manual or recording-start anchors. |
| `evt3_trigger_extract.py` | Streams EVT3 clock/trigger words into a trigger CSV and updates metadata. Called by preparation; no standalone CLI. |
| `trigger_sync.py` | Fits the camera/LattePanda clock relationship from UART writes and camera triggers. Called by preparation; no standalone CLI. |
| `build_angle_reference_csv.py` | Finds repeated angle crossings and builds their index. Used by `query-angle`; also has a CLI. |
| `make_angle_video.py` | Shared event rendering, time-bin lookup and video generation. Used by `run-video`; also has a CLI. |
| `make_angle_synchronised_video.py` | Creates two-/three-panel videos indexed by common angle. Used by `compare-runs-by-angle`; imported helper. |
| `make_angle_comparison_video.py` | Creates an MP4 around indexed instances of one angle. Used by `compare-angle`; also has a CLI. |
| `make_angle_comparison_gif.py` | GIF equivalent and playback-duration helper. Used by `compare-angle`; also has a CLI. |
| `steady_revolutions.py` | Reads prepared bins and reports complete steady turns shared between runs. Standalone CLI; does not render or modify data. |

The lower-level CLIs have historical file defaults. Supply your own paths or use `vpr_media.py` with an explicit catalogue.

## 2. Environment and inputs

Examples below use PowerShell from `code/servo_controller`. Use the project environment containing EventCV, OpenCV and Pillow:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe vpr_media.py --help
```

On Linux, replace `.\.venv\Scripts\python.exe` with `./.venv/bin/python` and use Linux paths. These tools import the media dependencies even for some commands that primarily inspect metadata.

For every acquisition, copy the motor and camera files into one experiment folder. Keep the original filenames and shared run IDs:

| Input | Required for |
|---|---|
| `*_recorded.csv` speed-profile export | Encoder samples, timing, speed and run ID. Position-profile CSVs use a different, unsupported preparation schema. |
| `<run-id>_*.raw` | Event rendering and trigger extraction. |
| `<run-id>_*_metadata.json` | Camera clocks, run association and recording details. |
| Matching motor `_events.csv` / `_metadata.json` | Retain as acquisition evidence and for later rig evaluation. |
| `<run-id>_*_0x00_writes.csv` | Trigger-based alignment. |
| `<run-id>_*_trigger_events.csv` | Trigger-based alignment; generated from EVT3 RAW if absent. |

Discovery requires exactly one RAW and one camera metadata file per motor run ID. Duplicate copies of a run export under the experiment folder can confuse discovery; keep backups elsewhere.

## 3. Prepare an experiment

Use a working copy of a new experiment. Preparation **moves files**, regenerates setup/catalogue files and can update copied camera metadata. It is not an atomic operation and also organises excluded runs. Do not rerun it on accepted analysis inputs merely to remove an alignment warning.

```powershell
.\.venv\Scripts\python.exe vpr_media.py prepare-experiment --folder "..\Experiments\Low-Speed"
```

The first pass creates `experiment_setup.csv` and organises the acquisitions. Runs with missing manual inputs are reported as needing information. Do not rely on automatically assigned run numbers to identify recording order: check each run ID against its source acquisition. Open the setup CSV and fill/check these fields:

| Field | Meaning |
|---|---|
| `include` | `true` to prepare the run; `false` to omit it from the catalogue. |
| `run_label` | Unique readable label within the folder, such as `run1`. |
| `sync_mode` | `manual` or `trigger`. |
| `run_id` and file paths | Generated linkage; preserve the acquisition identity. Setup paths are relative to the experiment folder. |
| `manual_evk4_motion_s` | Manual mode: visible motion onset in Studio recording-relative seconds. |
| `studio_duration_s` | Duration of this exact RAW recording as reported by Studio. Needed for media and trigger alignment. |
| `nominal_speed_rpm` | Descriptive experiment metadata; verify your motor/platform convention. The SAD study defines its nominal platform speeds separately. |
| `vertical_condition`, `notes` | Optional description of the acquisition. |

Run the same preparation command again after completing the setup. Read all messages and check that each intended run is marked ready; a created catalogue does not mean every acquisition was admitted.

```text
Low-Speed/
|-- experiment_setup.csv
|-- raw/                         RAW recordings
|-- runs/run1/                   Motor exports and camera sidecars
|-- runs/run2/
|-- runs/run3/
`-- outputs/
    |-- vpr_runs.csv             Prepared catalogue
    |-- time_bins/               One time-bin CSV per admitted run
    `-- trigger_sync/            Diagnostics for trigger-aligned runs
```

Catalogue paths resolve relative to `outputs/`, not the setup CSV. Copy the organised folder intact when moving it to another computer.

### Choosing an alignment

- **Manual:** select visible motion onset on the Studio recording timeline. Preparation pairs it with the first of three consecutive feedback samples at or above 0.5 motor rpm in magnitude. A single onset does not estimate clock drift.
- **Trigger:** supply the UART-write CSV and Studio duration. Preparation extracts falling MAIN-channel trigger edges if needed, then fits an affine camera/LattePanda map. Inspect the match CSV, missing edges and possible one-period ambiguity; a small fit residual alone does not establish the correct pulse correspondence.

`--timing-uncertainty-ms` defaults to `100` for preparation. This is a provisional annotation for manual mapping, not a measured accuracy. Set it from your own assessment. Trigger preparation instead records its fit diagnostic. Manual bins cover the encoder-supported portion; trigger/recording-start mappings can extend labels to recording edges.

## 4. Preview RAW before alignment

This command does not need prepared time bins. Replace the example RAW filename and duration with values from your recording:

```powershell
.\.venv\Scripts\python.exe raw_timing_preview.py --raw "..\Experiments\trial.raw" --studio-duration-s 31.76 --output "..\Experiments\trial_preview.mp4" --start-s 0 --end-s 5
```

Once a setup CSV contains the recording path and duration, select its run instead:

```powershell
.\.venv\Scripts\python.exe raw_timing_preview.py --setup "..\Experiments\Low-Speed\experiment_setup.csv" --run run1 --output "..\Experiments\Low-Speed\preview_run1.mp4"
```

With Studio duration, recording zero is estimated from the last RAW timestamp minus that duration, retaining quiet lead-in. Without it, preview time is first-event-relative and must not be copied directly into `manual_evk4_motion_s`.

## 5. Render an annotated run video

Global `vpr_media.py` options such as `--catalog` and `--output-root` go **before the subcommand**. The following examples use the same experiment and output directory:

```powershell
.\.venv\Scripts\python.exe vpr_media.py --catalog "..\Experiments\Low-Speed\outputs\vpr_runs.csv" --output-root "..\Experiments\Low-Speed\media" run-video --run run1 --accumulation-ms 2 --source-fps 100 --speed 0.3
```

To restrict the video, append either `--start-s 5 --end-s 10` or `--start-rev 1 --end-rev 1`. Times are seconds from camera recording zero; revolution bounds are zero-based and inclusive. Do not combine time and revolution bounds.

| Option | Controls |
|---|---|
| `--accumulation-ms` | Event integration duration in each displayed frame. |
| `--source-fps` | Sampling frequency along recording time. It is not the camera's event rate. |
| `--speed` | Playback speed; `0.3` means 30% of real time. It does not change event accumulation. |

White pixels are ON events and blue pixels are OFF events. In the current renderer, blue wins if both occur at the same pixel in the window. Overlay values come from associated time-bin measurements; they are not simultaneous hardware measurements.

## 6. Find a view and compare its instances

First index an angle. This writes its reference CSV and available principal-frame PNGs:

```powershell
.\.venv\Scripts\python.exe vpr_media.py --catalog "..\Experiments\Low-Speed\outputs\vpr_runs.csv" --output-root "..\Experiments\Low-Speed\media" query-angle 120 --runs run1 run2 run3 --accumulation-ms 2
```

Then make an MP4 or GIF from that index, keeping the same angle and output root:

```powershell
.\.venv\Scripts\python.exe vpr_media.py --catalog "..\Experiments\Low-Speed\outputs\vpr_runs.csv" --output-root "..\Experiments\Low-Speed\media" compare-angle 120 --format video --instances run1:1 run2:1 run3:1 --window-ms 30 --speed 0.1

.\.venv\Scripts\python.exe vpr_media.py --catalog "..\Experiments\Low-Speed\outputs\vpr_runs.csv" --output-root "..\Experiments\Low-Speed\media" compare-angle 120 --format gif --instances run1:1 run2:1 run3:1 --window-ms 30 --speed 0.1
```

`run1:1` means run label `run1`, revolution 1. Inspect available instances in the reference CSV before selecting them. `--window-ms` controls the comparison clip around the indexed crossing; it is distinct from the accumulation duration set when querying the angle.

Principal frames are quantised to the source-fps grid and represent a finite event window. They are qualitative inspection images, not exact instantaneous views. Media indexing uses the time-bin representation; SAD uses explicit mapped sample timestamps and interpolation instead.

## 7. Compare complete turns at common angles

Check available steady turns first:

```powershell
.\.venv\Scripts\python.exe steady_revolutions.py --catalog "..\Experiments\Low-Speed\outputs\vpr_runs.csv" --runs run1 run2 run3
```

Choose valid instances from that output:

```powershell
.\.venv\Scripts\python.exe vpr_media.py --catalog "..\Experiments\Low-Speed\outputs\vpr_runs.csv" --output-root "..\Experiments\Low-Speed\media" compare-runs-by-angle --instances run1:1 run2:1 run3:1 --angle-step 0.5 --accumulation-ms 2 --fps 30
```

Use `--start-angle 30 --end-angle 150` for a partial arc; the upper bound is exclusive. This aligns displayed angle, not elapsed time. Two or three instances are supported. Cross-speed comparisons need a combined catalogue with distinct labels and correct paths.

## 8. Register already-prepared data

Use `add-run` when valid motor/time-bin files already exist and you want a catalogue without reorganising the acquisition:

```powershell
.\.venv\Scripts\python.exe vpr_media.py --catalog "..\Experiments\combined_runs.csv" add-run --label low_run1 --recorded "..\Experiments\Low-Speed\runs\run1\trial_recorded.csv" --time-bins "..\Experiments\Low-Speed\outputs\time_bins\run1_time_bins.csv" --raw "..\Experiments\Low-Speed\raw\trial.raw" --studio-duration 31.76
```

Replace example filenames and duration with the real files. The RAW filename must begin with the motor CSV's actual run ID; `trial.raw` above is only a filename placeholder. Repeating a label is rejected unless `--replace` is supplied. Use a separate combined catalogue: `add-run` writes the minimal media schema and does not preserve the extra columns in a richer `prepare-experiment` catalogue.

## 9. Outputs and checks

| Output | Use |
|---|---|
| `outputs/vpr_runs.csv` and `outputs/time_bins/` | Inputs to later media and SAD workflows. |
| `media/angles/<angle>/..._reference.csv` | Index of angle instances, crossing/frame times and file paths. |
| Principal-frame PNGs | Visual checks of matched scene content. |
| MP4/GIF files | Presentations and qualitative comparisons. RAW previews also write a matching `_frames.csv` timing record. |
| Trigger-fit CSVs | Diagnose edge correspondence and clock fitting. |

Media output names are deterministic; rerunning the same query/settings can replace earlier outputs. Use a new output root to retain variants. Reference CSVs contain absolute file paths and may need regenerating after transfer to another computer.

If a run is missing, check preparation messages, `include`, duration and onset fields. For wrong scene correspondence, first check recording-relative timing, the correct RAW duration and run IDs. Changing video playback speed cannot repair alignment. When the prepared mapping is accepted, continue to [SAD VPR](04_SAD_VPR_README.md).
