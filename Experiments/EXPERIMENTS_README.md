# Experiments: recordings, preparation and analysis outputs

This folder stores acquisition records and derived results. Software lives in [servo_controller](../servo_controller/01_README.md); printable targets in `code/Scenes` are described in [SCENES_README.md](../Scenes/SCENES_README.md).

## 1. Folder map

| Folder | Contents / role |
|---|---|
| `Low-Speed/` | Three acquisitions at 30 motor rpm, equivalent to 15 platform rpm. |
| `Medium-Speed/` | Three acquisitions at 100 motor rpm, equivalent to 50 platform rpm. |
| `High-Speed/` | Three acquisitions at 300 motor rpm, equivalent to 150 platform rpm. |
| `LowLow-Speed/` | Additional acquisitions at 10 motor rpm / 5 platform rpm. Not part of the current nine-acquisition SAD baseline; its prepared catalogue is empty in this copy. |
| `Cross-Speed/` | Derived comparison videos and correspondence images. These are inspection/presentation outputs, not source recordings or recognition scores. |
| `VPR-SAD-2ms/` | Earlier 2 ms baseline, cached frames, ranked metrics and reference comparisons. |
| `VPR-Reevaluation-2ms/` | Timestamped 2 ms reevaluation; see the accepted comparison described below. |
| `VPR-Reevaluation-1ms-2ms/` | Paired 1 ms / 2 ms study with extraction, raw/L1 reference comparisons and sector analyses. |
| `VPR-SAD-angle0p6deg/` | Completed angular-aperture study with raw/L1 reference comparisons and sector results. |
| `VPR-SAD-revolutions/` | Planned-turn files only in this copy; no completed revolution-study marker was found. |
| `VPR-Stationary/` | Planned pre-motion intervals only in this copy. Completed stationary results are also stored under `Rig-Evaluation-Final/Complete-Linux/stationary/`. |
| `Rig-Evaluation/`, `Rig-Evaluation-Recovered/` | Earlier diagnostic outputs. Retain as history; do not combine their rows with a later evaluation as if they were additional trials. |
| `Rig-Evaluation-Final/` | Local motion analyses and the completed returned Linux motion/event analysis. |

Folder names describe their intended role. Check the relevant protocol, provenance and completion marker before treating a directory as a completed analysis or the source of a reported figure.

## 2. One acquisition folder

The three baseline conditions use the same structure:

```text
Low-Speed/
|-- motor-30rpm_10s.csv              Input motion profile
|-- experiment_setup.csv            Run IDs, paths, alignment and metadata
|-- runs/
|   |-- run1/                      Motor exports and camera sidecars
|   |-- run2/
|   `-- run3/
|-- raw/                            Expected location when RAW files are copied
`-- outputs/
    |-- vpr_runs.csv                Prepared run catalogue
    `-- time_bins/
        |-- run1_time_bins.csv
        |-- run2_time_bins.csv
        `-- run3_time_bins.csv
```

**Local data availability:** when this README was checked on 3 November 2026, the three main conditions each had their three motor exports, camera metadata, UART logs and time-bin files, but none of the RAW paths in their setup CSVs existed in this Windows copy. Restore/copy the corresponding RAW files from the recording/analysis computer before decoding events, regenerating media or extracting new SAD frames. This README does not establish that another copy is a complete backup.

### Files belonging to a run

| File | Meaning |
|---|---|
| `*_recorded.csv` | Motor feedback, encoder-derived angles, timing and optional scene measurements. |
| `*_recorded_events.csv` | Command/release and coordination event log. Keep it for motion timing analysis. |
| `*_recorded_metadata.json` | Motor-side run identity, source profile and clock mapping. |
| `<run-id>_<timestamp>.raw` | Camera event stream. Needed for new event extraction. |
| `<run-id>_<timestamp>_metadata.json` | Camera session clocks, biases, health/finalization and file linkage. |
| `<run-id>_<timestamp>_0x00_writes.csv` | UART trigger-write attempts and timing. |
| `*_trigger_events.csv`, if generated | Camera-clock trigger edges decoded from RAW. |
| `outputs/time_bins/*_time_bins.csv` | Prepared association between encoder samples and camera time. |

Use `run_id` to join files, not filename timestamps alone. `run1` is a local label reused in different conditions. Preserve the complete sidecar set when copying a run. Do not put duplicate backup exports inside an acquisition folder that will be rediscovered by preparation.

## 3. Setup, units and alignment

`experiment_setup.csv` contains human-editable preparation inputs, including `include`, `run_label`, `sync_mode`, `manual_evk4_motion_s` and `studio_duration_s`. Its paths are relative to the condition folder. `outputs/vpr_runs.csv` is the prepared catalogue; its paths are relative to `outputs/`.

For the current condition folders, `nominal_speed_rpm` records **motor rpm**: 30, 100 and 300. The SAD protocol separately records **platform rpm**: 15, 50 and 150. The installed reduction is 2:1. Do not compare these fields without converting units.

Keep these time references distinct:

| Field / unit | Reference |
|---|---|
| `motor_pc_monotonic_ns` | Motor-PC acquisition clock. |
| `lattepanda_monotonic_ns` | Mapped LattePanda clock. |
| `lattepanda_time_s` | Seconds relative to the scheduled run T0. |
| `manual_evk4_motion_s` | Visible motion onset on the Studio recording-relative timeline. |
| `encoder_sample_camera_us` | Prepared camera-relative timestamp for an encoder sample. |
| Native RAW timestamps | Camera timestamps before applying the recording-relative origin. |

Revolutions are numbered from zero. The standard motion analysis selects Low revolution 1, Medium 1-6 and High 6-17. The standard event/VPR selection uses Low 1, Medium 3 and High 11, sampling from wrapped 4 degrees. The protocol and coverage checks define the actual arc.

Preparation moves files and rewrites setup/catalogue mappings. Once an analysis has accepted and frozen a mapping, preserve its inputs. For retrospective analysis, use its `protocol_snapshot.json` and provenance rather than substituting newly edited setup values.

## 4. Finding the accepted analyses

The project rig-evaluation workflow identifies this completed 2 ms reevaluation as its alignment source:

```text
VPR-Reevaluation-2ms/20261020-141841/
|-- protocol.json
|-- cache/
|-- results/
|-- references_raw/
|-- references_l1/
`-- sectors_30deg_centred/
```

Its protocol uses raw 2 ms count images, a 160 x 90 grid, 1-degree sampling and a 5-degree correctness tolerance. For recognition results, start with `references_raw/summary_by_reference_speed.csv` and the matching L1 summary. Read the corresponding provenance when selecting plots or comparing studies. The separate `VPR-Reevaluation-1ms-2ms/` directory is the paired duration study, not an extra set of independent acquisitions.

The completed combined motion/event evaluation is in:

```text
Rig-Evaluation-Final/Complete-Linux/
|-- condition_summary.csv
|-- motion_per_run.csv
|-- data_quality.csv
|-- position_trajectory_repeatability.csv
|-- event_statistics.csv
|-- event_condition_summary.csv
|-- event_sector_counts_30deg_centred.csv
|-- event_vpr_sector_comparison.csv
|-- provenance.json
|-- event_arrays/
|-- figures/
|-- stationary/
`-- rig_event_pipeline_complete.json
```

The `Motion-Local` and `Motion-Local-initial` folders under `Rig-Evaluation-Final/` are motion-only outputs. They are not substitutes for completed RAW event analysis.

## 5. What to preserve when transferring results

- **New acquisitions:** profile CSV, motor recording/events/metadata, camera RAW/UART/metadata and any trigger extraction.
- **Prepared acquisitions:** the whole condition folder, including setup, catalogue and time bins. Preserve relative paths.
- **Completed SAD studies:** protocol, complete `results/` folder and sibling `cache/`; include follow-up comparison folders you need. New window extraction still requires RAW.
- **Completed rig evaluation:** all CSVs, provenance, figures, `event_arrays/`, stationary results and completion markers.

Completion markers include `run_complete.json`, `reference_comparison_complete.json`, `sector_analysis_complete.json`, `window_study_complete.json`, `stationary_complete.json` and `rig_event_pipeline_complete.json`. A plan JSON or partially populated folder is not evidence that extraction or evaluation completed.

## 6. Commands and next steps

Run tools from `code/servo_controller`, not this data directory. Choose a new output directory for a new configuration; do not write new results over accepted source records.

| Task | Instructions |
|---|---|
| Collect a new acquisition | [02 GUI and recording](../servo_controller/02_GUI_README.md) |
| Prepare files, inspect alignment or make videos | [03 VPR Media](../servo_controller/03_VPR_MEDIA_README.md) |
| Run SAD, compare studies or regenerate rig metrics | [04 SAD VPR](../servo_controller/04_SAD_VPR_README.md) |

The existing README files inside result directories describe those particular generated analyses. Retain them with the results; they are separate from this folder-level guide. No acquisition data or generated metrics were changed when creating this README.
