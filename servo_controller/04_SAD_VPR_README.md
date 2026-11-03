# SAD VPR: recognition and rig evaluation

Use this guide after collecting recordings with the [GUI](02_GUI_README.md) and accepting their [prepared timing and catalogue](03_VPR_MEDIA_README.md). The code extracts numeric event-count images from RAW data, matches independent acquisitions using sum of absolute differences (SAD), and evaluates matches against encoder-derived angles.

## 1. Scope and source files

The main study expects this layout:

```text
Experiments/
|-- Low-Speed/       nominal platform speed 15 rpm
|-- Medium-Speed/    nominal platform speed 50 rpm
`-- High-Speed/      nominal platform speed 150 rpm
    |-- experiment_setup.csv
    |-- raw/
    |-- runs/
    `-- outputs/vpr_runs.csv and time_bins/
```

Each condition must include `run1`, `run2` and `run3`, with unique acquisition IDs. The analysis assumes a 2:1 motor/platform reduction and forward rotation. Selected revolutions are Low 1, Medium 3 and High 11, numbered from zero. This is a rig-specific study, not a generic VPR dataset loader. Review `rig_vpr/data.py` and the evaluation code before changing speeds, run counts, direction or gearing.

| File | Responsibility / when to use it |
|---|---|
| `sad_vpr.py` | Main CLI for extraction, matching and follow-up studies. Start here. |
| `run_vpr_window_comparison.py` | Runs a controlled 1 ms versus 2 ms study with raw/L1 comparisons and sector summaries. |
| `evaluate_rig.py` | Standalone motion and continuous-event metrics; supports an explicit ramp-model assumption. |
| `run_rig_event_pipeline.py` | Runs motion analysis alone or combines motion, full-turn events and guarded pre-motion analysis. |
| `rig_vpr/data.py` | Study layout, protocol validation, acquisition discovery, mapped telemetry and frame-window planning. |
| `rig_vpr/algorithm.py` | RAW/fixture event reading, numeric counts, feature caches, descriptors, SAD and rank evaluation. |
| `rig_vpr/reporting.py` | Result CSVs, figures and summaries. |
| `rig_vpr/rescore.py` | Re-evaluates saved score matrices without re-reading RAW. |
| `rig_vpr/references.py` | Compares all independent reference/query pairs using cached count frames. |
| `rig_vpr/normalization.py` | Checks and reports paired raw/L1-normalised comparisons. |
| `rig_vpr/angular.py` | Equal-angular-aperture extraction and time-versus-angle study. |
| `rig_vpr/revolutions.py` | Compares query revolutions against the original fixed reference. |
| `rig_vpr/sectors.py` | Aggregates recognition by angular sector. |
| `rig_vpr/stationary.py` | Guarded pre-motion counts, spatial maps and spectra. |
| `rig_vpr/inspection.py` | Reconstructs accepted mappings and validates frozen inputs/caches. |
| `rig_vpr/__init__.py` | Package version. |

The modules in `rig_vpr/` are imported helpers, not separate commands. Shared timing helpers also come from `generate_vpr_time_bins.py`. Attribution and the upstream MIT licence are in [THIRD_PARTY_VPR.md](THIRD_PARTY_VPR.md). This implementation uses rig-specific extraction and evaluation; it does not reproduce every part of the published sparse/varying-pixel method.

## 2. Environment and paths

Commands below use Bash from `code/servo_controller`, with a working Linux EventCV environment. On Windows, use `.\.venv\Scripts\python.exe`, PowerShell paths and PowerShell continuation syntax instead of Bash backslashes.

```bash
./.venv/bin/python -m pip install -r requirements-vpr.txt
./.venv/bin/python sad_vpr.py --help

# Reuse these paths in the examples below; change them for your deployment.
EXPERIMENTS="../Experiments"
STUDY="$EXPERIMENTS/VPR-SAD-2ms"
```

EventCV must be able to open your RAW files. Installing Python packages alone does not verify that native decoding works. SciPy supports distance calculation; the code also has a NumPy backend. Matplotlib is needed for plots; most individual analysis commands accept `--no-plots` for numerical outputs only.

Keep accepted acquisitions and prepared inputs unchanged. Source identity, alignment and comparison consistency are checked, and changed inputs can invalidate a protocol.

## 3. Create, check and run a baseline

```bash
./.venv/bin/python sad_vpr.py init \
  --experiments-root "$EXPERIMENTS" \
  --output "$STUDY/protocol.json" \
  --accumulation-ms 2 --tolerance-deg 5

./.venv/bin/python sad_vpr.py check --protocol "$STUDY/protocol.json"

./.venv/bin/python sad_vpr.py run --protocol "$STUDY/protocol.json"
```

- **`init`** discovers the nine acquisitions, freezes their settings/input hashes and writes a new protocol. It refuses to replace an existing protocol.
- **`check`** validates IDs, input files, mapping, camera-bias agreement and the complete selected steady arc. It checks RAW presence without decoding it. `--metadata-only` permits missing RAW for a metadata-copy check.
- **`run`** decodes the selected windows, caches count frames, calculates distances, evaluates matches and writes results. `run_complete.json` marks completion. Partial files after an error do not constitute a completed run.

The example explicitly selects a 5-degree correctness tolerance. **The parser default is 2 degrees**, so omitting `--tolerance-deg 5` changes the protocol. Select the tolerance for your experiment; neither value measures alignment accuracy.

If discovery reports different setup and prepared onsets, inspect them before proceeding. Prepared catalogue values take precedence when present. Rerunning preparation uses the setup values and can change an accepted mapping; do not do that merely to suppress a warning.

### What the baseline computes

| Setting | Current default, unless overridden |
|---|---|
| Reference / queries | Low-Speed/run1 against the other eight acquisitions. `init --all-pairs` uses all 72 directed pairs. |
| Angular sampling | 360 centres, 1 degree apart, starting at wrapped 4 degrees. |
| Representation | Counts in a 160 x 90 grid; ON and OFF summed. |
| Window | Centred 2 ms time interval at each sampled angle. |
| Feature selection | All spatial bins (`--pixels 0`); a positive count selects fixed seeded random bins. |
| Normalisation | Raw counts (`--normalization none`); `l1` divides each nonempty descriptor by its count sum. |
| Matching | Exhaustive SAD over all nonempty reference descriptors. |
| Motion screen | Forward motion and feedback within 5% of nominal, with constant command. |

Angles are interpolated using explicit mapped encoder-sample times. They label and evaluate matches; they do not restrict the reference search. ON/OFF splitting is available with `--polarity split`. Spatial pooling preserves counts rather than turning them into binary occupancy.

At 15, 50 and 150 platform rpm, 2 ms spans nominally 0.18, 0.60 and 1.80 degrees. Fixed angular sample centres do not imply fixed angular window widths. These sampled windows are not continuous full-turn totals.

`Recall@1` requires a unique best match within the specified angular tolerance. `Recall@5` tests the first five ranked candidates using the same tolerance. Empty queries remain failures; empty references are excluded. Ties crossing the rank cutoff receive no credit, and ground-truth angle never breaks a score tie. Five adjacent reference views are not five independent places.

## 4. Understand and retain the outputs

```text
VPR-SAD-2ms/
|-- protocol.json
|-- cache/                       Reusable numeric count frames
`-- results/
    |-- run_complete.json        Completion marker
    |-- protocol_snapshot.json
    |-- provenance.json
    |-- input_audit.csv
    |-- frame_windows.csv
    |-- selected_spatial_bins.npy
    |-- summary_per_acquisition.csv
    |-- summary_by_speed.csv
    |-- matches.csv and top_candidates.csv
    |-- tolerance_sensitivity.csv
    |-- pairs/                  Per-pair scores and query results
    `-- figures/                Figures, unless --no-plots
```

Start with `summary_by_speed.csv`, then inspect the per-acquisition summaries and query rows. Preserve `protocol.json`, the whole `results/` folder and sibling `cache/` together. Some follow-up commands need all of them.

`run --overwrite` archives prior results into `results.previous-*` before rerunning. Valid extraction caches can be reused. Create a new study directory to change the experimental configuration. RAW cache identity uses file size, modification time and first/last MiB hashes; it is not a full-file checksum. Provenance also records code/input identities and package versions.

## 5. Compare raw and normalised counts across references

Run these after the baseline completes:

```bash
./.venv/bin/python sad_vpr.py compare-references \
  --results "$STUDY/results" --tolerance-deg 5 --normalization none \
  --output-root "$STUDY/references_raw"

./.venv/bin/python sad_vpr.py compare-references \
  --results "$STUDY/results" --tolerance-deg 5 --normalization l1 \
  --baseline-comparison "$STUDY/references_raw" \
  --output-root "$STUDY/references_l1"
```

This uses cached count frames without decoding RAW again. Every acquisition serves as reference for the other eight, giving 72 directed comparisons. The second command also checks the paired raw baseline and writes normalisation differences.

Each output contains per-pair and grouped summaries, query metrics, score matrices, provenance and plots. Read `summary_by_reference_speed.csv` for the reference-speed/query-speed comparison. `reference_comparison_complete.json` marks completion. Recordings are reused across pairs; 72 comparisons are not 72 independent acquisitions.

## 6. Optional follow-up analyses

These are alternatives/extensions, not required steps for the basic baseline. Use `--help` after the relevant subcommand for all options.

### Score an older completed run

```bash
./.venv/bin/python sad_vpr.py rescore --results "$STUDY/results"
```

Reads saved distance matrices, checks the original Recall@1 and writes both rank metrics to a separate sibling `ranked_metrics/` directory. It needs saved results, not RAW or frame caches. New baseline runs already report both metrics.

### Analyse angular sectors

```bash
./.venv/bin/python sad_vpr.py angle-sectors \
  --raw-results "$STUDY/references_raw" \
  --normalized-results "$STUDY/references_l1" \
  --sector-deg 30 --sector-start-deg -15 \
  --output-root "$STUDY/sectors_30deg_centred"
```

Uses existing raw/L1 query results, validates paired inputs and preserves empty-query failures. The example centres a sector on zero: `[-15,15)`, `[15,45)`, and so on modulo 360. Without `--sector-start-deg -15`, the default first lower boundary is zero. Retain the original `results/frame_windows.csv`; RAW decoding is not needed.

### Compare time and angular accumulation

```bash
./.venv/bin/python sad_vpr.py angular-study \
  --source-results "$STUDY/results" --accumulation-deg 0.6 --tolerance-deg 5 \
  --output-root "$EXPERIMENTS/VPR-SAD-angle0p6deg"
```

Uses the same acquisitions and centres with an equal-angle aperture, then compares time/angle accumulation with raw/L1 counts. New RAW extraction is required: existing 2 ms count images cannot reconstruct another window. `--prepare-only` checks/freezes the plan without RAW extraction or recognition scores. A 0.6-degree aperture is separate from the 1-degree sampling step and 5-degree correctness tolerance.

### Change the query revolution

```bash
./.venv/bin/python sad_vpr.py revolution-study \
  --source-results "$STUDY/results" --motion-phase steady --tolerance-deg 5 \
  --output-root "$EXPERIMENTS/VPR-Revolution-Study"
```

Keeps the original independent reference and tests eligible query turns. `--motion-phase steady` restricts the study to steady motion; the default `any` allows forward non-steady phases. Use `--low-revolutions`, `--medium-revolutions` and `--high-revolutions` to select inclusive lists/ranges. `--prepare-only` writes the plan without extracting new RAW windows.

### Examine pre-motion activity

```bash
./.venv/bin/python sad_vpr.py stationary \
  --source-results "$STUDY/results" \
  --duration-s 0.5 --guard-s 0.2 --startup-s 0.2 --bin-ms 0.5 \
  --output-root "$EXPERIMENTS/VPR-Stationary"
```

Selects 0.5 s ending 0.2 s before the accepted manual onset, excluding the first 0.2 s of recording. It produces temporal counts, native spatial maps, spectra, candidate peaks and provenance. The default interval gives 2 Hz spectral resolution. `--prepare-only` inspects the plan without RAW extraction. This selection currently supports manual-aligned acquisitions; encoder coverage may not independently establish stationarity. Periodic activity does not by itself prove lighting flicker or sensor noise.

### Run a paired 1 ms / 2 ms study

```bash
./.venv/bin/python run_vpr_window_comparison.py prepare \
  --experiments-root "$EXPERIMENTS" --output-root "$EXPERIMENTS/VPR-Window-Study"

./.venv/bin/python run_vpr_window_comparison.py run \
  --experiments-root "$EXPERIMENTS" --output-root "$EXPERIMENTS/VPR-Window-Study"
```

The preparation stage requires RAW files to exist but does not decode them. It validates matching acquisitions, alignment and angular centres, and rejects unresolved discovery warnings. The run stage extracts both windows, compares all references with raw/L1 counts and writes centred-sector summaries. This runner requires plotting dependencies; it has no `--no-plots` switch. `window_study_complete.json` marks completion.

## 7. Evaluate the platform and continuous event stream

Recognition windows sample a recording. This separate analysis measures motion and, when requested, the entire selected event turn using the accepted VPR mapping.

```bash
# Motion only: no RAW decoding, but the completed VPR record is still required.
./.venv/bin/python run_rig_event_pipeline.py motion \
  --experiments-root "$EXPERIMENTS" --vpr-results "$STUDY/results" \
  --output-root "$EXPERIMENTS/Rig-Evaluation-Motion"

# Motion + continuous full-turn events + guarded pre-motion analysis.
./.venv/bin/python run_rig_event_pipeline.py complete \
  --experiments-root "$EXPERIMENTS" --vpr-results "$STUDY/results" \
  --output-root "$EXPERIMENTS/Rig-Evaluation-Complete"
```

Always choose a new output directory. The complete stage re-decodes full-turn events, reconciles totals and runs stationary analysis. Keep `event_arrays/`, all CSVs, provenance, figures and the `stationary/` subfolder when transferring results back.

| Analysis | Main outputs |
|---|---|
| Motion | `motion_per_run.csv`, `condition_summary.csv`, `revolution_periods.csv`, `position_trajectory_repeatability.csv`, `data_quality.csv`. |
| Continuous events | `event_statistics.csv`, `event_condition_summary.csv`, angular-sector tables, native maps/temporal arrays in `event_arrays/`, and event/VPR sector joins when matching comparisons are found. |
| Pre-motion | Tables, spatial arrays and spectra under `stationary/`. |

Motion diagnostics retain Low revolution 1, Medium 1-6 and High 6-17. Full-turn event analysis uses Low 1, Medium 3 and High 11, with default angular bounds 4 to 364 degrees. Native maps, complete disjoint 2 ms windows plus the final partial tail, and angular sectors account for the same selected event total.

**Ramp assumption:** the runner passes an estimated 8 s ramp at 500 motor rpm. It does not recover historical drive settings. For another assumption or numerical-only output, call the lower-level command explicitly:

```bash
./.venv/bin/python evaluate_rig.py \
  --experiments-root "$EXPERIMENTS" --vpr-results "$STUDY/results" \
  --output-root "$EXPERIMENTS/Rig-Motion-Custom" \
  --ramp-seconds 8 --ramp-reference-rpm 500 --no-plots
```

Add `--events` for continuous RAW event analysis. `--event-results` instead imports an existing event-evaluation result folder; it is mutually exclusive with `--events`.

## 8. Completion and troubleshooting

| Operation | Completion marker |
|---|---|
| Baseline | `results/run_complete.json` |
| Rescore | `rescore_complete.json` |
| Reference comparison | `reference_comparison_complete.json` |
| Angular sectors | `sector_analysis_complete.json` |
| Angular/window study | `window_study_complete.json` |
| Revolution study | `revolution_study_complete.json` |
| Stationary analysis | `stationary_complete.json` |
| Motion-only evaluation | `rig_motion_complete.json` |
| Combined rig pipeline | `rig_event_pipeline_complete.json` |

A marker in one stage does not establish completion of another. Check the relevant output folder and its provenance.

| Problem | Next check |
|---|---|
| Setup/catalogue alignment warning | Compare accepted onset values and paths before changing either mapping. |
| Input hash changed | Identify what changed, then create a new protocol if the change is intentional. |
| Steady arc or coverage rejected | Inspect selected revolution, start angle, feedback and camera/telemetry coverage; do not silently relax the screen. |
| RAW cannot open | Correct file location, EventCV/native backend and the environment used for extraction. |
| Matplotlib missing | Install the analysis requirements or use an individual command's `--no-plots` option where supported. |
| Follow-up cannot find cached frames | Keep the original `cache/` beside `results/` and preserve its recorded filenames. |
| Output already exists | Choose a new output root, or use a documented `--overwrite` option on commands that provide one. |

Report recorded event characteristics and recognition outcomes alongside the scene, biases, timing assumptions and representation settings. Keep native RAW timestamps, Studio-relative seconds, Motor-PC monotonic time and LattePanda-relative time distinct when interpreting files. These scripts do not determine event-loss causes, independently calibrate platform angle or establish a general speed limit for event-based VPR.
