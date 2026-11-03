#!/usr/bin/env python3
"""Run an encoder-referenced event-count SAD VPR baseline on prepared rig acquisitions.

Commands: init snapshots the protocol; check validates coverage/alignment;
run extracts/caches RAW descriptors and evaluates independent acquisitions.
Acquisition files and preparation outputs are never modified.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from rig_vpr import VERSION
from rig_vpr.algorithm import code_fingerprints, extract, match, metrics, pixel_indices, runtime_versions
from rig_vpr.data import check_inputs, discover, load_protocol, sha256
from rig_vpr.reporting import candidate_rows, match_rows, pair_figures, plotting, report, speed_summary, summary_figure, write_csv


def run(protocol_path, no_plots=False, overwrite=False):
    protocol_path = Path(protocol_path).expanduser().resolve()
    protocol, root = load_protocol(protocol_path)
    output = protocol_path.parent/"results"
    if output.is_symlink():
        raise ValueError("Results directory must not be a symlink")
    previous = output.is_dir() and any(output.iterdir())
    if previous and not overwrite:
        raise FileExistsError(f"{output} already has a run. Use a new protocol directory or --overwrite to replace its results (cached features are reusable).")
    if not no_plots:
        plotting()  # Fail before extraction if optional plotting dependencies are absent.
    plans, audit = check_inputs(protocol, root, require_raw=True)
    if previous:
        archive = output.with_name("results.previous-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        if output.resolve().parent != protocol_path.parent or archive.parent.resolve() != protocol_path.parent:
            raise ValueError("Results archive would leave the protocol directory")
        output.rename(archive)
        print(f"[ARCHIVE] Previous results retained at {archive}", flush=True)
    output.mkdir(parents=True, exist_ok=True)
    (output/"run_complete.json").unlink(missing_ok=True)
    (output/"run_started.json").write_text(json.dumps(dict(started_utc=datetime.now(timezone.utc).isoformat(),
        protocol_sha256=sha256(protocol_path), version=VERSION), indent=2), encoding="utf-8")
    write_csv(output/"input_audit.csv", [{k:v for k,v in row.items() if k != "sources"} for row in audit])
    for warning in protocol.get("discovery_warnings", []):
        print("[ALIGNMENT] "+warning, flush=True)
    cfg = protocol["settings"]
    features, metadata = {}, {}
    for entry, source_audit in zip(protocol["acquisitions"], audit):
        features[entry["key"]], metadata[entry["key"]] = extract(entry, cfg, plans[entry["key"]], root, protocol_path.parent/"cache", source_audit)
    if len({tuple(m["sensor_size"]) for m in metadata.values()}) != 1:
        raise ValueError("Acquisitions use different native sensor dimensions")
    indices = pixel_indices(cfg)
    np.save(output/"selected_spatial_bins.npy", indices)
    provenance = dict(version=VERSION, protocol=protocol, protocol_sha256=sha256(protocol_path),
        code_sha256=code_fingerprints(), runtime=runtime_versions(), inputs=metadata,
        ground_truth="explicit mapped encoder_sample_camera_us, linearly interpolated; no bin-midpoint approximation",
        primary_metric="Recall@1 with empty/tied queries counted as failures",
        additional_metric="Recall@5 at the same angle tolerance; boundary-spanning tie groups receive no credit",
        tolerance_status="baseline choice, not a measured synchronization accuracy",
        raw_checksum_scope="partial first/last MiB and file stat only; not full RAW hashes")
    (output/"provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    entries = protocol["acquisitions"]
    refs = entries if protocol["pair_mode"] == "all-independent-pairs" else [e for e in entries if e["key"] == protocol["reference"]]
    pairs, all_matches, sensitivity, windows, all_candidates = [], [], [], [], []
    for entry in entries:
        data = features[entry["key"]]
        for i in range(len(data["frames"])):
            windows.append(dict(acquisition=entry["key"], frame_index=i,
                center_s=float(data["center_us"][i]/1e6), start_s=float(data["start_us"][i]/1e6), end_s=float(data["end_us"][i]/1e6),
                wrapped_angle_deg=float(data["angle_deg"][i]), requested_wrapped_angle_deg=float(data["requested_angle_deg"][i]),
                angular_span_deg=float(data["window_angle_span_deg"][i]), platform_rpm=float(data["speed_rpm"][i]),
                event_count=int(data["event_counts"][i])))
    write_csv(output/"frame_windows.csv", windows)
    for reference_entry in refs:
        for query_entry in entries:
            if reference_entry["run_id"] == query_entry["run_id"]:
                continue
            ref_key, qry_key = reference_entry["key"], query_entry["key"]
            reference, query = features[ref_key], features[qry_key]
            result = match(reference, query, cfg, indices)
            row = dict(reference=ref_key, query=qry_key, query_condition=query_entry["condition"],
                query_run=query_entry["run"], nominal_query_platform_rpm=query_entry["nominal_platform_rpm"],
                tolerance_deg=cfg["tolerance_deg"], reference_frames=len(reference["frames"]),
                reference_empty_descriptors=int(np.sum(~result["reference_valid"])),
                distance_backend=result["backend"], **metrics(result, cfg["tolerance_deg"]))
            pairs.append(row)
            stem = ref_key.replace("/", "_")+"__"+qry_key.replace("/", "_")
            folder = output/"pairs"/stem
            folder.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(folder/"distance_matrix.npz", scores=result["scores"],
                reference_angles_deg=reference["angle_deg"], query_angles_deg=query["angle_deg"],
                reference_valid=result["reference_valid"], query_valid=result["query_valid"])
            rows = match_rows(reference_entry, query_entry, reference, query, result)
            write_csv(folder/"matches.csv", rows)
            all_matches.extend(rows)
            ranked_rows = candidate_rows(ref_key, qry_key, result, cfg["tolerance_deg"])
            write_csv(folder/"top_candidates.csv", ranked_rows)
            all_candidates.extend(ranked_rows)
            for tolerance in sorted(set([.5, 1., 2., 5., 10., cfg["tolerance_deg"]])):
                sensitivity.append(dict(reference=ref_key, query=qry_key, tolerance_deg=tolerance, **metrics(result, tolerance)))
            if not no_plots:
                pair_figures(folder, reference, query, result, ref_key, qry_key, cfg["tolerance_deg"])
            print(f"[MATCH] {qry_key}: Recall@1={100*row['recall_at_1']:.2f}%, Recall@5={100*row['recall_at_5']:.2f}%, coverage={100*row['match_coverage']:.2f}% against {ref_key}", flush=True)
    if not pairs:
        raise ValueError("No independent reference/query pairs")
    summaries = speed_summary(pairs)
    write_csv(output/"matches.csv", all_matches)
    write_csv(output/"top_candidates.csv", all_candidates)
    write_csv(output/"summary_per_acquisition.csv", pairs)
    write_csv(output/"summary_by_speed.csv", summaries)
    write_csv(output/"tolerance_sensitivity.csv", sensitivity)
    report(output/"README_results.md", protocol, pairs, summaries)
    if not no_plots:
        summary_figure(output/"figures", pairs, summaries, cfg["tolerance_deg"])
        summary_figure(output/"figures", pairs, summaries, cfg["tolerance_deg"], k=5)
    (output/"protocol_snapshot.json").write_text(json.dumps(protocol, indent=2), encoding="utf-8")
    (output/"run_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),
        pairs=len(pairs), primary_tolerance_deg=cfg["tolerance_deg"], protocol_sha256=sha256(protocol_path)), indent=2), encoding="utf-8")
    print(f"[DONE] {len(pairs)} independent pairs: {output}", flush=True)
    return output


def parser():
    main = argparse.ArgumentParser(description=__doc__)
    sub = main.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init", help="Create a portable baseline protocol from existing experiment setup/catalogues")
    init.add_argument("--experiments-root", type=Path, required=True)
    init.add_argument("--output", type=Path, required=True, help="New protocol.json, preferably in a new directory per configuration")
    init.add_argument("--accumulation-ms", type=float, default=2.)
    init.add_argument("--angle-step-deg", type=float, default=1.)
    init.add_argument("--tolerance-deg", type=float, default=2.)
    init.add_argument("--start-wrapped-deg", type=float, default=4.)
    init.add_argument("--width", type=int, default=160)
    init.add_argument("--height", type=int, default=90)
    init.add_argument("--polarity", choices=("sum", "split"), default="sum")
    init.add_argument("--normalization", choices=("none", "l1"), default="none")
    init.add_argument("--pixels", type=int, default=0, help="0 uses all spatial bins; positive selects fixed seeded random bins")
    init.add_argument("--seed", type=int, default=0)
    init.add_argument("--steady-tolerance-percent", type=float, default=5.)
    init.add_argument("--all-pairs", action="store_true", help="Optional alternative protocol: all 72 ordered independent acquisition pairs")
    check = sub.add_parser("check", help="Validate inputs, alignment and complete steady coverage without decoding RAW")
    check.add_argument("--protocol", type=Path, required=True)
    check.add_argument("--metadata-only", action="store_true", help="Permit absent RAW files when inspecting a metadata-only copy")
    execute = sub.add_parser("run", help="Extract/cache frames, match and write results")
    execute.add_argument("--protocol", type=Path, required=True)
    execute.add_argument("--no-plots", action="store_true")
    execute.add_argument("--overwrite", action="store_true", help="Archive any previous results and rerun using valid cached features")
    rescore = sub.add_parser("rescore", help="Add Recall@5 from completed saved score matrices, without RAW decoding")
    rescore.add_argument("--results", type=Path, required=True)
    rescore.add_argument("--output-root", type=Path, help="Defaults to ranked_metrics beside the original results directory")
    rescore.add_argument("--no-plots", action="store_true")
    rescore.add_argument("--overwrite", action="store_true", help="Archive previous ranked metrics before replacing them")
    references = sub.add_parser("compare-references", help="Compare every independent reference/query pair from the completed run's frame caches")
    references.add_argument("--results", type=Path, required=True)
    references.add_argument("--tolerance-deg", type=float, default=5.)
    references.add_argument("--output-root", type=Path)
    references.add_argument("--no-plots", action="store_true")
    references.add_argument("--overwrite", action="store_true")
    references.add_argument("--normalization", choices=("none","l1"), help="Override descriptor normalization using the same cached count frames")
    references.add_argument("--baseline-comparison", type=Path, help="Completed raw-count reference comparison for paired normalization tables and plots")
    angular=sub.add_parser("angular-study",help="Extract equal-angle windows and compare time/angle accumulation with raw and L1-normalised SAD")
    angular.add_argument("--source-results",type=Path,required=True,help="Completed original time-window results")
    angular.add_argument("--accumulation-deg",type=float,default=.6)
    angular.add_argument("--tolerance-deg",type=float,default=5.)
    angular.add_argument("--output-root",type=Path)
    angular.add_argument("--prepare-only",action="store_true",help="Freeze the angular protocol and check encoder coverage without requiring RAW")
    angular.add_argument("--no-plots",action="store_true")
    sectors=sub.add_parser("angle-sectors",help="Analyse existing raw/L1 query metrics by angular sector without RAW decoding")
    sectors.add_argument("--raw-results",type=Path,required=True)
    sectors.add_argument("--normalized-results",type=Path,required=True)
    sectors.add_argument("--sector-deg",type=float,default=30.)
    sectors.add_argument("--sector-start-deg",type=float,default=0.,help="First lower boundary; use -15 with 30-degree sectors to centre a sector on zero")
    sectors.add_argument("--output-root",type=Path)
    sectors.add_argument("--no-plots",action="store_true")
    stationary=sub.add_parser("stationary",help="Analyse guarded pre-motion activity and candidate flicker frequencies in all nine RAW recordings")
    stationary.add_argument("--source-results",type=Path,required=True)
    stationary.add_argument("--duration-s",type=float,default=.5)
    stationary.add_argument("--guard-s",type=float,default=.2)
    stationary.add_argument("--startup-s",type=float,default=.2)
    stationary.add_argument("--bin-ms",type=float,default=.5)
    stationary.add_argument("--output-root",type=Path)
    stationary.add_argument("--prepare-only",action="store_true")
    stationary.add_argument("--no-plots",action="store_true")
    revolutions=sub.add_parser("revolution-study",help="Vary query turns with the original independent reference fixed")
    revolutions.add_argument("--source-results",type=Path,required=True)
    revolutions.add_argument("--low-revolutions",default="all",help="Comma-separated indices/ranges or all")
    revolutions.add_argument("--medium-revolutions",default="all")
    revolutions.add_argument("--high-revolutions",default="all")
    revolutions.add_argument("--motion-phase",choices=("any","steady"),default="any")
    revolutions.add_argument("--tolerance-deg",type=float,default=5.)
    revolutions.add_argument("--output-root",type=Path)
    revolutions.add_argument("--prepare-only",action="store_true")
    revolutions.add_argument("--no-plots",action="store_true")
    main.add_argument("--version", action="version", version=VERSION)
    return main


def main(argv=None):
    cli = parser(); args = cli.parse_args(argv)
    try:
        if args.command == "init":
            path = args.output.expanduser().resolve()
            if path.exists():
                raise FileExistsError(f"Protocol already exists: {path}. Choose a new output path.")
            settings = {name:getattr(args, name) for name in ("accumulation_ms", "angle_step_deg", "tolerance_deg", "start_wrapped_deg", "width", "height", "polarity", "normalization", "pixels", "seed", "steady_tolerance_percent")}
            protocol = discover(args.experiments_root.expanduser(), path, settings, args.all_pairs)
            # Validate actual mapped telemetry and selected arcs before writing a protocol.
            _, audit = check_inputs(protocol, args.experiments_root.expanduser().resolve(), require_raw=False)
            protocol["frozen_input_sha256"] = {row["key"]:{k:v["sha256"] for k,v in row["sources"].items()} for row in audit}
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(protocol, indent=2), encoding="utf-8")
            print(f"[DONE] Baseline protocol: {path}")
            for warning in protocol["discovery_warnings"]:
                print("[ALIGNMENT] "+warning)
        elif args.command == "check":
            protocol, root = load_protocol(args.protocol)
            _, audit = check_inputs(protocol, root, require_raw=not args.metadata_only)
            for row in audit:
                print(f"[CHECK] {row['key']}: {row['frames']} frames, RAW={'present' if row['raw_exists'] else 'MISSING'}, worst speed error {row['worst_speed_error_percent']:.3f}%, {row['alignment_source']}")
            for warning in protocol.get("discovery_warnings", []):
                print("[ALIGNMENT] "+warning)
        elif args.command == "rescore":
            from rig_vpr.rescore import rescore
            rescore(args.results, args.output_root, args.no_plots, args.overwrite)
        elif args.command == "compare-references":
            from rig_vpr.references import compare_references
            compare_references(args.results,args.tolerance_deg,args.output_root,args.no_plots,args.overwrite,args.normalization,args.baseline_comparison)
        elif args.command == "angular-study":
            from rig_vpr.angular import angular_study
            angular_study(args.source_results,args.accumulation_deg,args.tolerance_deg,args.output_root,args.prepare_only,args.no_plots)
        elif args.command == "angle-sectors":
            from rig_vpr.sectors import analyse_sectors
            analyse_sectors(args.raw_results,args.normalized_results,args.sector_deg,args.output_root,args.no_plots,args.sector_start_deg)
        elif args.command == "stationary":
            from rig_vpr.stationary import stationary_analysis
            stationary_analysis(args.source_results,args.duration_s,args.guard_s,args.startup_s,args.bin_ms,args.output_root,args.prepare_only,args.no_plots)
        elif args.command == "revolution-study":
            from rig_vpr.revolutions import revolution_study
            revolution_study(args.source_results,args.low_revolutions,args.medium_revolutions,args.high_revolutions,args.motion_phase,args.tolerance_deg,args.output_root,args.prepare_only,args.no_plots)
        else:
            run(args.protocol, args.no_plots, args.overwrite)
    except (OSError, ValueError, RuntimeError, KeyError, ImportError) as exc:
        cli.exit(1, f"[ERROR] {exc}\n")


if __name__ == "__main__":
    main()
