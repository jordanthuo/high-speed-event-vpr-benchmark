"""Re-evaluate saved score matrices without telemetry, caches or RAW decoding."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from . import VERSION
from .algorithm import code_fingerprints, evaluate_scores, metrics, runtime_versions
from .data import read_csv, sha256, validate_protocol
from .reporting import candidate_rows, plotting, report, speed_summary, summary_figure, top5_query_fields, write_csv


def rescore(results_path, output_root=None, no_plots=False, overwrite=False):
    source = Path(results_path).expanduser().resolve()
    output = Path(output_root).expanduser().absolute() if output_root else source.parent/"ranked_metrics"
    if output.is_symlink():
        raise ValueError("Ranked output must not be a symlink")
    output = output.resolve()
    if output == source or output in source.parents or source in output.parents:
        raise ValueError("Ranked output must be separate from the source results directory")
    previous = output.is_dir() and any(output.iterdir())
    if previous and (not overwrite or not (output/"ranking_started.json").is_file()):
        raise FileExistsError(f"{output} is not empty. Use another output directory, or --overwrite for an existing ranked-metrics run.")
    complete = json.loads((source/"run_complete.json").read_text(encoding="utf-8"))
    protocol = json.loads((source/"protocol_snapshot.json").read_text(encoding="utf-8"))
    provenance = json.loads((source/"provenance.json").read_text(encoding="utf-8"))
    validate_protocol(protocol)
    if provenance["protocol"] != protocol or complete["protocol_sha256"] != provenance["protocol_sha256"]:
        raise ValueError("Completed source run, protocol snapshot and provenance disagree")
    prior_rows = read_csv(source/"summary_per_acquisition.csv")
    prior = {(r["reference"], r["query"]):r for r in prior_rows}
    if len(prior) != len(prior_rows):
        raise ValueError("Duplicate pairs in original summary")
    entries = protocol["acquisitions"]
    refs = entries if protocol["pair_mode"] == "all-independent-pairs" else [e for e in entries if e["key"] == protocol["reference"]]
    expected = [(r,q) for r in refs for q in entries if r["run_id"] != q["run_id"]]
    expected_keys = {(r["key"],q["key"]) for r,q in expected}
    if len(expected) != complete["pairs"] or expected_keys != set(prior):
        raise ValueError("Original summary does not contain exactly the completed independent pairs")
    if not no_plots:
        plotting()
    # Validate/recompute every pair before creating or archiving any output.
    cfg = protocol["settings"]
    pairs, matches, sensitivity, candidates, source_files = [], [], [], [], {}
    for ref_entry, qry_entry in expected:
        ref, qry = ref_entry["key"], qry_entry["key"]
        folder = source/"pairs"/(ref.replace("/","_")+"__"+qry.replace("/","_"))
        matrix_path = folder/"distance_matrix.npz"
        with np.load(matrix_path, allow_pickle=False) as stored:
            result = evaluate_scores(stored["scores"], stored["reference_angles_deg"], stored["query_angles_deg"],
                stored["reference_valid"], stored["query_valid"], cfg["tolerance_deg"])
        summary = metrics(result, cfg["tolerance_deg"])
        old = prior[(ref,qry)]
        for field in ("query_count", "correct_queries", "matched_queries", "empty_queries", "ambiguous_queries"):
            if int(old[field]) != summary[field]:
                raise ValueError(f"{qry}: Recall@1 regression in {field}; original results retained")
        if not np.isclose(float(old["recall_at_1"]), summary["recall_at_1"], rtol=0, atol=1e-12):
            raise ValueError(f"{qry}: Recall@1 does not reproduce the original score")
        pairs.append(dict(reference=ref, query=qry, query_condition=qry_entry["condition"], query_run=qry_entry["run"],
            nominal_query_platform_rpm=qry_entry["nominal_platform_rpm"], tolerance_deg=cfg["tolerance_deg"],
            reference_frames=len(result["reference_valid"]), reference_empty_descriptors=int(np.sum(~result["reference_valid"])),
            distance_backend=old["distance_backend"], **summary))
        old_matches = read_csv(folder/"matches.csv")
        if len(old_matches) != summary["query_count"] or {int(r["query_index"]) for r in old_matches} != set(range(summary["query_count"])):
            raise ValueError(f"{qry}: source match rows differ from score-matrix queries")
        for row in sorted(old_matches, key=lambda r:int(r["query_index"])):
            i = int(row["query_index"])
            if row["reference"] != ref or row["query"] != qry or not np.isclose(float(row["query_wrapped_angle_deg"]),result["query_angles_deg"][i],rtol=0,atol=1e-8):
                raise ValueError(f"{qry}: source match metadata disagrees with saved angles")
            matches.append(dict(row, **top5_query_fields(result, i)))
        candidates.extend(candidate_rows(ref,qry,result,cfg["tolerance_deg"]))
        for tolerance in sorted(set([.5,1.,2.,5.,10.,cfg["tolerance_deg"]])):
            sensitivity.append(dict(reference=ref, query=qry, tolerance_deg=tolerance, **metrics(result,tolerance)))
        source_files[matrix_path.relative_to(source).as_posix()] = sha256(matrix_path)
        source_files[(folder/"matches.csv").relative_to(source).as_posix()] = sha256(folder/"matches.csv")
        print(f"[RESCORE] {qry}: Recall@1={100*summary['recall_at_1']:.2f}%, Recall@5={100*summary['recall_at_5']:.2f}% within {cfg['tolerance_deg']:g} degrees",flush=True)
    if previous:
        archive = output.with_name(output.name+".previous-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        if archive.resolve().parent != output.parent:
            raise ValueError("Ranked archive would leave the output parent directory")
        output.rename(archive)
    output.mkdir(parents=True,exist_ok=True)
    (output/"ranking_started.json").write_text(json.dumps(dict(version=VERSION,source_results=str(source),started_utc=datetime.now(timezone.utc).isoformat()),indent=2),encoding="utf-8")
    summaries = speed_summary(pairs)
    write_csv(output/"summary_per_acquisition.csv",pairs)
    write_csv(output/"summary_by_speed.csv",summaries)
    write_csv(output/"tolerance_sensitivity.csv",sensitivity)
    write_csv(output/"matches.csv",matches)
    write_csv(output/"top_candidates.csv",candidates)
    report(output/"README_results.md",protocol,pairs,summaries)
    if not no_plots:
        for k in (1,5):
            summary_figure(output/"figures",pairs,summaries,cfg["tolerance_deg"],k=k)
    for name in ("run_complete.json","protocol_snapshot.json","provenance.json","summary_per_acquisition.csv"):
        source_files[name] = sha256(source/name)
    ranked_provenance = dict(version=VERSION,source_results=str(source),source_run_complete=complete,
        protocol=protocol,source_sha256=source_files,code_sha256=code_fingerprints(),runtime=runtime_versions(),
        uses_saved_distance_matrices_only=True,raw_decoding_performed=False,original_recall_at_1_verified=True,
        tie_policy="Candidates tied across rank five are not credited; strictly higher candidates retain credit. Stable index ordering is display-only.")
    (output/"ranking_provenance.json").write_text(json.dumps(ranked_provenance,indent=2),encoding="utf-8")
    (output/"rescore_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),pairs=len(pairs),
        primary_tolerance_deg=cfg["tolerance_deg"],source_protocol_sha256=complete["protocol_sha256"],version=VERSION),indent=2),encoding="utf-8")
    print(f"[DONE] Ranked metrics from {len(pairs)} saved matrices: {output}",flush=True)
    return output
