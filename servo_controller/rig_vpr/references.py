"""Compare reference acquisitions using frozen frame caches, without RAW decoding."""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path

import numpy as np

from . import VERSION
from .algorithm import code_fingerprints, descriptors, evaluate_scores, metrics, pixel_indices, runtime_versions, sad_matrix
from .data import CONDITIONS, accumulation_label, read_csv, sha256, validate_protocol
from .reporting import plotting, save_figure, speed_summary, top5_query_fields, write_csv


def load_cached_features(source, protocol, provenance):
    """Bind each cache to the completed run and verify its counts/angle index."""
    cache_dir = source.parent/"cache"
    windows = read_csv(source/"frame_windows.csv")
    window_lookup = {(r["acquisition"],int(r["frame_index"])):r for r in windows}
    if len(window_lookup) != len(windows):
        raise ValueError("Duplicate frame-window rows in the completed run")
    cfg = protocol["settings"]
    shape = (round(360/cfg["angle_step_deg"]),2 if cfg["polarity"]=="split" else 1,cfg["height"],cfg["width"])
    features, audit = {}, {}
    for entry in protocol["acquisitions"]:
        key = entry["key"]
        expected = provenance["inputs"][key]
        filename = expected["cache_file"]
        if Path(filename).name != filename:
            raise ValueError("Cache filenames must be basenames")
        cache_path = cache_dir/filename
        with np.load(cache_path,allow_pickle=False) as stored:
            metadata = json.loads(stored["metadata"].item())
            stamp = {k:metadata[k] for k in ("version","entry","settings","sources","code","raw")}
            identity = hashlib.sha256(json.dumps(stamp,sort_keys=True).encode()).hexdigest()
            if stored["identity"].item()!=identity or metadata!=expected or metadata["entry"]!=entry or metadata["settings"]!=cfg:
                raise ValueError(f"{key}: cached extraction does not match the completed source run")
            data = {k:stored[k].copy() for k in ("frames","angle_deg","event_counts","center_us","start_us","end_us")}
        frames = data["frames"]
        if frames.shape!=shape or not np.all(np.isfinite(frames)) or np.any(frames<0):
            raise ValueError(f"{key}: invalid cached numeric count frames")
        n = shape[0]
        if any(data[k].shape!=(n,) for k in data if k!="frames"):
            raise ValueError(f"{key}: cache index lengths disagree")
        if not np.allclose(frames.sum(axis=(1,2,3),dtype=np.float64),data["event_counts"],rtol=0,atol=.1):
            raise ValueError(f"{key}: cached counts do not reconcile with event totals")
        for i in range(n):
            row = window_lookup.get((key,i))
            if row is None or not np.isclose(float(row["wrapped_angle_deg"]),data["angle_deg"][i],rtol=0,atol=1e-8) or int(row["event_count"])!=data["event_counts"][i] or not np.isclose(float(row["center_s"]),data["center_us"][i]/1e6,rtol=0,atol=1e-8):
                raise ValueError(f"{key}: cached frames disagree with the source frame-window audit")
        features[key] = data
        audit[key] = dict(cache_file=filename,sha256=sha256(cache_path),identity=identity,source_metadata=metadata)
    if len(windows)!=sum(len(d["angle_deg"]) for d in features.values()):
        raise ValueError("Unexpected acquisitions or frames in the frame-window audit")
    return features,audit


def reference_condition_summary(summaries,entries):
    """Descriptive variation across reference means, avoiding pooled-query precision."""
    lookup={e["key"]:e for e in entries}
    output=[]
    for ref_condition in CONDITIONS:
        for qry_condition in CONDITIONS:
            selected=[r for r in summaries if lookup[r["reference"]]["condition"]==ref_condition and r["query_condition"]==qry_condition]
            r1=np.array([r["mean_recall_at_1"] for r in selected])
            r5=np.array([r["mean_recall_at_5"] for r in selected])
            output.append(dict(reference_condition=ref_condition,query_condition=qry_condition,
                reference_acquisitions=len(selected),ordered_pairs=sum(r["independent_query_acquisitions"] for r in selected),
                mean_recall_at_1=float(r1.mean()),mean_recall_at_5=float(r5.mean()),
                sd_across_reference_means_recall_at_5=float(r5.std(ddof=1)) if len(r5)>1 else None,
                minimum_reference_mean_recall_at_5=float(r5.min()),maximum_reference_mean_recall_at_5=float(r5.max())))
    return output


def comparison_figures(folder,summaries,grouped,entries,tolerance,normalization="none"):
    plt=plotting()
    labels=[e["key"] for e in entries]
    conditions=list(CONDITIONS)
    look={(r["reference"],r["query_condition"]):r for r in summaries}
    values=np.array([[100*look[(key,c)]["mean_recall_at_5"] for c in conditions] for key in labels])
    for name,array,ylabs,size,ytitle in (
        ("reference_runs_recall_at_5",values,labels,(7.8,6.2),"Reference acquisition"),
        ("reference_speeds_recall_at_5",np.array([[100*next(r for r in grouped if r["reference_condition"]==ref and r["query_condition"]==qry)["mean_recall_at_5"] for qry in conditions] for ref in conditions]),conditions,(6.6,4.5),"Reference speed")):
        fig,ax=plt.subplots(figsize=size)
        image=ax.imshow(array,vmin=0,vmax=100,cmap="viridis",aspect="auto")
        for i in range(len(ylabs)):
            for j in range(3):
                value=array[i,j]
                ax.text(j,i,f"{value:.1f}%",ha="center",va="center",color="black" if value>=60 else "white",fontsize=12)
        ax.set_xticks(range(3),[f"{CONDITIONS[c][0]:g} rpm" for c in conditions])
        ax.set_yticks(range(len(ylabs)),ylabs if len(ylabs)>3 else [f"{CONDITIONS[c][0]:g} rpm" for c in ylabs])
        label="L1-normalised counts" if normalization=="l1" else "Raw counts"
        ax.set(xlabel="Query speed",ylabel=ytitle,title=f"{label}: Recall@5 within ±{tolerance:g}°")
        fig.colorbar(image,ax=ax,label="Recall@5 (%)")
        fig.text(.5,.01,"Each reference excludes its own acquisition from the query set.",ha="center",fontsize=9)
        fig.subplots_adjust(bottom=.16,left=.28 if len(ylabs)>3 else .16,right=.87,top=.90)
        save_figure(plt,fig,Path(folder)/name)


def comparison_report(path,summaries,grouped,entries,cfg,tolerance):
    lookup={(r["reference"],r["query_condition"]):r for r in summaries}
    lines=["# Reference-acquisition comparison", "",
        f"Recall@5 at an angular tolerance of **±{tolerance:g}°**. Each of nine acquisitions serves as reference for the other eight acquisitions (72 ordered pairs).", "",
        f"Uses frozen {accumulation_label(cfg)} count windows, {cfg['width']} × {cfg['height']} bins, {cfg['polarity']} polarity and **{cfg['normalization']}** descriptor normalization. Frame centres, camera alignment and selected spatial bins are retained from the completed source run.", "",
        "| Reference | Low queries | Medium queries | High queries |", "|---|---:|---:|---:|"]
    for entry in entries:
        values=[lookup[(entry["key"],c)]["mean_recall_at_5"]*100 for c in CONDITIONS]
        lines.append(f"| {entry['key']} | {values[0]:.2f}% | {values[1]:.2f}% | {values[2]:.2f}% |")
    lines += ["", "Each cell is the equal-weight mean over query acquisitions. A reference has two queries at its own speed and three queries at either other speed.", "",
        "## Interpretation limits", "",
        "- A within-speed reference change also changes which same-speed acquisition is excluded from the queries. Cross-speed reference changes within a reference speed use the same three query acquisitions.",
        "- The 72 ordered comparisons reuse nine recordings. They are not 72 independent experiments. Variation across reference means is descriptive, with shared queries.",
        "- L1 distance is symmetric, but retrieval is directional: swapping the reference changes the candidate library. Recall can therefore differ when the same two acquisitions swap roles.",
        "- The five candidates are angular samples and may be neighbouring views, not five independent physical places. Empty queries fail, and ties spilling over rank five do not receive credit.",
        "- This evaluates cached count frames from the completed run. Camera alignment, accumulation, spatial grid and pixel mask remain fixed. The descriptor normalization used is stated above.",
        "- This is an additional tolerance/reference sensitivity analysis. It does not retrospectively replace the original predeclared primary configuration.", ""]
    Path(path).write_text("\n".join(lines),encoding="utf-8")


def compare_references(results_path,tolerance=5.,output_root=None,no_plots=False,overwrite=False,normalization=None,baseline_comparison=None):
    if not np.isfinite(tolerance) or not 0<tolerance<=180:
        raise ValueError("Use an angular tolerance in (0,180]")
    source=Path(results_path).expanduser().resolve()
    if normalization not in (None,"none","l1"):
        raise ValueError("Normalization must be none or l1")
    suffix="" if normalization is None else normalization+"_"
    output=Path(output_root).expanduser().absolute() if output_root else source.parent/("reference_comparison_"+suffix+f"{tolerance:g}".replace(".","p")+"deg")
    if output.is_symlink():
        raise ValueError("Comparison output must not be a symlink")
    output=output.resolve()
    cache_root=source.parent/"cache"
    if output==source or output in source.parents or source in output.parents or output==cache_root or cache_root in output.parents:
        raise ValueError("Comparison output must be separate from source results and caches")
    previous=output.is_dir() and any(output.iterdir())
    if previous and (not overwrite or not (output/"reference_comparison_started.json").is_file()):
        raise FileExistsError(f"{output} is not empty. Use another destination or --overwrite for a prior reference comparison.")
    complete=json.loads((source/"run_complete.json").read_text(encoding="utf-8"))
    protocol=json.loads((source/"protocol_snapshot.json").read_text(encoding="utf-8"))
    provenance=json.loads((source/"provenance.json").read_text(encoding="utf-8"))
    validate_protocol(protocol)
    if provenance["protocol"]!=protocol or provenance["protocol_sha256"]!=complete["protocol_sha256"]:
        raise ValueError("Completed run, protocol snapshot and provenance disagree")
    if not no_plots:
        plotting()
    features,cache_audit=load_cached_features(source,protocol,provenance)
    cfg=protocol["settings"]
    evaluation_cfg=dict(cfg,normalization=normalization if normalization is not None else cfg["normalization"])
    same_normalization=evaluation_cfg["normalization"]==cfg["normalization"]
    baseline=None
    if baseline_comparison is not None:
        if evaluation_cfg["normalization"]!="l1":
            raise ValueError("Paired normalization comparison requires l1 descriptors")
        from .normalization import load_baseline_comparison
        baseline=load_baseline_comparison(baseline_comparison,protocol,tolerance,cache_audit)
        if output==baseline["path"] or output in baseline["path"].parents or baseline["path"] in output.parents:
            raise ValueError("Comparison output must be separate from the baseline comparison")
    entries=protocol["acquisitions"]
    indices=pixel_indices(cfg)
    if not np.array_equal(indices,np.load(source/"selected_spatial_bins.npy",allow_pickle=False)):
        raise ValueError("Selected spatial bins differ from the completed run")
    descriptor_sets={key:descriptors(d["frames"],indices,evaluation_cfg["normalization"]) for key,d in features.items()}
    if any(not valid.any() for _,valid in descriptor_sets.values()):
        raise ValueError("An acquisition has no nonempty reference descriptors")
    if previous:
        archive=output.with_name(output.name+".previous-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        if archive.resolve().parent!=output.parent:
            raise ValueError("Archive would leave output parent")
        output.rename(archive)
    output.mkdir(parents=True,exist_ok=True)
    (output/"reference_comparison_started.json").write_text(json.dumps(dict(started_utc=datetime.now(timezone.utc).isoformat(),version=VERSION,tolerance_deg=tolerance,evaluation_settings=evaluation_cfg),indent=2),encoding="utf-8")
    matrix_dir=output/"distance_matrices"; matrix_dir.mkdir()
    prior={(r["reference"],r["query"]):r for r in read_csv(source/"summary_per_acquisition.csv")}
    pair_rows,query_rows,matrix_audit=[],[],[]
    reused,computed=0,0
    for left,right in combinations(entries,2):
        a,b=left["key"],right["key"]
        raw_scores=None; reused_from=None; backend=None
        # Saved matrices can be reused in either direction if every reference was nonempty.
        for ref,qry,transpose in (((a,b,False),(b,a,True)) if same_normalization else ()):
            candidate=source/"pairs"/(ref.replace("/","_")+"__"+qry.replace("/","_"))/"distance_matrix.npz"
            if (ref,qry) not in prior or not candidate.is_file():
                continue
            with np.load(candidate,allow_pickle=False) as stored:
                if not np.all(stored["reference_valid"]) or not np.all(np.isfinite(stored["scores"])):
                    continue
                if not np.array_equal(stored["reference_angles_deg"],features[ref]["angle_deg"]) or not np.array_equal(stored["query_angles_deg"],features[qry]["angle_deg"]) or not np.array_equal(stored["reference_valid"],descriptor_sets[ref][1]) or not np.array_equal(stored["query_valid"],descriptor_sets[qry][1]):
                    raise ValueError(f"{ref}/{qry}: saved matrix metadata disagrees with cached features")
                # Check an angle-independent sample of distances against cached descriptors.
                ri=np.unique(np.linspace(0,len(descriptor_sets[ref][0])-1,4,dtype=int))
                qi=np.unique(np.linspace(0,len(descriptor_sets[qry][0])-1,4,dtype=int))
                sampled,_=sad_matrix(descriptor_sets[ref][0][ri],descriptor_sets[qry][0][qi])
                if not np.allclose(sampled,stored["scores"][np.ix_(ri,qi)],rtol=1e-6,atol=1e-6):
                    raise ValueError(f"{ref}/{qry}: sampled saved distances disagree with cached features")
                raw_scores=stored["scores"].T.copy() if transpose else stored["scores"].copy()
            reused_from=dict(path=candidate.relative_to(source).as_posix(),sha256=sha256(candidate),sampled_distance_check="up to 4 × 4 index samples")
            backend="saved-source-matrix"; reused+=1; break
        if raw_scores is None:
            raw_scores,backend=sad_matrix(descriptor_sets[a][0],descriptor_sets[b][0]); computed+=1
        name=a.replace("/","_")+"__"+b.replace("/","_")+".npz"
        matrix_path=matrix_dir/name
        np.savez_compressed(matrix_path,scores=raw_scores,reference_angles_deg=features[a]["angle_deg"],query_angles_deg=features[b]["angle_deg"],
            reference_valid=descriptor_sets[a][1],query_valid=descriptor_sets[b][1])
        matrix_audit.append(dict(matrix_file=name,left=a,right=b,sha256=sha256(matrix_path),backend=backend,reused_from=reused_from))
        for ref_entry,qry_entry,scores,transposed in ((left,right,raw_scores,False),(right,left,raw_scores.T,True)):
            ref,qry=ref_entry["key"],qry_entry["key"]
            evaluated=evaluate_scores(scores,features[ref]["angle_deg"],features[qry]["angle_deg"],descriptor_sets[ref][1],descriptor_sets[qry][1],tolerance)
            if same_normalization and (ref,qry) in prior:
                old=prior[(ref,qry)]; original=metrics(evaluated,cfg["tolerance_deg"])
                if original["correct_queries"]!=int(old["correct_queries"]) or original["query_count"]!=int(old["query_count"]) or not np.isclose(original["recall_at_1"],float(old["recall_at_1"]),rtol=0,atol=1e-12):
                    raise ValueError(f"{ref}/{qry}: original Recall@1 does not reproduce")
            summary=metrics(evaluated,tolerance)
            pair_rows.append(dict(reference=ref,reference_condition=ref_entry["condition"],reference_run=ref_entry["run"],query=qry,
                query_condition=qry_entry["condition"],query_run=qry_entry["run"],nominal_query_platform_rpm=qry_entry["nominal_platform_rpm"],
                tolerance_deg=tolerance,matrix_file=name,matrix_transposed=transposed,**summary))
            for i in range(summary["query_count"]):
                query_rows.append(dict(reference=ref,query=qry,query_index=i,query_wrapped_angle_deg=float(evaluated["query_angles_deg"][i]),
                    status=str(evaluated["status"][i]),correct_at_1=bool(evaluated["correct"][i]),
                    top5_reference_indices=";".join(str(int(v)) for v in evaluated["top_indices"][:,i]),
                    top5_credited_mask=";".join(str(int(v)) for v in evaluated["top_credited"][:,i]),**top5_query_fields(evaluated,i)))
        print(f"[REFERENCES] {len(matrix_audit)}/{len(entries)*(len(entries)-1)//2}: {a} and {b} ({backend})",flush=True)
    summaries=speed_summary(pair_rows)
    grouped=reference_condition_summary(summaries,entries)
    write_csv(output/"summary_per_pair.csv",pair_rows)
    write_csv(output/"summary_by_reference.csv",summaries)
    write_csv(output/"summary_by_reference_speed.csv",grouped)
    write_csv(output/"query_metrics.csv",query_rows)
    comparison_report(output/"README_results.md",summaries,grouped,entries,evaluation_cfg,tolerance)
    if not no_plots:
        comparison_figures(output/"figures",summaries,grouped,entries,tolerance,evaluation_cfg["normalization"])
    normalization_audit=None
    if baseline is not None:
        from .normalization import write_normalization_comparison
        normalization_audit=write_normalization_comparison(output,baseline,pair_rows,grouped,evaluation_cfg,tolerance,no_plots)
    audit=dict(version=VERSION,source_results=str(source),source_protocol=protocol,evaluation_tolerance_deg=tolerance,
        source_run_complete=complete,raw_decoding_performed=False,unchanged_cached_representation=same_normalization,
        unchanged_cached_count_frames=True,evaluation_settings=evaluation_cfg,normalization_comparison=normalization_audit,
        cached_inputs=cache_audit,matrices=matrix_audit,computed_unique_matrices=computed,reused_unique_matrices=reused,
        directed_pairs=len(pair_rows),code_sha256=code_fingerprints(),runtime=runtime_versions(),
        source_file_sha256={name:sha256(source/name) for name in ("run_complete.json","protocol_snapshot.json","provenance.json","summary_per_acquisition.csv","frame_windows.csv","selected_spatial_bins.npy")})
    (output/"comparison_provenance.json").write_text(json.dumps(audit,indent=2),encoding="utf-8")
    (output/"reference_comparison_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),
        tolerance_deg=tolerance,reference_acquisitions=len(entries),ordered_pairs=len(pair_rows),normalization=evaluation_cfg["normalization"],version=VERSION),indent=2),encoding="utf-8")
    print(f"[DONE] {len(pair_rows)} ordered comparisons at ±{tolerance:g} degrees: {output}",flush=True)
    return output
