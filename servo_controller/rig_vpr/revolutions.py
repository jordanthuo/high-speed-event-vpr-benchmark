"""Vary query revolutions while holding the independent reference acquisition fixed."""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime,timezone
import json
from pathlib import Path
import numpy as np

from . import VERSION
from .algorithm import code_fingerprints,extract,match,metrics,pixel_indices,runtime_versions
from .data import CONDITIONS,accumulation_label,frame_plan,read_csv,sha256
from .inspection import completed_source,frozen_cache,frozen_trace,verify_raw
from .reporting import plotting,save_figure,top5_query_fields,write_csv


def parse_revolutions(text,trace):
    if text=="all":
        return list(range(max(0,int(np.ceil((trace["angles"][-1]-trace["angles"][0])/360)))+1))
    selected=set()
    for part in text.split(','):
        if not part.strip():
            raise ValueError("Revolution lists must contain integers or inclusive ranges such as 0,1,3-5")
        bits=part.strip().split('-')
        try:
            if len(bits)==1:
                selected.add(int(bits[0]))
            elif len(bits)==2:
                a,b=map(int,bits)
                if b<a:
                    raise ValueError()
                selected.update(range(a,b+1))
            else:
                raise ValueError()
        except ValueError:
            raise ValueError("Use nonnegative revolution integers or inclusive ranges") from None
    if not selected or min(selected)<0 or max(selected)>10000:
        raise ValueError("Use revolution indices in [0,10000]")
    return sorted(selected)


def phase_label(plan,nominal):
    if plan["worst_speed_error_percent"]<=5:
        return "nominal-speed"
    change=float(plan["speed_rpm"][-1]-plan["speed_rpm"][0])
    if change>nominal*.05:
        return "acceleration-or-mixed"
    if change<-nominal*.05:
        return "deceleration-or-mixed"
    return "varying-or-mixed"


def revolution_figures(output,summaries,protocol,tolerance):
    plt=plotting();fig,axes=plt.subplots(1,3,figsize=(12.5,4.5),sharey=True)
    for ax,(condition,(rpm,_)) in zip(axes,CONDITIONS.items()):
        for mode,color,marker,line,label in (("none","#0072B2","o","-","Raw counts"),("l1","#D55E00","s","--","L1-normalised")):
            rows=sorted([r for r in summaries if r["query_condition"]==condition and r["normalization"]==mode],key=lambda r:r["query_revolution"])
            if rows:
                x=[r["query_revolution"] for r in rows]
                ax.fill_between(x,[100*r["minimum_recall_at_5"] for r in rows],[100*r["maximum_recall_at_5"] for r in rows],color=color,alpha=.13)
                ax.plot(x,[100*r["mean_recall_at_5"] for r in rows],color=color,marker=marker,linestyle=line,label=label)
        original=next(e["revolution"] for e in protocol["acquisitions"] if e["condition"]==condition)
        ax.axvline(original,color="gray",linestyle=":",linewidth=1)
        ax.set(title=f"{condition.replace('-',' ')} queries (target {rpm:g} rpm)",xlabel="Query revolution (starting at zero)",ylim=(-3,103));ax.grid(axis="y",alpha=.2)
    axes[0].set_ylabel(f"Recall@5 (%) within ±{tolerance:g}°")
    fig.legend(*axes[0].get_legend_handles_labels(),loc="upper center",bbox_to_anchor=(.5,.9),ncol=2,frameon=False)
    fig.suptitle(f"Fixed {protocol['reference']} reference; {accumulation_label(protocol['settings'])} windows",fontsize=12)
    fig.text(.5,.02,"Bands: min-max over available query acquisitions. Dotted lines: original query turns. Counts and measured speeds are in the CSVs.",ha="center",fontsize=8)
    fig.subplots_adjust(top=.73,bottom=.18,wspace=.15)
    save_figure(plt,fig,output/"figures/recall_by_revolution")


def revolution_study(source_results,low="all",medium="all",high="all",motion_phase="any",tolerance=5.,output_root=None,prepare_only=False,no_plots=False):
    if motion_phase not in ("any","steady") or not np.isfinite(tolerance) or not 0<tolerance<=180:
        raise ValueError("Use any/steady motion phase and an angular tolerance in (0,180]")
    source=completed_source(source_results);protocol=source["protocol"];cfg=dict(protocol["settings"],motion_screen="forward" if motion_phase=="any" else "steady")
    entries=protocol["acquisitions"];reference_entry=next(e for e in entries if e["key"]==protocol["reference"])
    reference,ref_audit=frozen_cache(source,reference_entry)
    indices=pixel_indices(cfg)
    if not np.array_equal(indices,np.load(source["path"]/"selected_spatial_bins.npy",allow_pickle=False)):
        raise ValueError("Source spatial selection changed")
    selections=dict(zip(CONDITIONS,(low,medium,high)))
    planned=[];plans={};traces={};inputs={}
    for entry in entries:
        trace,used=frozen_trace(source,entry);traces[entry["key"]]=trace;inputs[entry["key"]]=used
        if entry["run_id"]==reference_entry["run_id"]:
            continue
        for rev in parse_revolutions(selections[entry["condition"]],trace):
            row=dict(query=entry["key"],query_condition=entry["condition"],query_revolution=rev,original_query_revolution=entry["revolution"])
            try:
                plan=frame_plan(trace,dict(entry,revolution=rev),cfg)
            except ValueError as exc:
                planned.append(dict(**row,status="skipped",reason=str(exc)));continue
            plans[(entry["key"],rev)]=plan
            planned.append(dict(**row,status="selected",reason="",motion_phase=phase_label(plan,entry["nominal_platform_rpm"]),
                camera_start_s=plan["turn_start_s"],camera_end_s=plan["turn_end_s"],
                mean_sampled_platform_rpm=float(plan["speed_rpm"].mean()),minimum_feedback_rpm=plan["minimum_feedback_rpm"],maximum_feedback_rpm=plan["maximum_feedback_rpm"],
                worst_speed_error_percent=plan["worst_speed_error_percent"]))
    if not plans:
        raise ValueError("No requested complete forward revolutions passed the selected motion screen")
    settings=dict(source_protocol_sha256=source["complete"]["protocol_sha256"],reference=protocol["reference"],reference_revolution=reference_entry["revolution"],
        reference_cache=ref_audit,motion_phase=motion_phase,tolerance_deg=tolerance,selections=selections,planned=planned)
    output=Path(output_root).expanduser().resolve() if output_root else source["path"].parent.with_name("VPR-SAD-revolutions")
    if output==source["path"].parent or source["path"].parent in output.parents or output in source["path"].parent.parents:
        raise ValueError("Revolution output must be separate from source results/cache")
    if output.exists() and any(output.iterdir()):
        if not (output/"revolution_plan.json").is_file() or json.loads((output/"revolution_plan.json").read_text(encoding="utf-8"))!=settings:
            raise FileExistsError("Revolution output is not empty or settings differ; choose another output root")
        if (output/"revolution_study_complete.json").exists():
            raise FileExistsError("Revolution study already completed; choose another output root")
    output.mkdir(parents=True,exist_ok=True)
    (output/"revolution_plan.json").write_text(json.dumps(settings,indent=2),encoding="utf-8")
    write_csv(output/"planned_turns.csv",planned)
    if prepare_only:
        print(f"[PREPARED ONLY] {len(plans)} query turns selected; {len(planned)-len(plans)} skipped. No RAW extracted: {output}",flush=True);return output
    entry_lookup={e["key"]:e for e in entries}
    for key,rev in plans:
        if rev!=entry_lookup[key]["revolution"]:
            verify_raw(source,entry_lookup[key])
    if not no_plots:
        plotting()
    selected_rows={(r["query"],r["query_revolution"]):r for r in planned if r["status"]=="selected"}
    summaries=[];query_rows=[];frame_rows=[];cache_inputs={};matrices=[]
    original_pairs={(r["reference"],r["query"]):r for r in read_csv(source["path"]/"summary_per_acquisition.csv")}
    for (key,rev),plan in plans.items():
        entry=entry_lookup[key]
        if rev==entry["revolution"]:
            query,audit=frozen_cache(source,entry)
            if not np.array_equal(query["start_us"],plan["start_us"]) or not np.array_equal(query["end_us"],plan["end_us"]):
                raise ValueError("Original-turn windows changed")
            audit=dict(**audit,origin="original_frozen_cache")
        else:
            query,metadata=extract(dict(entry,revolution=rev),cfg,plan,source["root"],output/"cache",dict(sources=inputs[key]))
            if metadata["raw_origin_us"]!=source["audit"]["inputs"][key]["raw_origin_us"]:
                raise ValueError("New query extraction changed the RAW time origin")
            audit=dict(origin="new_turn_extraction",metadata=metadata,sha256=sha256(output/"cache"/metadata["cache_file"]))
        cache_inputs[key+f"/rev{rev}"]=audit
        for i in range(len(query["frames"])):
            frame_rows.append(dict(query=key,query_revolution=rev,frame_index=i,camera_center_s=float(query["center_us"][i]/1e6),
                requested_angle_deg=float(query["requested_angle_deg"][i]),platform_rpm=float(query["speed_rpm"][i]),event_count=int(query["event_counts"][i])))
        for mode in ("none","l1"):
            result=match(reference,query,dict(cfg,normalization=mode,tolerance_deg=tolerance),indices)
            if mode=="none" and rev==entry["revolution"] and (reference_entry["key"],key) in original_pairs:
                old=original_pairs[(reference_entry["key"],key)]
                check=metrics(result,protocol["settings"]["tolerance_deg"])
                if check["correct_queries"]!=int(old["correct_queries"]) or not np.isclose(check["recall_at_1"],float(old["recall_at_1"]),rtol=0,atol=1e-12):
                    raise ValueError("Original-turn baseline Recall@1 did not reproduce")
            summary=metrics(result,tolerance)
            row=dict(normalization=mode,reference=reference_entry["key"],reference_revolution=reference_entry["revolution"],
                **selected_rows[(key,rev)],tolerance_deg=tolerance,**summary)
            summaries.append(row)
            folder=output/"distance_matrices";folder.mkdir(exist_ok=True)
            matrix=folder/(key.replace('/','_')+f"_rev{rev}_{mode}.npz")
            np.savez_compressed(matrix,scores=result["scores"],reference_angles_deg=reference["angle_deg"],query_angles_deg=query["angle_deg"],reference_valid=result["reference_valid"],query_valid=result["query_valid"])
            matrices.append(dict(file=matrix.relative_to(output).as_posix(),sha256=sha256(matrix)))
            for i in range(summary["query_count"]):
                query_rows.append(dict(normalization=mode,reference=reference_entry["key"],query=key,query_revolution=rev,query_index=i,
                    requested_angle_deg=float(query["requested_angle_deg"][i]),measured_angle_deg=float(query["angle_deg"][i]),
                    platform_rpm=float(query["speed_rpm"][i]),event_count=int(query["event_counts"][i]),
                    status=str(result["status"][i]),correct_at_1=bool(result["correct"][i]),**top5_query_fields(result,i)))
        print(f"[REVOLUTION] {key} rev {rev}: {selected_rows[(key,rev)]['motion_phase']}",flush=True)
    groups=defaultdict(list)
    for row in summaries:
        groups[(row["normalization"],row["query_condition"],row["query_revolution"])].append(row)
    condition_rows=[]
    for (mode,condition,rev),selected in groups.items():
        values=[r["recall_at_5"] for r in selected]
        condition_rows.append(dict(normalization=mode,query_condition=condition,query_revolution=rev,query_acquisitions=len(selected),
            mean_recall_at_5=float(np.mean(values)),minimum_recall_at_5=min(values),maximum_recall_at_5=max(values),
            mean_recall_at_1=float(np.mean([r["recall_at_1"] for r in selected])),queries=sum(r["query_count"] for r in selected)))
    baseline={(r["normalization"],r["query"]):r for r in summaries if r["query_revolution"]==r["original_query_revolution"]}
    delta=[]
    for row in summaries:
        before=baseline.get((row["normalization"],row["query"]))
        delta.append(dict(normalization=row["normalization"],query=row["query"],query_revolution=row["query_revolution"],
            original_query_revolution=row["original_query_revolution"],recall_at_5=row["recall_at_5"],
            baseline_recall_at_5=before["recall_at_5"] if before else None,change_pp=100*(row["recall_at_5"]-before["recall_at_5"]) if before else None))
    write_csv(output/"summary_per_turn.csv",summaries);write_csv(output/"summary_by_condition_revolution.csv",condition_rows)
    write_csv(output/"delta_vs_original_turn.csv",delta);write_csv(output/"query_metrics.csv",query_rows);write_csv(output/"frame_windows.csv",frame_rows)
    if not no_plots:
        revolution_figures(output,condition_rows,protocol,tolerance)
    text=["# Query-revolution sensitivity","",
        f"The reference is fixed to {reference_entry['key']}, revolution {reference_entry['revolution']}. Every query uses another recording; every turn from the reference recording is excluded. Accumulation stays {accumulation_label(cfg)}. Raw and L1-normalised SAD use Recall@5 within ±{tolerance:g}°.","",
        "Revolutions start at zero. Incomplete/non-forward turns and turns failing the chosen screen are explicitly listed as skipped. With motion-phase any, acceleration/deceleration are admitted only when the complete turn and its event apertures have coverage. Nominal target labels are not actual speeds: inspect measured min/max feedback and angular-sampled mean speed. Phase labels are coarse descriptive labels, not independent motion-phase measurements.","",
        "The existing baseline reference and source alignment remain frozen. For manual runs the mapping is reconstructed from hash-checked telemetry and the completed protocol's accepted onset, then checked against all original sampled centres. Current setup/catalogue onsets are never substituted. Original-turn cached queries are reused; new turns require RAW extraction.","",
        "Repeated turns reuse recordings and are not independent acquisitions. Summary means use the available query acquisitions at each revolution; skipped turns can change their membership. Delta-versus-baseline fields are populated only where that recording's original query turn was included in this study. No reference is chosen using the resulting recognition scores.",""]
    (output/"README.md").write_text("\n".join(text),encoding="utf-8")
    (output/"revolution_provenance.json").write_text(json.dumps(dict(version=VERSION,settings=settings,source_protocol=protocol,inputs=inputs,cache_inputs=cache_inputs,
        matrices=matrices,code_sha256=code_fingerprints(),runtime=runtime_versions()),indent=2),encoding="utf-8")
    (output/"revolution_study_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),query_turns=len(plans),metric_rows=len(summaries)),indent=2),encoding="utf-8")
    print(f"[DONE] Fixed-reference query-revolution study: {output}",flush=True);return output
