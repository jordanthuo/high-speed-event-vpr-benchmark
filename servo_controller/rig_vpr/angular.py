"""Equal-angle accumulation study using new RAW extraction and frozen alignment."""
from __future__ import annotations

import copy
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from . import VERSION
from .algorithm import code_fingerprints, runtime_versions
from .data import CONDITIONS, accumulation_label, check_inputs, read_csv, relative, resolve, sha256, validate_protocol
from .references import compare_references
from .reporting import plotting, save_figure, write_csv


def ensure_comparison(source,tolerance,normalization,no_plots,baseline=None):
    suffix="" if normalization=="none" else "l1_"
    folder=source.parent/("reference_comparison_"+suffix+f"{tolerance:g}".replace(".","p")+"deg")
    protocol=json.loads((source/"protocol_snapshot.json").read_text(encoding="utf-8"))
    marker=folder/"reference_comparison_complete.json"
    if marker.is_file():
        audit=json.loads((folder/"comparison_provenance.json").read_text(encoding="utf-8"))
        cfg=audit.get("evaluation_settings",audit["source_protocol"]["settings"])
        if audit["source_protocol"]!=protocol or audit["evaluation_tolerance_deg"]!=tolerance or cfg["normalization"]!=normalization:
            raise ValueError(f"Existing comparison settings disagree: {folder}")
        if not (folder/"summary_per_pair.csv").is_file():
            raise ValueError(f"Completed comparison is missing its pair summary: {folder}")
        print(f"[REUSE] {folder}",flush=True)
        return folder
    return compare_references(source,tolerance,folder,no_plots,False,
        normalization=None if normalization=="none" else "l1",baseline_comparison=baseline)


def write_window_comparison(output,comparisons,protocol,tolerance,no_plots):
    pairs,groups,inputs=[],[],{}
    for normalization,(time_folder,angle_folder) in comparisons.items():
        time_audit=json.loads((time_folder/"comparison_provenance.json").read_text(encoding="utf-8"))
        angle_audit=json.loads((angle_folder/"comparison_provenance.json").read_text(encoding="utf-8"))
        # Compare the same recordings and fixed camera-to-encoder mapping.
        if time_audit["source_protocol"]["acquisitions"]!=angle_audit["source_protocol"]["acquisitions"]:
            raise ValueError("Time/angle comparisons do not use identical acquisitions/alignment")
        for key,value in time_audit["cached_inputs"].items():
            other=angle_audit["cached_inputs"][key]
            if value["source_metadata"]["sources"]!=other["source_metadata"]["sources"]:
                raise ValueError("Time/angle source telemetry changed")
            for field in ("size_bytes","first_and_last_mib_sha256"):
                if value["source_metadata"]["raw"][field]!=other["source_metadata"]["raw"][field]:
                    raise ValueError("Time/angle RAW identities disagree")
        for level,filename,id_fields,metric_prefix,target in (
            ("pair","summary_per_pair.csv",("reference","query"),"",pairs),
            ("speed","summary_by_reference_speed.csv",("reference_condition","query_condition"),"mean_",groups)):
            old={tuple(r[k] for k in id_fields):r for r in read_csv(time_folder/filename)}
            new=read_csv(angle_folder/filename)
            if len(old)!=len(new) or set(old)!={tuple(r[k] for k in id_fields) for r in new}:
                raise ValueError("Time/angle comparison pairs disagree")
            for row in new:
                previous=old[tuple(row[k] for k in id_fields)]
                item=dict(normalization=normalization,**{k:row[k] for k in id_fields})
                if level=="pair":
                    if row["query_count"]!=previous["query_count"]:
                        raise ValueError("Time/angle query denominators changed")
                    item.update(query_count=int(row["query_count"]),reference_condition=row["reference_condition"],query_condition=row["query_condition"])
                for k in (1,5):
                    raw=float(previous[f"{metric_prefix}recall_at_{k}"])
                    angular=float(row[f"{metric_prefix}recall_at_{k}"])
                    item.update({f"time_recall_at_{k}":raw,f"angle_recall_at_{k}":angular,f"change_pp_at_{k}":100*(angular-raw)})
                target.append(item)
        inputs[normalization]={label:{"path":str(folder),"sha256":{name:sha256(folder/name) for name in
            ("comparison_provenance.json","reference_comparison_complete.json","summary_per_pair.csv","summary_by_reference_speed.csv")}}
            for label,folder in (("time",time_folder),("angle",angle_folder))}
    write_csv(output/"window_comparison_per_pair.csv",pairs)
    write_csv(output/"window_comparison_by_speed.csv",groups)
    lines=["# Fixed-time versus fixed-angle accumulation","",
        f"Recall@5 within ±{tolerance:g}°. The angular representation uses {accumulation_label(protocol['settings'])} windows. Angular centres, selected revolutions, acquisition identities, alignment, spatial bins and query denominators are held fixed. Raw-count and L1-normalised SAD are compared separately.","",
        "| Normalization | Reference speed | Query speed | Time Recall@5 | Angle Recall@5 | Change (pp) |",
        "|---|---|---|---:|---:|---:|"]
    for row in groups:
        lines.append(f"| {row['normalization']} | {row['reference_condition']} | {row['query_condition']} | {100*row['time_recall_at_5']:.2f}% | {100*row['angle_recall_at_5']:.2f}% | {row['change_pp_at_5']:+.2f} |")
    lines += ["", "Each cell averages the reference-run means. All 72 directed pairs reuse nine recordings. This is a descriptive method comparison on existing acquisitions, not independent validation of a setting selected from these scores.","",
        "Angular boundaries are interpolated encoder crossings at centre angle ± half the angular width, rounded to microseconds. The matcher sees count descriptors only. Encoder angles determine extraction boundaries and evaluation labels, not candidate selection. Equal angular span does not guarantee identical event content or equal event counts.",""]
    (output/"README_window_comparison.md").write_text("\n".join(lines),encoding="utf-8")
    if not no_plots:
        plt=plotting(); conditions=list(CONDITIONS)
        fig,axes=plt.subplots(2,3,figsize=(12.6,8),layout="constrained")
        for i,normalization in enumerate(("none","l1")):
            look={(r["reference_condition"],r["query_condition"]):r for r in groups if r["normalization"]==normalization}
            temporal=np.array([[100*look[(a,b)]["time_recall_at_5"] for b in conditions] for a in conditions])
            angular=np.array([[100*look[(a,b)]["angle_recall_at_5"] for b in conditions] for a in conditions])
            for ax,array,title,delta in zip(axes[i],(temporal,angular,angular-temporal),("Time window","Angular window","Change (pp)"),(False,False,True)):
                im=ax.imshow(array,cmap="RdBu" if delta else "viridis",vmin=-100 if delta else 0,vmax=100)
                for a in range(3):
                    for b in range(3):
                        v=array[a,b]; color=("white" if abs(v)>55 else "black") if delta else ("black" if v>=60 else "white")
                        ax.text(b,a,f"{v:+.1f}" if delta else f"{v:.1f}%",ha="center",va="center",color=color)
                ax.set_xticks(range(3),[f"{CONDITIONS[c][0]:g}" for c in conditions]);ax.set_yticks(range(3),[f"{CONDITIONS[c][0]:g}" for c in conditions])
                ax.set(title=("Raw counts" if normalization=="none" else "L1-normalised")+": "+title,xlabel="Query speed (rpm)",ylabel="Reference speed (rpm)")
                fig.colorbar(im,ax=ax,fraction=.05,pad=.03)
        fig.suptitle(f"Recall@5 within ±{tolerance:g}°: fixed time versus {accumulation_label(protocol['settings'])} accumulation")
        save_figure(plt,fig,output/"figures/time_vs_angle_recall_at_5")
    return inputs


def angular_study(source_results,accumulation_deg=.6,tolerance=5.,output_root=None,prepare_only=False,no_plots=False):
    source=Path(source_results).expanduser().resolve()
    original=json.loads((source/"protocol_snapshot.json").read_text(encoding="utf-8"))
    completion=json.loads((source/"run_complete.json").read_text(encoding="utf-8"))
    validate_protocol(original)
    if original["settings"].get("accumulation_mode","time")!="time" or original["settings"]["normalization"]!="none":
        raise ValueError("Source must be a completed raw-count, time-window protocol")
    if not np.isfinite(tolerance) or not 0<tolerance<=180:
        raise ValueError("Angular correctness tolerance must be in (0,180]")
    if not np.isfinite(accumulation_deg) or not 0<accumulation_deg<=360:
        raise ValueError("Angular accumulation must be in (0,360] degrees")
    root=resolve(source.parent,original["experiments_root"])
    output=Path(output_root).expanduser().resolve() if output_root else source.parent.with_name("VPR-SAD-angle"+f"{accumulation_deg:g}".replace(".","p")+"deg")
    if output==source.parent or source.parent in output.parents or output in source.parent.parents:
        raise ValueError("Angular study must use a separate protocol directory")
    protocol=copy.deepcopy(original)
    protocol["experiments_root"]=relative(root,output)
    protocol["settings"].update(accumulation_mode="angle",accumulation_deg=accumulation_deg)
    protocol["study_source"]=dict(original_protocol_sha256=completion["protocol_sha256"],original_run_complete_sha256=sha256(source/"run_complete.json"))
    validate_protocol(protocol)
    plans,audit=check_inputs(protocol,root,require_raw=False)
    # Freeze exactly the same scene angles/centres as the completed time-window run.
    previous={(r["acquisition"],int(r["frame_index"])):r for r in read_csv(source/"frame_windows.csv")}
    planned=[]
    for key,plan in plans.items():
        for i,center in enumerate(plan["center_us"]):
            row=previous.get((key,i))
            if row is None or abs(float(row["center_s"])*1e6-center)>.01 or not np.isclose(float(row["wrapped_angle_deg"]),plan["angle_deg"][i],rtol=0,atol=1e-8):
                raise ValueError("Angular window study changed the frozen sampling centres")
            planned.append(dict(acquisition=key,frame_index=i,center_s=float(center/1e6),start_s=float(plan["start_us"][i]/1e6),
                end_s=float(plan["end_us"][i]/1e6),duration_ms=float((plan["end_us"][i]-plan["start_us"][i])/1000),angular_span_deg=float(plan["window_angle_span_deg"][i])))
    if len(previous)!=len(planned):
        raise ValueError("Angular window study changed the number of frames")
    path=output/"protocol.json"
    if path.exists():
        if json.loads(path.read_text(encoding="utf-8"))!=protocol:
            raise ValueError("Existing angular study protocol disagrees; choose another output directory")
    elif output.exists() and any(output.iterdir()):
        raise FileExistsError("Angular study output is not empty; choose a new directory")
    else:
        output.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(protocol,indent=2),encoding="utf-8")
    write_csv(output/"planned_frame_windows.csv",planned)
    write_csv(output/"planned_input_audit.csv",[{k:v for k,v in r.items() if k!="sources"} for r in audit])
    if prepare_only:
        print(f"[PREPARED ONLY] {len(planned)} angular windows; RAW extraction and VPR comparison have not run: {output}",flush=True)
        return output
    check_inputs(protocol,root,require_raw=True)
    if not no_plots:
        plotting()
    results=output/"results"
    if (results/"run_complete.json").is_file():
        if json.loads((results/"protocol_snapshot.json").read_text(encoding="utf-8"))!=protocol:
            raise ValueError("Completed angular results disagree with the study protocol")
    else:
        from sad_vpr import run
        run(path,no_plots=True,overwrite=results.exists())
    comparisons={}
    for mode in ("none","l1"):
        temporal=ensure_comparison(source,tolerance,mode,no_plots,baseline=comparisons.get("none",(None,None))[0])
        angular=ensure_comparison(results,tolerance,mode,no_plots,baseline=comparisons.get("none",(None,None))[1])
        comparisons[mode]=(temporal,angular)
    inputs=write_window_comparison(output,comparisons,protocol,tolerance,no_plots)
    (output/"window_study_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),version=VERSION,
        accumulation_deg=accumulation_deg,tolerance_deg=tolerance,inputs=inputs,code_sha256=code_fingerprints(),runtime=runtime_versions()),indent=2),encoding="utf-8")
    print(f"[DONE] Time/angle comparison, raw and L1-normalised counts: {output}",flush=True)
    return output
