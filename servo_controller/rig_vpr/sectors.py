"""Descriptive angular-sector analysis of existing raw/normalised query results."""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from . import VERSION
from .algorithm import code_fingerprints, runtime_versions
from .data import CONDITIONS, read_csv, sha256
from .reporting import plotting, save_figure, write_csv


def sector_index(angle,width,start_deg=0.):
    if not np.isfinite(angle):
        raise ValueError("Sector angles must be finite")
    return int(np.floor(((angle-start_deg)%360)/width))


def parse_bool(value):
    if value not in ("True","False"):
        raise ValueError("Expected explicit True/False query flags")
    return value=="True"


def load_queries(folder,expected_mode):
    folder=Path(folder).expanduser().resolve()
    complete=json.loads((folder/"reference_comparison_complete.json").read_text(encoding="utf-8"))
    audit=json.loads((folder/"comparison_provenance.json").read_text(encoding="utf-8"))
    cfg=audit.get("evaluation_settings",audit["source_protocol"]["settings"])
    if cfg["normalization"]!=expected_mode or complete["tolerance_deg"]!=audit["evaluation_tolerance_deg"]:
        raise ValueError("Comparison mode/tolerance disagrees with its completion marker")
    rows=read_csv(folder/"query_metrics.csv")
    pairs=read_csv(folder/"summary_per_pair.csv")
    grouped=defaultdict(list)
    lookup={}
    for row in rows:
        key=(row["reference"],row["query"],int(row["query_index"]))
        if key in lookup or key[0]==key[1]:
            raise ValueError("Duplicate query or self-comparison")
        row=dict(row,query_index=key[2],correct_at_1=parse_bool(row["correct_at_1"]),correct_at_5=parse_bool(row["correct_at_5"]))
        lookup[key]=row; grouped[key[:2]].append(row)
    pair_keys={(r["reference"],r["query"]) for r in pairs}
    if len(pair_keys)!=len(pairs) or pair_keys!=set(grouped) or len(pairs)!=complete["ordered_pairs"]:
        raise ValueError("Query rows and completed pair summaries disagree")
    for pair in pairs:
        selected=grouped[(pair["reference"],pair["query"])]
        if len(selected)!=int(pair["query_count"]):
            raise ValueError("Query denominator disagrees with pair summary")
        if float(pair["tolerance_deg"])!=complete["tolerance_deg"] or sum(r["status"]=="empty_query_descriptor" for r in selected)!=int(pair["empty_queries"]):
            raise ValueError("Query empty counts or pair tolerance disagree")
        for k,count_field in ((1,"correct_queries"),(5,"correct_queries_at_5")):
            count=sum(r[f"correct_at_{k}"] for r in selected)
            if count!=int(pair[count_field]) or not np.isclose(count/len(selected),float(pair[f"recall_at_{k}"]),rtol=0,atol=1e-12):
                raise ValueError("Query success flags do not reproduce the existing pair metrics")
    return dict(path=folder,audit=audit,complete=complete,cfg=cfg,rows=lookup,
        hashes={name:sha256(folder/name) for name in ("reference_comparison_complete.json","comparison_provenance.json","summary_per_pair.csv","query_metrics.csv")})


def aggregate_sectors(raw,normalized,windows,width,start_deg=0.):
    protocol=raw["audit"]["source_protocol"]
    entries={e["key"]:e for e in protocol["acquisitions"]}
    buckets=defaultdict(list)
    assignment=[]
    if set(raw["rows"])!=set(normalized["rows"]):
        raise ValueError("Raw and normalised comparisons use different queries")
    for key,row in raw["rows"].items():
        other=normalized["rows"][key]
        if row["query_wrapped_angle_deg"]!=other["query_wrapped_angle_deg"]:
            raise ValueError("Raw and normalised query angles changed")
        ref,qry,index=key
        frame=windows[(qry,index)]
        if not np.isclose(float(row["query_wrapped_angle_deg"]),float(frame["wrapped_angle_deg"]),rtol=0,atol=1e-8):
            raise ValueError("Query angle disagrees with frozen frame audit")
        angle=float(frame["requested_wrapped_angle_deg"])%360
        sector=sector_index(angle,width,start_deg)
        assignment.append(dict(reference=ref,query=qry,query_index=index,requested_angle_deg=angle,
            measured_angle_deg=float(row["query_wrapped_angle_deg"]),sector_start_deg=start_deg+sector*width,
            sector_end_deg=start_deg+(sector+1)*width,raw_correct_at_5=row["correct_at_5"],l1_correct_at_5=other["correct_at_5"]))
        for mode,query in (("none",row),("l1",other)):
            buckets[(mode,ref,qry,sector)].append(query)
    pair_rows=[]
    for (mode,ref,qry,sector),selected in sorted(buckets.items()):
        n=len(selected)
        pair_rows.append(dict(normalization=mode,reference=ref,query=qry,reference_condition=entries[ref]["condition"],
            query_condition=entries[qry]["condition"],sector_start_deg=start_deg+sector*width,sector_end_deg=start_deg+(sector+1)*width,
            query_count=n,correct_at_1=sum(r["correct_at_1"] for r in selected),correct_at_5=sum(r["correct_at_5"] for r in selected),
            recall_at_1=float(np.mean([r["correct_at_1"] for r in selected])),recall_at_5=float(np.mean([r["correct_at_5"] for r in selected])),
            empty_queries=sum(r["status"]=="empty_query_descriptor" for r in selected)))
    def means(rows,fields):
        groups=defaultdict(list)
        for row in rows:
            groups[tuple(row[k] for k in fields)].append(row)
        result=[]
        for key,selected in sorted(groups.items()):
            item=dict(zip(fields,key))
            item.update(query_count=sum(r["query_count"] for r in selected),empty_queries=sum(r["empty_queries"] for r in selected))
            for k in (1,5):
                values=np.array([r[f"recall_at_{k}"] for r in selected])
                item.update({f"recall_at_{k}":float(values.mean()),f"minimum_recall_at_{k}":float(values.min()),f"maximum_recall_at_{k}":float(values.max()),
                    f"correct_at_{k}":sum(r[f"correct_at_{k}"] for r in selected)})
            item["contributing_acquisitions" if "reference" in fields else "reference_acquisitions"]=len(selected)
            result.append(item)
        return result
    reference_rows=means(pair_rows,("normalization","reference","reference_condition","query_condition","sector_start_deg","sector_end_deg"))
    speed_rows=means(reference_rows,("normalization","reference_condition","query_condition","sector_start_deg","sector_end_deg"))
    return pair_rows,reference_rows,speed_rows,assignment


def sector_figures(output,reference_rows,speed_rows,entries,width,tolerance,start_deg=0.):
    plt=plotting(); sectors=start_deg+np.arange(round(360/width))*width; centres=sectors+width/2
    labels=[f"{x:g}-{x+width:g}" for x in sectors]
    speed_lookup={(r["normalization"],r["reference_condition"],r["query_condition"],r["sector_start_deg"]):r for r in speed_rows}
    ref_lookup={(r["normalization"],r["reference"],r["query_condition"],r["sector_start_deg"]):r for r in reference_rows}
    pdfs=[]
    for condition,(rpm,_) in CONDITIONS.items():
        prefix=condition.replace("-Speed","").lower()+"_speed"
        fig,axes=plt.subplots(1,3,figsize=(12.5,4.6),sharey=True)
        for ax,(reference_condition,(ref_rpm,_)) in zip(axes,CONDITIONS.items()):
            for mode,color,marker,line,label in (("none","#0072B2","o","-","Raw counts"),("l1","#D55E00","s","--","L1-normalised")):
                rows=[speed_lookup[(mode,reference_condition,condition,float(s))] for s in sectors]
                mean=100*np.array([r["recall_at_5"] for r in rows]);lo=100*np.array([r["minimum_recall_at_5"] for r in rows]);hi=100*np.array([r["maximum_recall_at_5"] for r in rows])
                ax.fill_between(centres,lo,hi,color=color,alpha=.13,linewidth=0)
                ax.plot(centres,mean,color=color,marker=marker,linestyle=line,markersize=4,linewidth=1.6,label=label)
            ax.set(title=f"Reference: {ref_rpm:g} rpm",xlim=(start_deg,start_deg+360),ylim=(-3,103),xlabel="Query angle sector (degrees)")
            ax.set_xticks(sectors+width/2,labels,rotation=55,ha="right",fontsize=8)
            ax.set_yticks([0,25,50,75,100]);ax.grid(axis="y",alpha=.22)
        axes[0].set_ylabel(f"Recall@5 (%) within ±{tolerance:g}°")
        fig.suptitle(f"{condition.replace('-',' ')} queries ({rpm:g} rpm): {width:g}° sectors",y=.99,fontsize=14)
        fig.legend(*axes[0].get_legend_handles_labels(),loc="upper center",bbox_to_anchor=(.5,.92),ncol=2,frameon=False)
        fig.text(.5,.015,"Lines: mean over reference runs. Bands: min-max of reference-run means; descriptive, not confidence intervals.",ha="center",fontsize=8)
        fig.subplots_adjust(top=.76,bottom=.27,left=.07,right=.99,wspace=.12)
        stem=output/(prefix+f"_{width:g}deg_sectors")
        save_figure(plt,fig,stem);pdfs.append(str(stem.with_suffix('.pdf')))
        fig,axes=plt.subplots(2,1,figsize=(11.8,9.6))
        keys=[e["key"] for e in entries]
        for ax,mode,title in zip(axes,("none","l1"),("Raw counts","L1-normalised counts")):
            values=np.array([[100*ref_lookup[(mode,key,condition,float(s))]["recall_at_5"] for s in sectors] for key in keys])
            im=ax.imshow(values,cmap="viridis",vmin=0,vmax=100,aspect="auto")
            for i in range(len(keys)):
                for j in range(len(sectors)):
                    v=values[i,j];ax.text(j,i,f"{v:.1f}",ha="center",va="center",fontsize=7.5,color="black" if v>=60 else "white")
            ax.set_xticks(range(len(sectors)),labels,rotation=35,ha="right",fontsize=8)
            ax.set_yticks(range(len(keys)),[k.replace('-Speed/',' / ') for k in keys],fontsize=9)
            ax.set(title=title,xlabel="Query angle sector (degrees)",ylabel="Reference acquisition")
            fig.colorbar(im,ax=ax,fraction=.025,pad=.02,label="Recall@5 (%)")
        fig.suptitle(f"{condition.replace('-',' ')} queries ({rpm:g} rpm): reference-run detail, Recall@5 within ±{tolerance:g}°",y=.99,fontsize=13)
        fig.text(.5,.015,"Cells average query runs: two at the reference's speed, three otherwise. Empty queries remain failures.",ha="center",fontsize=8)
        fig.subplots_adjust(left=.16,right=.96,top=.94,bottom=.10,hspace=.48)
        stem=output/(prefix+"_reference_runs_heatmap")
        save_figure(plt,fig,stem);pdfs.append(str(stem.with_suffix('.pdf')))
    return pdfs


def analyse_sectors(raw_results,normalized_results,sector_deg=30.,output_root=None,no_plots=False,sector_start_deg=0.):
    if not np.isfinite(sector_deg) or not 0<sector_deg<=180 or not np.isclose(360/sector_deg,round(360/sector_deg),rtol=0,atol=1e-8):
        raise ValueError("Sector width must divide 360 evenly and be in (0,180]")
    if not np.isfinite(sector_start_deg) or not -360 < sector_start_deg < 360:
        raise ValueError("Sector start must be finite and between -360 and 360 degrees")
    raw=load_queries(raw_results,"none");normalized=load_queries(normalized_results,"l1")
    if raw["audit"]["source_protocol"]!=normalized["audit"]["source_protocol"] or raw["complete"]["tolerance_deg"]!=normalized["complete"]["tolerance_deg"]:
        raise ValueError("Raw and normalised source protocols/tolerances differ")
    for key,value in raw["audit"]["cached_inputs"].items():
        if value["sha256"]!=normalized["audit"]["cached_inputs"][key]["sha256"]:
            raise ValueError("Raw and normalised frame caches differ")
    source=Path(raw["audit"]["source_results"])
    if not (source/"frame_windows.csv").is_file():
        source=raw["path"].parent/"results"
    windows=read_csv(source/"frame_windows.csv")
    window_hash=sha256(source/"frame_windows.csv")
    if window_hash!=raw["audit"]["source_file_sha256"]["frame_windows.csv"] or window_hash!=normalized["audit"]["source_file_sha256"]["frame_windows.csv"]:
        raise ValueError("Source frame-window audit changed")
    window_lookup={(r["acquisition"],int(r["frame_index"])):r for r in windows}
    if len(window_lookup)!=len(windows):
        raise ValueError("Duplicate frame-window audit rows")
    pairs,references,speeds,assignment=aggregate_sectors(raw,normalized,window_lookup,sector_deg,sector_start_deg)
    suffix="" if sector_start_deg==0 else f"_start{sector_start_deg:g}deg"
    output=Path(output_root).expanduser().resolve() if output_root else raw["path"].parent/(f"angle_sectors_{sector_deg:g}deg"+suffix)
    for folder in (raw["path"],normalized["path"],source):
        if output==folder or output in folder.parents or folder in output.parents:
            raise ValueError("Sector output must be separate from source results")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Sector output is not empty; choose a new directory")
    if not no_plots:
        plotting()
    output.mkdir(parents=True,exist_ok=True)
    write_csv(output/"sectors_per_pair.csv",pairs)
    write_csv(output/"sectors_by_reference.csv",references)
    write_csv(output/"sectors_by_reference_speed.csv",speeds)
    write_csv(output/"sector_query_assignments.csv",assignment)
    old={(r["reference_condition"],r["query_condition"],r["sector_start_deg"]):r for r in speeds if r["normalization"]=="none"}
    delta=[]
    for r in speeds:
        if r["normalization"]=="l1":
            before=old[(r["reference_condition"],r["query_condition"],r["sector_start_deg"])]
            delta.append(dict(reference_condition=r["reference_condition"],query_condition=r["query_condition"],sector_start_deg=r["sector_start_deg"],sector_end_deg=r["sector_end_deg"],
                raw_recall_at_5=before["recall_at_5"],l1_recall_at_5=r["recall_at_5"],change_pp=100*(r["recall_at_5"]-before["recall_at_5"])))
    write_csv(output/"sector_normalization_comparison.csv",delta)
    tolerance=raw["complete"]["tolerance_deg"]
    pdfs=sector_figures(output,references,speeds,raw["audit"]["source_protocol"]["acquisitions"],sector_deg,tolerance,sector_start_deg) if not no_plots else []
    notes=["# Recognition by angular sector","",f"Twelve {sector_deg:g}° sectors" if sector_deg==30 else f"{round(360/sector_deg)} sectors of {sector_deg:g}°",f"Recall@5 within ±{tolerance:g}°, with Recall@1 also retained in CSVs.","",
        "Sectors are half-open [start,end), with 360° wrapping to 0°. Sector assignment uses the frozen requested sampling angle, avoiding numerical timestamp-rounding jitter around sector boundaries. The success flags are unchanged from the original evaluator, which uses interpolated encoder angles and the existing tie policy.","",
        f"The first sector starts at {sector_start_deg:g}°. Bounds are displayed without wrapping; negative angles denote equivalent orientations modulo 360°. For example [-15,15) contains [345,360) and [0,15).", "",
        "Per-pair sector recall includes every requested query, including empty failures. Reference-run means give equal weight to query acquisitions. Reference-speed means give equal weight to the three reference-run means. Min-max bands describe variation between those means, not confidence intervals. Shared recordings and overlapping windows are not independent trials.","",
        "The source frames, accumulation settings and selected revolutions are unchanged. This is a descriptive analysis of where recognition succeeds or fails, not evidence establishing the cause of an angular pattern. No sectors are excluded from the overall result.",""]
    (output/"README.md").write_text("\n".join(notes),encoding="utf-8")
    audit=dict(version=VERSION,sector_deg=sector_deg,sector_start_deg=sector_start_deg,tolerance_deg=tolerance,source_protocol=raw["audit"]["source_protocol"],
        sources={"raw":dict(path=str(raw["path"]),sha256=raw["hashes"]),"l1":dict(path=str(normalized["path"]),sha256=normalized["hashes"])},
        frame_windows_path=str(source/"frame_windows.csv"),frame_windows_sha256=window_hash,code_sha256=code_fingerprints(),runtime=runtime_versions(),
        paired_query_evaluations=len(assignment),sector_pair_rows=len(pairs),sector_reference_rows=len(references),sector_speed_rows=len(speeds),pdfs=pdfs)
    (output/"sector_provenance.json").write_text(json.dumps(audit,indent=2),encoding="utf-8")
    (output/"sector_analysis_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),sector_deg=sector_deg,sector_start_deg=sector_start_deg,tolerance_deg=tolerance,pdfs=len(pdfs)),indent=2),encoding="utf-8")
    print(f"[DONE] Angular-sector analysis: {output}",flush=True)
    return output
