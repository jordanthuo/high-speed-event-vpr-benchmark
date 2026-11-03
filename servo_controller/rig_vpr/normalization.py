"""Paired raw-count versus normalised-count comparisons on identical frames."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .data import CONDITIONS, accumulation_label, read_csv, sha256
from .reporting import plotting, save_figure, write_csv


def load_baseline_comparison(path,protocol,tolerance,cache_audit):
    path=Path(path).expanduser().resolve()
    complete=json.loads((path/"reference_comparison_complete.json").read_text(encoding="utf-8"))
    audit=json.loads((path/"comparison_provenance.json").read_text(encoding="utf-8"))
    cfg=audit.get("evaluation_settings",audit["source_protocol"]["settings"])
    if audit["source_protocol"]!=protocol or complete["tolerance_deg"]!=tolerance or audit["evaluation_tolerance_deg"]!=tolerance or cfg["normalization"]!="none":
        raise ValueError("Baseline must use the same source protocol/tolerance and raw counts")
    if any(audit["cached_inputs"][key]["sha256"]!=value["sha256"] for key,value in cache_audit.items()):
        raise ValueError("Baseline comparison uses different cached frames")
    pairs=read_csv(path/"summary_per_pair.csv")
    groups=read_csv(path/"summary_by_reference_speed.csv")
    keys={entry["key"] for entry in protocol["acquisitions"]}
    expected={(a,b) for a in keys for b in keys if a!=b}
    if len(pairs)!=len(expected) or {(r["reference"],r["query"]) for r in pairs}!=expected or len(groups)!=len(CONDITIONS)**2:
        raise ValueError("Baseline has incomplete or duplicate comparisons")
    if {(r["reference_condition"],r["query_condition"]) for r in groups}!={(a,b) for a in CONDITIONS for b in CONDITIONS}:
        raise ValueError("Baseline has incomplete speed groups")
    # Reconcile reported group means against their individual directed pairs.
    for group in groups:
        selected=[r for r in pairs if r["reference_condition"]==group["reference_condition"] and r["query_condition"]==group["query_condition"]]
        for k in (1,5):
            if not np.isclose(np.mean([float(r[f"recall_at_{k}"]) for r in selected]),float(group[f"mean_recall_at_{k}"]),rtol=0,atol=1e-12):
                raise ValueError("Baseline group means disagree with pair results")
    return dict(path=path,pairs=pairs,groups=groups,sha256={name:sha256(path/name) for name in
        ("reference_comparison_complete.json","comparison_provenance.json","summary_per_pair.csv","summary_by_reference_speed.csv")})


def write_normalization_comparison(output,baseline,pairs,groups,cfg,tolerance,no_plots):
    if cfg["normalization"]!="l1":
        raise ValueError("Paired normalization comparison requires l1 descriptors")
    old_pairs={(r["reference"],r["query"]):r for r in baseline["pairs"]}
    old_groups={(r["reference_condition"],r["query_condition"]):r for r in baseline["groups"]}
    paired=[]
    for row in pairs:
        old=old_pairs[(row["reference"],row["query"])]
        item={name:row[name] for name in ("reference","query","reference_condition","query_condition","tolerance_deg","query_count")}
        if int(old["query_count"])!=row["query_count"] or int(old["empty_queries"])!=row["empty_queries"]:
            raise ValueError("Normalization comparison changed query sampling or empty-query handling")
        for k in (1,5):
            raw=float(old[f"recall_at_{k}"])
            normalized=row[f"recall_at_{k}"]
            item.update({f"raw_recall_at_{k}":raw,f"l1_recall_at_{k}":normalized,f"change_pp_at_{k}":100*(normalized-raw)})
        paired.append(item)
    paired_groups=[]
    for row in groups:
        old=old_groups[(row["reference_condition"],row["query_condition"])]
        item={name:row[name] for name in ("reference_condition","query_condition","ordered_pairs")}
        for k in (1,5):
            raw=float(old[f"mean_recall_at_{k}"])
            normalized=row[f"mean_recall_at_{k}"]
            item.update({f"raw_mean_recall_at_{k}":raw,f"l1_mean_recall_at_{k}":normalized,f"change_pp_at_{k}":100*(normalized-raw)})
        paired_groups.append(item)
    write_csv(output/"normalization_comparison_per_pair.csv",paired)
    write_csv(output/"normalization_comparison_by_speed.csv",paired_groups)
    lines=["# Raw versus L1-normalised event counts","",
        f"Recall@5 at ±{tolerance:g}°. Identical count frames, selected revolutions, angular grid, alignment, spatial bins and query denominators; only per-frame count normalization changes.","",
        "For a nonempty frame with counts c_i, the normalised descriptor is c_i / sum_j(c_j). SAD is then applied to these unit-sum descriptors. Empty query frames remain failures and empty reference frames remain excluded. No encoder angles are used in normalization or ranking.","",
        "| Reference speed | Query speed | Raw Recall@5 | L1 Recall@5 | Change (percentage points) |",
        "|---|---|---:|---:|---:|"]
    for r in paired_groups:
        lines.append(f"| {r['reference_condition']} | {r['query_condition']} | {100*r['raw_mean_recall_at_5']:.2f}% | {100*r['l1_mean_recall_at_5']:.2f}% | {r['change_pp_at_5']:+.2f} |")
    cross=[r for r in paired if r["reference_condition"]!=r["query_condition"]]
    within=[r for r in paired if r["reference_condition"]==r["query_condition"]]
    means={name:{"raw_recall_at_5":float(np.mean([r["raw_recall_at_5"] for r in selected])),
                 "l1_recall_at_5":float(np.mean([r["l1_recall_at_5"] for r in selected])),"ordered_pairs":len(selected)}
           for name,selected in (("cross_speed",cross),("same_speed",within))}
    lines += ["", "Each speed cell averages the three reference-run means. Same-speed cells contain six directed pairs and cross-speed cells nine. The 72 comparisons reuse nine recordings; they are descriptive comparisons, not 72 independent experiments.","",
        "Improvement after normalization supports a contribution from overall count magnitude to raw-SAD errors. It does not isolate a physical cause: normalization also changes feature weighting. Spatial event distributions can differ between speeds even after total count is removed. The accumulation mode is recorded in the source protocol.",""]
    (output/"normalization_comparison.md").write_text("\n".join(lines),encoding="utf-8")
    if not no_plots:
        plt=plotting()
        conditions=list(CONDITIONS)
        lookup={(r["reference_condition"],r["query_condition"]):r for r in paired_groups}
        raw=np.array([[100*lookup[(a,b)]["raw_mean_recall_at_5"] for b in conditions] for a in conditions])
        normalized=np.array([[100*lookup[(a,b)]["l1_mean_recall_at_5"] for b in conditions] for a in conditions])
        fig,axes=plt.subplots(1,3,figsize=(13.8,4.8),layout="constrained")
        for ax,array,title,delta in zip(axes,(raw,normalized,normalized-raw),("Raw counts","L1-normalised counts","Change"),(False,False,True)):
            im=ax.imshow(array,cmap="RdBu" if delta else "viridis",vmin=-100 if delta else 0,vmax=100)
            for i in range(3):
                for j in range(3):
                    value=array[i,j]
                    color=("black" if abs(value)<55 else "white") if delta else ("black" if value>=60 else "white")
                    ax.text(j,i,f"{value:+.1f}" if delta else f"{value:.1f}%",ha="center",va="center",color=color,fontsize=11)
            ax.set_xticks(range(3),[f"{CONDITIONS[c][0]:g}" for c in conditions])
            ax.set_yticks(range(3),[f"{CONDITIONS[c][0]:g}" for c in conditions])
            ax.set(xlabel="Query speed (rpm)",ylabel="Reference speed (rpm)",title=title)
            fig.colorbar(im,ax=ax,orientation="horizontal",fraction=.07,pad=.18,label="Percentage points" if delta else "Recall@5 (%)")
        fig.suptitle(f"Recall@5 within ±{tolerance:g}° — identical {accumulation_label(cfg)} count frames")
        save_figure(plt,fig,output/"figures/raw_vs_l1_recall_at_5")
    return dict(baseline_comparison=str(baseline["path"]),baseline_file_sha256=baseline["sha256"],paired_ordered_comparisons=len(paired),means=means)
