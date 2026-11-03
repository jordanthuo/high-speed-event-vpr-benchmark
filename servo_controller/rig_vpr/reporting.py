"""Acquisition-level summaries and optional PDF/PNG scientific figures."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .data import CONDITIONS, accumulation_label

COLORS = ("#0072B2", "#D55E00", "#AA3377")


def write_csv(path, rows):
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        fields = list(dict.fromkeys(k for row in rows for k in row))
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k:"" if v is None else v for k, v in row.items()} for row in rows)


def speed_summary(rows):
    results = []
    for reference in dict.fromkeys(row["reference"] for row in rows):
        for condition in CONDITIONS:
            subset = [row for row in rows if row["reference"] == reference and row["query_condition"] == condition]
            if not subset:
                continue
            recalls = np.array([row["recall_at_1"] for row in subset])
            recalls5 = np.array([row["recall_at_5"] for row in subset])
            results.append(dict(reference=reference, query_condition=condition,
                nominal_platform_rpm=CONDITIONS[condition][0], independent_query_acquisitions=len(subset),
                mean_recall_at_1=float(recalls.mean()),
                between_acquisition_sample_sd_recall=float(recalls.std(ddof=1)) if len(recalls)>1 else None,
                total_queries=sum(row["query_count"] for row in subset),
                total_correct=sum(row["correct_queries"] for row in subset),
                mean_recall_at_5=float(recalls5.mean()),
                between_acquisition_sample_sd_recall_at_5=float(recalls5.std(ddof=1)) if len(recalls5)>1 else None,
                total_correct_at_5=sum(row["correct_queries_at_5"] for row in subset),
                mean_match_coverage=float(np.mean([row["match_coverage"] for row in subset]))))
    return results


def top5_query_fields(result, index):
    credited_errors = abs(result["top_errors_deg"][:, index][result["top_credited"][:, index]])
    return dict(correct_at_5=bool(result["correct_at_5"][index]),
        top5_effective_k=int(result["top5_effective_k"]),
        top5_credited_candidates=int(result["top_credited"][:, index].sum()),
        top5_boundary_tie=bool(result["top5_boundary_tie"][index]),
        top5_min_credited_angle_error_deg=float(credited_errors.min()) if len(credited_errors) else None)


def candidate_rows(reference_key, query_key, result, tolerance):
    rows = []
    for i in range(len(result["query_angles_deg"])):
        for rank, ref_index in enumerate(result["top_indices"][:, i]):
            within = abs(result["top_errors_deg"][rank, i]) <= tolerance + 1e-8
            credited = bool(result["top_credited"][rank, i])
            rows.append(dict(reference=reference_key, query=query_key, query_index=i, rank=rank+1,
                reference_index=int(ref_index), sad=float(result["top_scores"][rank, i]),
                query_wrapped_angle_deg=float(result["query_angles_deg"][i]),
                reference_wrapped_angle_deg=float(result["reference_angles_deg"][ref_index]),
                signed_error_deg=float(result["top_errors_deg"][rank, i]), tolerance_deg=tolerance,
                angle_within_tolerance=bool(within), query_nonempty=bool(result["query_valid"][i]),
                credited_candidate=credited, credited_correct=bool(credited and within),
                top5_boundary_tie=bool(result["top5_boundary_tie"][i])))
    return rows


def match_rows(reference_entry, query_entry, reference, query, result):
    output = []
    for i, best in enumerate(result["best"]):
        accepted = bool(result["accepted"][i])
        output.append(dict(reference=reference_entry["key"], query=query_entry["key"], query_index=i,
            query_camera_center_s=float(query["center_us"][i]/1e6),
            query_window_start_s=float(query["start_us"][i]/1e6), query_window_end_s=float(query["end_us"][i]/1e6),
            query_wrapped_angle_deg=float(query["angle_deg"][i]),
            query_window_angle_span_deg=float(query["window_angle_span_deg"][i]),
            query_platform_rpm=float(query["speed_rpm"][i]), query_event_count=int(query["event_counts"][i]),
            candidate_reference_index=int(best), candidate_reference_angle_deg=float(reference["angle_deg"][best]),
            candidate_reference_camera_s=float(reference["center_us"][best]/1e6),
            predicted_wrapped_angle_deg=float(reference["angle_deg"][best]) if accepted else None,
            signed_error_deg=float(result["error_deg"][i]) if accepted else None,
            absolute_error_deg=float(abs(result["error_deg"][i])) if accepted else None,
            best_sad=float(result["best_distance"][i]),
            second_best_sad=float(result["second_distance"][i]) if np.isfinite(result["second_distance"][i]) else None,
            best_score_tie_count=int(result["ties"][i]), status=str(result["status"][i]),
            correct=bool(result["correct"][i]), nearest_reference_error_deg=float(result["nearest_reference_error_deg"][i]),
            **top5_query_fields(result, i)))
    return output


def plotting():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Plots require matplotlib. Install requirements-vpr.txt or use --no-plots.") from exc
    plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":11,
        "axes.spines.top":False, "axes.spines.right":False, "pdf.fonttype":42, "ps.fonttype":42})
    return plt


def save_figure(plt, fig, stem):
    stem = Path(stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(stem)+".pdf", bbox_inches="tight")
    fig.savefig(str(stem)+".png", bbox_inches="tight", dpi=180)
    plt.close(fig)


def pair_figures(folder, reference, query, result, reference_key, query_key, tolerance):
    plt = plotting()
    folder = Path(folder)
    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    data = np.ma.masked_invalid(result["scores"])
    image = ax.imshow(data, origin="lower", aspect="auto", cmap="viridis")
    ax.plot(np.arange(len(query["angle_deg"]))[result["accepted"]], result["best"][result["accepted"]],
            ".", color="#FFFFFF", markersize=2, label="Selected match")
    ax.set(xlabel="Query sample index", ylabel="Reference sample index", title=f"{query_key} against {reference_key}")
    fig.colorbar(image, ax=ax, label="SAD")
    save_figure(plt, fig, folder/"distance_matrix")
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    accepted = result["accepted"]
    ax.plot(query["angle_deg"][accepted], result["error_deg"][accepted], ".", color=COLORS[0], markersize=3)
    ax.axhline(tolerance, color=COLORS[1], linestyle="--", label=f"±{tolerance:g}° correctness tolerance")
    ax.axhline(-tolerance, color=COLORS[1], linestyle="--")
    ax.set(xlabel="Query wrapped encoder angle (degrees)", ylabel="Signed circular error (degrees)",
           title=f"{query_key}: accepted matches only")
    ax.legend(fontsize=9)
    save_figure(plt, fig, folder/"angle_errors")
    # Deterministic examples: first success, first accepted failure, first abstention.
    examples = []
    for name, mask in (("first_correct", result["correct"]),
                       ("first_incorrect", accepted & ~result["correct"]),
                       ("first_abstention", ~accepted)):
        indices = np.flatnonzero(mask)
        if not len(indices):
            continue
        i = int(indices[0]); best = int(result["best"][i])
        # Nearest-angle view is displayed for review only, after algorithm prediction.
        from .algorithm import circular_error
        ground_truth = int(np.argmin(abs(circular_error(reference["angle_deg"], query["angle_deg"][i]))))
        frames = (query["frames"][i], reference["frames"][best], reference["frames"][ground_truth])
        titles = (f"Query {query['angle_deg'][i]:.2f}°", f"SAD candidate {reference['angle_deg'][best]:.2f}°",
                  f"Nearest-angle reference {reference['angle_deg'][ground_truth]:.2f}°")
        fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.3))
        maximum = max(float(frame.sum(axis=0).max()) for frame in frames)
        for ax, frame, title in zip(axes, frames, titles):
            ax.imshow(frame.sum(axis=0), cmap="gray", vmin=0, vmax=maximum or 1, interpolation="nearest")
            ax.set_title(title, fontsize=10); ax.axis("off")
        fig.suptitle(f"{query_key}, sample {i}: {result['status'][i]}" +
                     (f", error {result['error_deg'][i]:.2f}°" if accepted[i] else " (no accepted prediction)"), fontsize=11)
        save_figure(plt, fig, folder/name)
        examples.append(dict(example=name, query_index=i, candidate_reference_index=best,
                             nearest_angle_reference_index=ground_truth))
    write_csv(folder/"examples.csv", examples)


def summary_figure(folder, pair_rows, summaries, tolerance, k=1):
    plt = plotting()
    for reference in dict.fromkeys(row["reference"] for row in pair_rows):
        fig, ax = plt.subplots(figsize=(7.0, 4.2))
        for x, condition in enumerate(CONDITIONS):
            subset = [r for r in pair_rows if r["reference"] == reference and r["query_condition"] == condition]
            if not subset:
                continue
            values = [100*r[f"recall_at_{k}"] for r in subset]
            offsets = np.linspace(-.10,.10,len(subset)) if len(subset)>1 else [0]
            for offset, row, value in zip(offsets, subset, values):
                color = COLORS[int(row["query_run"].replace("run", ""))-1]
                ax.plot(x+offset, value, "o", color=color, markersize=6)
            ax.errorbar(x, np.mean(values), yerr=np.std(values, ddof=1) if len(values)>1 else 0,
                        color="black", fmt="_", markersize=16, capsize=4)
        labels = []
        for condition, (speed, _) in CONDITIONS.items():
            n = sum(r["reference"] == reference and r["query_condition"] == condition for r in pair_rows)
            labels.append(f"{speed:g} rpm\nn={n} query runs")
        ax.set_xticks(range(3), labels)
        ax.set(ylim=(-3,103), ylabel=f"Recall@{k} (%) within {tolerance:g}°", title=f"Reference: {reference}")
        ax.grid(axis="y", alpha=.25)
        from matplotlib.lines import Line2D
        handles=[Line2D([],[],color=c,marker="o",linestyle="none",label=f"Run {i+1}") for i,c in enumerate(COLORS)]
        handles.append(Line2D([],[],color="black",marker="_",linestyle="none",label="Mean and sample SD"))
        fig.subplots_adjust(top=.79,bottom=.20)
        fig.legend(handles=handles,loc="upper center",bbox_to_anchor=(.5,1.0),ncol=4,frameon=False,fontsize=9)
        prefix = "recall_" if k == 1 else f"recall_at_{k}_"
        save_figure(plt, fig, Path(folder)/(prefix+reference.replace("/", "_")))


def report(path, protocol, pair_rows, summaries):
    cfg = protocol["settings"]
    lines = ["# Rotary-rig SAD baseline", "", "Protocol status: **"+protocol["protocol_status"]+"**.", "",
        f"Reference policy: `{protocol['pair_mode']}`. Fixed reference: `{protocol['reference']}`.",
        f"Representation: {accumulation_label(cfg)} centred count windows; {cfg['width']} × {cfg['height']} spatial bins; polarity {cfg['polarity']}; normalization {cfg['normalization']}.",
        f"Sampling: {cfg['angle_step_deg']:g}° across a full turn, starting at wrapped {cfg['start_wrapped_deg']:g}°. Primary correctness tolerance: {cfg['tolerance_deg']:g}°.",
        f"Pixels: {'all spatial bins' if cfg['pixels']==0 else str(cfg['pixels'])+' seeded fixed random spatial bins'}, seed {cfg['seed']}.", "",
        "| Reference | Query speed | Query acquisitions | Mean Recall@1 | SD (pp) | Mean Recall@5 | SD (pp) |", "|---|---:|---:|---:|---:|---:|---:|"]
    for row in summaries:
        sd = row["between_acquisition_sample_sd_recall"]
        sd5 = row["between_acquisition_sample_sd_recall_at_5"]
        lines.append(f"| {row['reference']} | {row['nominal_platform_rpm']:g} rpm | {row['independent_query_acquisitions']} | {100*row['mean_recall_at_1']:.2f}% | {'n/a' if sd is None else f'{100*sd:.2f}'} | {100*row['mean_recall_at_5']:.2f}% | {'n/a' if sd5 is None else f'{100*sd5:.2f}'} |")
    lines += ["", "## Interpretation", "",
        "- Recall@1 and Recall@5 use the same angular tolerance and all requested queries. Empty query descriptors are failures for both. Recall@5 asks whether a correct reference appears among the five lowest SAD scores, not whether error is within five degrees.",
        "- Recall@1 retains the original unique-best-score policy. Recall@5 credits a correct candidate only if its whole score-tie group fits inside the top five. A tie spilling over rank five is not credited, but strictly higher candidates still count. Ties wholly inside the top five can count even when Recall@1 abstains. Stable reference-index order is for display only; angles never break ties.",
        "- Empty reference descriptors are excluded globally. The matcher evaluates every remaining reference candidate without an angle constraint.",
        "- Error summaries labelled `matched` describe accepted predictions only. Read them alongside coverage and Recall@1.",
        "- Condition means give equal weight to query acquisitions. SD is between-query-acquisition sample SD. The shared fixed reference is not an independent replicate; reference-acquisition uncertainty is not measured by this design.",
        ("- Low-speed run 1 is the reference, so the fixed-reference design has two low-speed queries and three each at medium and high speed."
         if protocol["pair_mode"] == "fixed-reference" else
         "- All ordered independent acquisition pairs are evaluated. Summaries remain conditional on each reference; reused acquisitions do not create independent pair-level replicates."),
        "- Tolerance sensitivity is descriptive. Do not select the report's correctness tolerance retrospectively from the best score.",
        "- One-degree sampling does not establish one-degree angular accuracy. Angles are interpolated between encoder samples and rely on the accepted camera alignment and nominal transmission ratio.",
        "- Numeric descriptors never include rendered timestamp, angle or speed overlays. Angle-based sampling is dataset construction; encoder truth is not a search restriction.",
        "- These sampled windows are not continuous full-revolution event totals. Windows may overlap or leave gaps depending on speed and sampling step.",
        "- This is an adaptation of the event-count/SAD baseline in Tobias Fischer and Michael Milford's sparse-event-VPR code and tutorial. It does not reproduce the full published sparse/varying-pixel and sequence-matching evaluation.", "",
        "Sources: https://github.com/Tobias-Fischer/sparse-event-vpr ; https://doi.org/10.1109/LRA.2022.3216226", ""]
    if protocol.get("discovery_warnings"):
        lines += ["## Alignment/discovery notes", "", *["- "+s for s in protocol["discovery_warnings"]], ""]
    Path(path).write_text("\n".join(lines), encoding="utf-8")
