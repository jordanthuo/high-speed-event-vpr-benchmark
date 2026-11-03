#!/usr/bin/env python3
"""Stage a fresh, paired 1 ms / 2 ms VPR evaluation on prepared acquisitions."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent


def execute(*args):
    command = [sys.executable, str(HERE / "sad_vpr.py"), *map(str, args)]
    print("[STEP] " + " ".join(command), flush=True)
    subprocess.run(command, check=True, env={**os.environ, "MPLBACKEND": "Agg"})


def validate_pair(root):
    import numpy as np
    from rig_vpr.data import check_inputs, load_protocol
    protocols, plans = [], []
    for ms in (1, 2):
        path = root / f"{ms}ms/protocol.json"
        protocol, experiments = load_protocol(path)
        if protocol.get("discovery_warnings"):
            raise ValueError("Resolve alignment warnings and create a new study: " +
                             "; ".join(protocol["discovery_warnings"]))
        windows, _ = check_inputs(protocol, experiments, require_raw=True)
        plans.append(windows)
        if protocol["settings"]["accumulation_ms"] != ms:
            raise ValueError(f"Unexpected accumulation time in {path}")
        protocols.append(protocol)
    left, right = protocols
    if left["acquisitions"] != right["acquisitions"] or left["frozen_input_sha256"] != right["frozen_input_sha256"]:
        raise ValueError("The two protocols must use identical acquisitions and source hashes")
    a, b = dict(left["settings"]), dict(right["settings"])
    a.pop("accumulation_ms"); b.pop("accumulation_ms")
    expected = dict(angle_step_deg=1., tolerance_deg=5., start_wrapped_deg=4.,
                    width=160, height=90, normalization="none", polarity="sum",
                    pixels=0, seed=0, steady_tolerance_percent=5.)
    if a != b or any(a.get(k) != v for k, v in expected.items()):
        raise ValueError("Unexpected settings or unmatched protocols")
    if any(p["pair_mode"] != "fixed-reference" or p["reference"] != "Low-Speed/run1"
           for p in protocols):
        raise ValueError("Expected the Low-Speed/run1 baseline reference")
    for key in plans[0]:
        if not np.array_equal(plans[0][key]["center_us"], plans[1][key]["center_us"]):
            raise ValueError(f"Frame centres differ between accumulation windows: {key}")
    print("[CHECK] Both windows use identical inputs, alignment, angles and settings.", flush=True)


def duration_summary(root):
    tables = {}
    for ms in (1, 2):
        for mode in ("raw", "l1"):
            path = root / f"{ms}ms/references_{mode}/summary_by_reference_speed.csv"
            with path.open(newline="", encoding="utf-8-sig") as stream:
                rows = list(csv.DictReader(stream))
            tables[ms, mode] = {
                (r["reference_condition"], r["query_condition"]): r for r in rows
            }
    result = []
    for mode in ("raw", "l1"):
        one, two = tables[1, mode], tables[2, mode]
        if set(one) != set(two):
            raise ValueError("Reference/query speed groups differ between windows")
        for ref, qry in sorted(one):
            row = dict(normalization=mode, reference_condition=ref, query_condition=qry)
            for k in (1, 5):
                field = f"mean_recall_at_{k}"
                x, y = float(one[ref, qry][field]), float(two[ref, qry][field])
                row.update({f"recall_at_{k}_1ms_percent": 100*x,
                            f"recall_at_{k}_2ms_percent": 100*y,
                            f"recall_at_{k}_1ms_minus_2ms_percentage_points": 100*(x-y)})
            result.append(row)
    path = root / "window_comparison_by_speed.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result[0]))
        writer.writeheader(); writer.writerows(result)
    print(f"[DONE] Duration comparison: {path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("prepare", "run"))
    parser.add_argument("--experiments-root", type=Path,
                        default=Path.home() / "Documents/code/Experiments")
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.output_root.expanduser().resolve()
    if args.stage == "prepare":
        # A new parent means there can be no stale extraction caches.
        root.mkdir(parents=True, exist_ok=False)
        for ms in (1, 2):
            execute("init", "--experiments-root", args.experiments_root.expanduser().resolve(),
                    "--output", root / f"{ms}ms/protocol.json",
                    "--accumulation-ms", ms, "--tolerance-deg", 5,
                    "--angle-step-deg", 1, "--start-wrapped-deg", 4,
                    "--width", 160, "--height", 90)
        validate_pair(root)
        from rig_vpr.data import sha256
        plan = dict(sector_deg=30, sector_start_deg=-15, tolerance_deg=5, windows_ms=[1, 2],
                    protocol_sha256={str(ms): sha256(root / f"{ms}ms/protocol.json") for ms in (1, 2)})
        (root / "study_plan.json").write_text(json.dumps(plan, indent=2)+"\n", encoding="utf-8")
        protocol = json.loads((root / "1ms/protocol.json").read_text(encoding="utf-8"))
        fields = ("key", "revolution", "manual_evk4_motion_s", "studio_duration_s", "alignment_source")
        with (root / "accepted_alignment.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for entry in protocol["acquisitions"]:
                writer.writerow({key: entry[key] for key in fields})
        print(f"[READY] Checked protocols: {root}. Run the 'run' stage next.")
        return

    from rig_vpr.data import sha256
    plan = json.loads((root / "study_plan.json").read_text(encoding="utf-8"))
    # Grouping can change without changing either frozen extraction protocol.
    plan.update(sector_deg=30, sector_start_deg=-15)
    for ms in (1, 2):
        if sha256(root / f"{ms}ms/protocol.json") != plan["protocol_sha256"][str(ms)]:
            raise ValueError("A prepared protocol changed; create a new study")
    validate_pair(root)  # Check both before decoding either window.
    from rig_vpr.reporting import plotting
    plotting()  # Missing plot dependencies must fail before expensive extraction.
    (root / "window_study_complete.json").unlink(missing_ok=True)
    for ms in (1, 2):
        folder = root / f"{ms}ms"
        execute("run", "--protocol", folder / "protocol.json", "--overwrite")
        execute("compare-references", "--results", folder / "results",
                "--tolerance-deg", 5, "--normalization", "none",
                "--output-root", folder / "references_raw", "--overwrite")
        execute("compare-references", "--results", folder / "results",
                "--tolerance-deg", 5, "--normalization", "l1",
                "--baseline-comparison", folder / "references_raw",
                "--output-root", folder / "references_l1", "--overwrite")
        sectors = folder / "sectors_30deg_centred"
        if sectors.exists():
            archive = sectors.with_name("sectors_30deg_centred.previous-" +
                datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
            sectors.rename(archive)
            print(f"[ARCHIVE] Previous sectors retained at {archive}", flush=True)
        execute("angle-sectors", "--raw-results", folder / "references_raw",
                "--normalized-results", folder / "references_l1",
                "--sector-deg", 30, "--sector-start-deg", -15,
                "--output-root", folder / "sectors_30deg_centred")
    duration_summary(root)
    (root / "window_study_complete.json").write_text(
        json.dumps(dict(plan, completed_utc=datetime.now(timezone.utc).isoformat()), indent=2)+"\n", encoding="utf-8")
    print(f"[DONE] Full 1 ms / 2 ms VPR comparison: {root}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        sys.exit(f"[ERROR] {exc}")
