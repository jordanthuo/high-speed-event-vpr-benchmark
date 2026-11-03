"""Read completed-run identities and reconstruct its frozen manual time mapping."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np

from .algorithm import raw_fingerprint
from .data import frame_plan,load_trace,read_csv,resolve,sha256,validate_protocol


def completed_source(path):
    source=Path(path).expanduser().resolve()
    protocol=json.loads((source/"protocol_snapshot.json").read_text(encoding="utf-8"))
    audit=json.loads((source/"provenance.json").read_text(encoding="utf-8"))
    complete=json.loads((source/"run_complete.json").read_text(encoding="utf-8"))
    validate_protocol(protocol)
    if audit["protocol"]!=protocol or audit["protocol_sha256"]!=complete["protocol_sha256"] or sha256(source/"protocol_snapshot.json")!=complete["protocol_sha256"]:
        raise ValueError("Completed source protocol/provenance hashes disagree")
    root=resolve(source.parent,protocol["experiments_root"])
    return dict(path=source,root=root,protocol=protocol,audit=audit,complete=complete,
        windows={(r["acquisition"],int(r["frame_index"])):r for r in read_csv(source/"frame_windows.csv")})


def frozen_trace(source,entry):
    """Manual onsets are copied from the completed protocol, never rediscovered."""
    inputs={}
    for field in ("recorded_csv","camera_metadata"):
        path=resolve(source["root"],entry[field])
        expected=source["audit"]["inputs"][entry["key"]]["sources"][field]["sha256"]
        if sha256(path)!=expected:
            raise ValueError(f"{entry['key']}: frozen {field} changed")
        inputs[field]=dict(path=entry[field],sha256=expected)
    mapping=dict(entry)
    if entry["sync_mode"]=="manual":
        mapping["time_bins_csv"]=None
    trace=load_trace(source["root"],mapping)
    original=frame_plan(trace,entry,source["protocol"]["settings"])
    for i,center in enumerate(original["center_us"]):
        row=source["windows"][(entry["key"],i)]
        if abs(float(row["center_s"])*1e6-center)>.01 or not np.isclose(float(row["wrapped_angle_deg"]),original["angle_deg"][i],rtol=0,atol=1e-8):
            raise ValueError(f"{entry['key']}: reconstructed mapping does not reproduce frozen centres")
    inputs["source_protocol"]=dict(path=str(source["path"]/"protocol_snapshot.json"),sha256=source["complete"]["protocol_sha256"])
    return trace,inputs


def verify_raw(source,entry):
    path=resolve(source["root"],entry["raw_file"])
    expected=source["audit"]["inputs"][entry["key"]]["raw"]
    current=raw_fingerprint(path)
    for field in ("size_bytes","first_and_last_mib_sha256"):
        if current[field]!=expected[field]:
            raise ValueError(f"{entry['key']}: RAW identity changed")
    return path,current


def frozen_cache(source,entry):
    metadata=source["audit"]["inputs"][entry["key"]]
    name=metadata["cache_file"]
    if Path(name).name!=name:
        raise ValueError("Source cache filename must be a basename")
    path=source["path"].parent/"cache"/name
    with np.load(path,allow_pickle=False) as saved:
        if json.loads(saved["metadata"].item())!=metadata:
            raise ValueError("Source cache metadata disagree")
        result={k:saved[k].copy() for k in saved.files if k not in ("metadata","identity")}
    if not np.all(np.isfinite(result["frames"])) or np.any(result["frames"]<0) or not np.allclose(result["frames"].sum(axis=(1,2,3),dtype=np.float64),result["event_counts"],rtol=0,atol=.1):
        raise ValueError("Source cached counts are invalid")
    return result,dict(cache_file=name,sha256=sha256(path))
