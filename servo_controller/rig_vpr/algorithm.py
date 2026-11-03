"""Numeric event counts and exhaustive L1 matching, without angle-based search."""
from __future__ import annotations

import hashlib
import json
import platform
import sys
from pathlib import Path

import numpy as np

from . import VERSION
from .data import resolve


def circular_error(predicted, truth):
    return (np.asarray(predicted) - np.asarray(truth) + 180) % 360 - 180


def count_frame(events, sensor_size, shape, polarity="sum"):
    """Spatially bin [x,y,t_us,p] events, preserving counts (not binary occupancy)."""
    sw, sh = sensor_size
    width, height = shape
    channels = 2 if polarity == "split" else 1
    if width > sw or height > sh:
        raise ValueError("Feature dimensions must not upsample the sensor")
    if not len(events):
        return np.zeros((channels, height, width), dtype=np.float32)
    if events.ndim != 2 or events.shape[1] != 4 or not np.all(np.isfinite(events)):
        raise ValueError("Expected finite EventCV [x,y,timestamp_us,polarity] events")
    x, y, p = events[:, 0], events[:, 1], events[:, 3]
    if np.any(x < 0) or np.any(x >= sw) or np.any(y < 0) or np.any(y >= sh):
        raise ValueError("Event coordinates outside sensor bounds")
    if np.any(x != np.floor(x)) or np.any(y != np.floor(y)):
        raise ValueError("Event pixel coordinates must be integers")
    bx = (x.astype(np.int64) * width // sw)
    by = (y.astype(np.int64) * height // sh)
    index = by * width + bx
    if polarity == "split":
        if not np.all(np.isin(p, (0, 1))):
            raise ValueError("Split polarity expects EventCV values 0 and 1")
        index += p.astype(np.int64) * width * height
    return np.bincount(index, minlength=channels*width*height).reshape(channels, height, width).astype(np.float32)


class EventSource:
    """EventCV RAW source; NPZ support is for transparent fixture/adapter testing."""

    def __init__(self, path, studio_duration_s):
        self.path = Path(path)
        self.reader = None
        self.archive = None
        if self.path.suffix.lower() == ".npz":
            self.archive = np.load(self.path, allow_pickle=False)
            self.events = self.archive["events"]
            if self.events.ndim != 2 or self.events.shape[1] != 4 or not len(self.events):
                raise ValueError("NPZ events must be a non-empty Nx4 [x,y,t_us,p] array")
            if np.any(np.diff(self.events[:, 2]) < 0):
                raise ValueError("NPZ event timestamps must be sorted")
            self.sensor_size = tuple(int(v) for v in self.archive["sensor_size"])
            self.origin_us = int(self.archive["raw_origin_us"].item())
            self.backend = "npz-event-adapter"
        else:
            try:
                import eventcv
            except ImportError as exc:
                raise RuntimeError("RAW extraction requires eventcv in the Python environment used to run this command") from exc
            self.reader = eventcv.open(str(self.path), dt_ms=10)
            self.sensor_size = tuple(int(v) for v in self.reader.sensor_size)
            # Match the established Studio-relative convention, retaining quiet lead-in.
            self.origin_us = round(self.reader.time_span("us")[1] - studio_duration_s * 1e6)
            self.backend = "eventcv"

    def events_between(self, start_us, end_us):
        a, b = self.origin_us + int(start_us), self.origin_us + int(end_us)
        if self.reader is None:
            first, last = np.searchsorted(self.events[:, 2], [a, b], side="left")
            events = self.events[first:last]
        else:
            events = self.reader.slice(t0_us=a, t1_us=b).numpy()
        # Enforce half-open windows even if a backend includes an endpoint.
        if len(events):
            events = events[(events[:, 2] >= a) & (events[:, 2] < b)]
        return events

    def close(self):
        if self.archive is not None:
            self.archive.close()
        close = getattr(self.reader, "close", None)
        if close is not None:
            close()


def code_fingerprints():
    folder = Path(__file__).parent
    files = [*sorted(folder.glob("*.py")), folder.parent/"sad_vpr.py", folder.parent/"generate_vpr_time_bins.py"]
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def raw_fingerprint(path):
    """Cheap cache identity; explicitly not a complete RAW-file checksum."""
    path = Path(path)
    stat = path.stat()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        digest.update(stream.read(1024*1024))
        stream.seek(max(0, stat.st_size-1024*1024))
        digest.update(stream.read(1024*1024))
    return dict(size_bytes=stat.st_size, mtime_ns=stat.st_mtime_ns,
        first_and_last_mib_sha256=digest.hexdigest(), checksum_scope="first and last 1 MiB, not full file")


def extract(entry, cfg, plan, root, cache_dir, audit):
    raw = resolve(root, entry["raw_file"])
    stamp = dict(version=VERSION, entry=entry, settings=cfg, sources=audit["sources"],
        code=code_fingerprints(), raw=raw_fingerprint(raw))
    identity = hashlib.sha256(json.dumps(stamp, sort_keys=True).encode()).hexdigest()
    path = Path(cache_dir) / (entry["key"].replace("/", "_") + "_" + identity[:16] + ".npz")
    if path.is_file():
        with np.load(path, allow_pickle=False) as cached:
            if cached["identity"].item() != identity:
                raise ValueError("Feature cache identity mismatch")
            result = {k:cached[k].copy() for k in cached.files if k not in ("identity", "metadata")}
            metadata = json.loads(cached["metadata"].item())
        if len(result["frames"]) != len(plan["center_us"]):
            raise ValueError("Feature cache frame count mismatch")
        print(f"[CACHE] {entry['key']}: {len(result['frames'])} frames", flush=True)
        return result, metadata
    source = EventSource(raw, entry["studio_duration_s"])
    try:
        n = len(plan["center_us"])
        frames = np.zeros((n, 2 if cfg["polarity"] == "split" else 1, cfg["height"], cfg["width"]), dtype=np.float32)
        counts = np.zeros(n, dtype=np.int64)
        for i, (a, b) in enumerate(zip(plan["start_us"], plan["end_us"])):
            events = source.events_between(a, b)
            counts[i] = len(events)
            frames[i] = count_frame(events, source.sensor_size, (cfg["width"], cfg["height"]), cfg["polarity"])
            if i % 60 == 0 or i == n-1:
                print(f"[FRAMES] {entry['key']}: {i+1}/{n}", flush=True)
        if not counts.any():
            raise ValueError(f"{entry['key']}: every selected RAW window is empty; check camera alignment")
        result = dict(frames=frames, event_counts=counts,
            center_us=plan["center_us"], angle_deg=plan["angle_deg"],
            requested_angle_deg=plan["requested_angle_deg"], start_us=plan["start_us"], end_us=plan["end_us"],
            window_angle_span_deg=plan["window_angle_span_deg"], speed_rpm=plan["speed_rpm"])
        metadata = dict(**stamp, sensor_size=list(source.sensor_size), raw_origin_us=source.origin_us,
            event_source_backend=source.backend, cache_file=path.name)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp.npz")
        np.savez_compressed(temporary, identity=identity, metadata=json.dumps(metadata, sort_keys=True), **result)
        temporary.replace(path)
        return result, metadata
    finally:
        source.close()


def pixel_indices(cfg):
    count = cfg["width"] * cfg["height"]
    if cfg["pixels"] == 0:
        return np.arange(count)
    return np.sort(np.random.default_rng(cfg["seed"]).choice(count, cfg["pixels"], replace=False))


def descriptors(frames, indices, normalization="none"):
    n, channels, height, width = frames.shape
    values = frames.reshape(n, channels, height*width)[:, :, indices].reshape(n, -1).astype(np.float32)
    valid = values.sum(axis=1) > 0
    if normalization == "l1":
        values /= np.maximum(values.sum(axis=1, keepdims=True), 1)
    return values, valid


def sad_matrix(reference, query, backend="auto"):
    """Rows = references, columns = queries. Float subtraction avoids unsigned wrap."""
    ref, qry = np.asarray(reference, dtype=np.float32), np.asarray(query, dtype=np.float32)
    if ref.ndim != 2 or qry.ndim != 2 or ref.shape[1] != qry.shape[1] or not len(ref) or not len(qry):
        raise ValueError("SAD needs non-empty NxD reference and query arrays with the same feature dimension")
    if not np.all(np.isfinite(ref)) or not np.all(np.isfinite(qry)):
        raise ValueError("SAD descriptors must be finite")
    if backend != "numpy":
        try:
            from scipy.spatial.distance import cdist
        except ImportError:
            if backend == "scipy":
                raise
        else:
            return cdist(ref, qry, metric="cityblock"), "scipy-cityblock"
    result = np.empty((len(ref), len(qry)), dtype=np.float64)
    # Bound the temporary array to ~2 million floats regardless of image size.
    qblock = 4
    rblock = max(1, min(32, 2_000_000 // max(1, qblock * ref.shape[1])))
    for q in range(0, len(qry), qblock):
        for r in range(0, len(ref), rblock):
            difference = ref[r:r+rblock, None, :] - qry[None, q:q+qblock, :]
            result[r:r+rblock, q:q+qblock] = np.abs(difference).sum(axis=2, dtype=np.float64)
    return result, "numpy-blocked-l1"


def match(reference, query, cfg, indices):
    ref, ref_valid = descriptors(reference["frames"], indices, cfg["normalization"])
    qry, qry_valid = descriptors(query["frames"], indices, cfg["normalization"])
    if not ref_valid.any():
        raise ValueError("All reference descriptors are empty at the chosen pixels")
    scores, backend = sad_matrix(ref, qry)
    result = evaluate_scores(scores, reference["angle_deg"], query["angle_deg"], ref_valid, qry_valid, cfg["tolerance_deg"])
    result["backend"] = backend
    return result


def rank_candidates(scores, reference_valid, k=5):
    """Rank by score only. Give no credit to ties spilling across the K boundary.

    A correct candidate is guaranteed inside top K only when its entire score-tie
    group fits. Higher ranked candidates still count if a lower boundary is tied.
    Stable reference-index ordering is for display, never angle-based tie breaking.
    """
    count = int(np.sum(reference_valid))
    if k < 1 or not count:
        raise ValueError("Ranking needs positive K and a nonempty reference set")
    effective_k = min(k, count)
    order = np.argsort(scores, axis=0, kind="stable")[:effective_k]
    columns = np.arange(scores.shape[1])
    top_scores = scores[order, columns]
    cutoff = top_scores[-1]
    tied = np.isclose(scores, cutoff[None, :], atol=1e-8, rtol=1e-7)
    before = (scores < cutoff[None, :]) & ~tied
    overflow = ((before | tied) & reference_valid[:, None]).sum(axis=0) > effective_k
    credited = reference_valid[:, None] & (before | (tied & ~overflow[None, :]))
    return dict(indices=order, scores=top_scores, credited=credited[order, columns],
                boundary_tie=overflow, effective_k=effective_k)


def evaluate_scores(scores, reference_angles, query_angles, reference_valid, query_valid, tolerance):
    """Evaluate saved or newly computed scores, preserving the original Recall@1."""
    scores = np.asarray(scores, dtype=np.float64).copy()
    ref_angles, qry_angles = np.asarray(reference_angles), np.asarray(query_angles)
    ref_valid, qry_valid = np.asarray(reference_valid, dtype=bool), np.asarray(query_valid, dtype=bool)
    if scores.shape != (len(ref_angles), len(qry_angles)) or ref_valid.shape != ref_angles.shape or qry_valid.shape != qry_angles.shape:
        raise ValueError("Score matrix, angles and validity masks have inconsistent shapes")
    if not len(qry_angles) or not ref_valid.any() or not np.all(np.isfinite(np.r_[ref_angles, qry_angles])):
        raise ValueError("Evaluation needs finite angles, queries and a nonempty reference set")
    if not np.all(np.isfinite(scores[ref_valid])) or np.any(scores[ref_valid] < 0):
        raise ValueError("Nonempty reference candidates must have finite nonnegative scores")
    # Exclude empty reference descriptors for every query, without consulting angles.
    scores[~ref_valid] = np.inf
    best = scores.argmin(axis=0)
    minimum = scores[best, np.arange(len(qry_angles))]
    ties = np.isclose(scores, minimum[None, :], atol=1e-8, rtol=1e-7).sum(axis=0)
    accepted = qry_valid & (ties == 1)
    # Ground truth enters only after the exhaustive search has completed.
    error = circular_error(ref_angles[best], qry_angles)
    nearest_gt = np.min(abs(circular_error(ref_angles[ref_valid, None], qry_angles[None, :])), axis=0)
    status = np.where(~qry_valid, "empty_query_descriptor", np.where(ties > 1, "ambiguous_tie", "matched"))
    correct = accepted & (abs(error) <= tolerance + 1e-8)
    second = np.partition(scores, 1, axis=0)[1] if len(ref_angles) > 1 else np.full(len(qry_angles), np.inf)
    ranked = rank_candidates(scores, ref_valid, 5)
    top_errors = circular_error(ref_angles[ranked["indices"]], qry_angles[None, :])
    credited = ranked["credited"] & qry_valid[None, :]
    return dict(scores=scores, best=best, best_distance=minimum, second_distance=second,
        ties=ties, accepted=accepted, status=status, correct=correct, error_deg=error,
        nearest_reference_error_deg=nearest_gt, reference_valid=ref_valid, query_valid=qry_valid,
        reference_angles_deg=ref_angles, query_angles_deg=qry_angles,
        top_indices=ranked["indices"], top_scores=ranked["scores"], top_credited=credited,
        top_errors_deg=top_errors, top5_boundary_tie=ranked["boundary_tie"], top5_effective_k=ranked["effective_k"],
        correct_at_5=np.any(credited & (abs(top_errors) <= tolerance + 1e-8), axis=0))


def metrics(result, tolerance):
    valid = result["accepted"]
    errors = abs(result["error_deg"][valid])
    correct5 = np.any(result["top_credited"] & (abs(result["top_errors_deg"]) <= tolerance + 1e-8), axis=0)
    return dict(query_count=len(valid), matched_queries=int(valid.sum()),
        empty_queries=int(np.sum(result["status"] == "empty_query_descriptor")),
        ambiguous_queries=int(np.sum(result["status"] == "ambiguous_tie")),
        correct_queries=int(np.sum(valid & (abs(result["error_deg"]) <= tolerance + 1e-8))),
        recall_at_1=float(np.mean(valid & (abs(result["error_deg"]) <= tolerance + 1e-8))),
        correct_queries_at_5=int(correct5.sum()), recall_at_5=float(correct5.mean()),
        top5_effective_k=result["top5_effective_k"],
        top5_queries_with_credited_candidates=int(np.any(result["top_credited"], axis=0).sum()),
        top5_boundary_tie_queries=int(np.sum(result["top5_boundary_tie"] & result["query_valid"])),
        match_coverage=float(valid.mean()),
        mean_absolute_error_deg_matched=float(errors.mean()) if len(errors) else None,
        median_absolute_error_deg_matched=float(np.median(errors)) if len(errors) else None,
        p95_absolute_error_deg_matched=float(np.percentile(errors, 95)) if len(errors) else None,
        maximum_nearest_reference_error_deg=float(result["nearest_reference_error_deg"].max()))


def runtime_versions():
    result = dict(python=sys.version, platform=platform.platform(), numpy=np.__version__)
    from importlib.metadata import PackageNotFoundError, version
    for package in ("eventcv", "scipy", "matplotlib"):
        try:
            result[package] = version(package)
        except PackageNotFoundError:
            result[package] = None
    return result
