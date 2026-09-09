#!/usr/bin/env python3
"""Export immutable historical vectors for the standalone C++ parity harness."""

from __future__ import annotations

import argparse
import csv
import hashlib
import struct
from pathlib import Path

import joblib
import numpy as np
import pandas as pd


V2_ROOT = Path("/Users/jiangjiashun/Documents/门控数据分析/revision_analysis/20260907_v2_model_development")
V3_ROOT = Path("/Users/jiangjiashun/Documents/门控数据分析/revision_analysis/20260910_v3_engineering_shadow_artifact")
PHASE_TRACE = Path("/Users/jiangjiashun/Documents/门控数据分析/revision_analysis/20260909_phase_conditioned_slow_trim_final_dev/02_phase_detection/phase_trace.csv")
PHASES = {
    "STRAIGHT_OR_LOW_MANEUVER": 0,
    "TURN_ENTRY": 1,
    "STEADY_TURN": 2,
    "TURN_EXIT": 3,
    "EXTREME_OR_INVALID": 4,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transformed(model, normalization, values):
    z = (np.asarray(values, float) - normalization["mean"]) / normalization["std"]
    return model.predict(np.nan_to_num(z).astype(np.float32))


def temporal_summary(history):
    selected = np.asarray(history, float)
    parts = [selected[index] for index in [99, 94, 89, 74, 49, 0]]
    parts.extend([np.nanmean(selected, axis=0), np.nanstd(selected, axis=0),
                  np.nanmin(selected, axis=0), np.nanmax(selected, axis=0)])
    time = np.arange(100, dtype=float) * 0.02
    centered = time - time.mean()
    slope = np.einsum("t,tf->f", centered,
                      selected - np.nanmean(selected, axis=0, keepdims=True)) / np.sum(centered ** 2)
    parts.append(slope)
    return np.concatenate(parts).astype(np.float32)


def write_v2(output: Path):
    data_path = V2_ROOT / "cache/v2_development_dataset.npz"
    data = np.load(data_path, allow_pickle=True)
    usable = np.flatnonzero(np.isfinite(data["X"]).all(axis=(1, 2)))
    selected = []
    for flight in sorted(np.unique(data["flight_id"])):
        candidates = usable[data["flight_id"][usable] == flight]
        if len(candidates):
            selected.append(int(candidates[len(candidates) // 2]))
    remaining = np.setdiff1d(usable, np.asarray(selected, int))
    selected.extend(map(int, remaining[np.linspace(0, len(remaining) - 1, 64 - len(selected), dtype=int)]))
    selected = np.asarray(selected, int)

    simple = V2_ROOT / "models/candidates/V2_SIMPLE"
    temporal = V2_ROOT / "models/candidates/V2_TEMPORAL"
    roll_model = joblib.load(simple / "roll_model.joblib")
    roll_norm = np.load(simple / "roll_normalization.npz")
    base_model = joblib.load(temporal / "pitch_base_model.joblib")
    base_norm = np.load(temporal / "pitch_base_normalization.npz")
    residual_model = joblib.load(temporal / "pitch_residual_model.joblib")
    residual_norm = np.load(temporal / "pitch_residual_normalization.npz")
    roll_columns = [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32]

    vector_path = output / "v2_golden.bin"
    with vector_path.open("wb") as stream:
        stream.write(b"FASV2G1\0")
        stream.write(struct.pack("<I", len(selected)))
        for row in selected:
            history = data["X"][row].astype("<f4")
            current = history[-1]
            roll = transformed(roll_model, roll_norm, current[roll_columns][None, :])[0]
            base = transformed(base_model, base_norm, current[None, :])[0]
            temporal_features = temporal_summary(history)
            residual = transformed(residual_model, residual_norm, temporal_features[None, :])[0]
            history.tofile(stream)
            np.asarray([roll, base, residual, base + residual], dtype="<f4").tofile(stream)
    with (output / "v2_golden_index.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["vector", "dataset_row", "flight", "cohort", "anchor_time"])
        for vector, row in enumerate(selected):
            writer.writerow([vector, row, data["flight_id"][row], data["cohort"][row], data["anchor_time"][row]])
    return vector_path, data_path


def write_v3_model(output: Path):
    source = V3_ROOT / "V3_ENGINEERING_GOLDEN_VECTORS.npz"
    data = np.load(source)
    path = output / "v3_model_golden.bin"
    with path.open("wb") as stream:
        stream.write(b"FASV3G1\0")
        stream.write(struct.pack("<I", len(data["features"])))
        for features, prediction in zip(data["features"], data["prediction"], strict=True):
            np.asarray(features, dtype="<f4").tofile(stream)
            np.asarray([prediction], dtype="<f4").tofile(stream)
    return path, source


def write_phase_and_slow(output: Path):
    frame = pd.read_csv(PHASE_TRACE)
    phase_path = output / "phase_golden.bin"
    slow_path = output / "slow_update_golden.bin"
    previous = None
    with phase_path.open("wb") as phase_stream, slow_path.open("wb") as slow_stream:
        phase_stream.write(b"FASPHG1\0")
        phase_stream.write(struct.pack("<I", len(frame)))
        slow_stream.write(b"FASSLG1\0")
        slow_stream.write(struct.pack("<I", len(frame)))
        for row in frame.itertuples(index=False):
            key = (row.flight, row.segment_id)
            reset = key != previous
            previous = key
            extreme = row.phase == "EXTREME_OR_INVALID"
            phase_stream.write(struct.pack("<BBBB6f", reset, True, extreme, PHASES[row.phase],
                                           row.manual_roll, row.roll_sp, row.p_sp, row.p,
                                           row.ground_track_rate, row.confidence__SCHEME_B))
            slow_stream.write(struct.pack("<B5f", reset, row.residual__M3_DYNAMIC_HISTORY,
                                          row.confidence__SCHEME_B, row.b_hat__BOUNDED_SCHEME_B,
                                          row.innovation__BOUNDED_SCHEME_B,
                                          row.update__BOUNDED_SCHEME_B))
    return phase_path, slow_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    v2, v2_source = write_v2(output)
    v3, v3_source = write_v3_model(output)
    phase, slow = write_phase_and_slow(output)
    rows = [
        ("v2_golden", v2, v2_source),
        ("v3_model_golden", v3, v3_source),
        ("phase_golden", phase, PHASE_TRACE),
        ("slow_update_golden", slow, PHASE_TRACE),
    ]
    with (output / "golden_manifest.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["role", "path", "sha256", "source_path", "source_sha256"])
        for role, path, source in rows:
            writer.writerow([role, path, sha256(path), source, sha256(source)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
