#!/usr/bin/env python3
"""Generate constexpr V2 Shadow model headers from frozen artifacts."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import joblib
import numpy as np
import pandas as pd


DEFAULT_ARTIFACT_ROOT = Path(
    "/Users/jiangjiashun/Documents/门控数据分析/revision_analysis/20260907_v2_model_development"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cpp_float(value: float) -> str:
    value = float(np.float32(value))
    if not np.isfinite(value):
        raise ValueError("model constants must be finite")
    return f"{value:.9g}f"


def array(name: str, values: np.ndarray) -> str:
    rows = []
    flat = np.asarray(values).reshape(-1)
    for start in range(0, len(flat), 6):
        rows.append("\t" + ", ".join(cpp_float(v) for v in flat[start:start + 6]))
    return f"static constexpr float {name}[FeatureCount]{{\n" + ",\n".join(rows) + "\n};\n"


def header(namespace: str, model_path: Path, normalization_path: Path,
           feature_manifest_hash: str, extra: str = "") -> str:
    model = joblib.load(model_path)
    normalization = np.load(normalization_path)
    coefficients = np.asarray(model.coef_, dtype=np.float32).reshape(-1)
    mean = np.asarray(normalization["mean"], dtype=np.float64).reshape(-1)
    std = np.asarray(normalization["std"], dtype=np.float64).reshape(-1)
    if not (len(coefficients) == len(mean) == len(std)):
        raise ValueError(f"dimension mismatch for {model_path}")
    if np.any(~np.isfinite(mean)) or np.any(~np.isfinite(std)) or np.any(std <= 0):
        raise ValueError(f"invalid scaler for {model_path}")
    return f"""// AUTO-GENERATED - DO NOT EDIT
// SOURCE_ARTIFACT_SHA256={sha256(model_path)}
// SOURCE_NORMALIZATION_SHA256={sha256(normalization_path)}
// FEATURE_MANIFEST_SHA256={feature_manifest_hash}
// GENERATOR_SHA256={sha256(Path(__file__))}
#pragma once

#include <cstddef>

namespace flap_aug_shadow::generated::{namespace}
{{
static constexpr size_t FeatureCount = {len(coefficients)};
static constexpr float Intercept = {cpp_float(model.intercept_)};
{array("Mean", mean)}
{array("Std", std)}
{array("Coefficient", coefficients)}
{extra}
}} // namespace flap_aug_shadow::generated::{namespace}
"""


def validate_freeze_manifest(root: Path) -> None:
    with (root / "tables/candidate_freeze_manifest.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 18:
        raise ValueError(f"expected 18 frozen artifacts, found {len(rows)}")
    for row in rows:
        path = root / row["path"]
        if sha256(path) != row["sha256"] or path.stat().st_size != int(row["size_bytes"]):
            raise ValueError(f"frozen artifact mismatch: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.artifact_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    validate_freeze_manifest(root)

    simple = root / "models/candidates/V2_SIMPLE"
    temporal = root / "models/candidates/V2_TEMPORAL"
    simple_manifest = json.loads((simple / "candidate_manifest.json").read_text())
    temporal_manifest = json.loads((temporal / "candidate_manifest.json").read_text())
    if simple_manifest["axes"]["roll"]["model_name"] != "CONTROLLER_ONLY":
        raise ValueError("Roll frozen candidate is not CONTROLLER_ONLY")
    if temporal_manifest["model"] != "RIDGE_PLUS_LINEAR_RESIDUAL_TEMPORAL_FULL":
        raise ValueError("Pitch frozen temporal candidate identity mismatch")

    feature_manifest = pd.read_csv(simple / "feature_manifest.csv")
    feature_hash = sha256(simple / "feature_manifest.csv")
    controller_groups = {
        "INTEGRATOR_STATE", "CONTROLLER_STATE", "REFERENCE_AND_TRACKING",
        "CONTROLLER_OUTPUT", "ACTUATOR_OR_TAIL_COMMAND",
    }
    roll_features = feature_manifest.loc[
        feature_manifest.feature_group.isin(controller_groups), "feature"
    ].astype(str).tolist()
    if len(roll_features) != 20:
        raise ValueError("Roll feature count is not 20")
    pitch_features = feature_manifest.feature.astype(str).tolist()
    if len(pitch_features) != 47:
        raise ValueError("Pitch base feature count is not 47")

    roll_extra = "static constexpr const char *FeatureNames[FeatureCount]{\n" + \
        "\n".join(f'\t"{name}"{"," if i + 1 < len(roll_features) else ""}' for i, name in enumerate(roll_features)) + \
        "\n};\n"
    pitch_extra = "static constexpr const char *FeatureNames[FeatureCount]{\n" + \
        "\n".join(f'\t"{name}"{"," if i + 1 < len(pitch_features) else ""}' for i, name in enumerate(pitch_features)) + \
        "\n};\n"
    temporal_extra = "static constexpr size_t HistorySamples = 100;\n" \
        "static constexpr size_t BaseFeatureCount = 47;\n" \
        "static constexpr size_t LagIndices[6]{99, 94, 89, 74, 49, 0};\n"

    (output / "v2_roll_model.hpp").write_text(header(
        "v2_roll", simple / "roll_model.joblib", simple / "roll_normalization.npz",
        feature_hash, roll_extra), encoding="ascii")
    (output / "v2_pitch_base_model.hpp").write_text(header(
        "v2_pitch_base", temporal / "pitch_base_model.joblib",
        temporal / "pitch_base_normalization.npz", feature_hash, pitch_extra), encoding="ascii")
    (output / "v2_pitch_temporal_model.hpp").write_text(header(
        "v2_pitch_temporal", temporal / "pitch_residual_model.joblib",
        temporal / "pitch_residual_normalization.npz", feature_hash, temporal_extra), encoding="ascii")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

