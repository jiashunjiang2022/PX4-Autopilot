#!/usr/bin/env python3
"""Generate the V3 engineering-only Shadow model constexpr header."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import joblib
import numpy as np
import pandas as pd


DEFAULT_ARTIFACT = Path(
    "/Users/jiangjiashun/Documents/门控数据分析/revision_analysis/"
    "20260910_v3_engineering_shadow_artifact"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cpp_float(value: float) -> str:
    value = float(np.float32(value))
    if not np.isfinite(value):
        raise ValueError("model constants must be finite")
    return f"{value:.9g}f"


def array(name: str, values: np.ndarray) -> str:
    flat = np.asarray(values).reshape(-1)
    lines = []
    for start in range(0, len(flat), 6):
        lines.append("\t" + ", ".join(cpp_float(item) for item in flat[start:start + 6]))
    return f"static constexpr float {name}[FeatureCount]{{\n" + ",\n".join(lines) + "\n};\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact", type=Path, default=DEFAULT_ARTIFACT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.artifact.resolve()
    manifest_path = root / "V3_ENGINEERING_MODEL_MANIFEST.json"
    model_path = root / "V3_ENGINEERING_MODEL.joblib"
    normalization_path = root / "V3_ENGINEERING_NORMALIZATION.npz"
    feature_path = root / "V3_ENGINEERING_FEATURE_ORDER.csv"
    manifest = json.loads(manifest_path.read_text())
    if manifest["role"] != "ENGINEERING_SHADOW_ONLY":
        raise ValueError("V3 artifact is not engineering Shadow only")
    if manifest["scientific_status"] != "PARTIAL_DEVELOPMENT_NOT_VALIDATED":
        raise ValueError("V3 scientific status changed")
    if manifest["selected_formulation"] != "BOUNDED_SCHEME_B":
        raise ValueError("V3 selected formulation changed")
    for field, path in (("model_sha256", model_path),
                        ("normalization_sha256", normalization_path),
                        ("feature_order_sha256", feature_path)):
        if sha256(path) != manifest[field]:
            raise ValueError(f"V3 artifact hash mismatch: {field}")

    model = joblib.load(model_path)
    normalization = np.load(normalization_path)
    features = pd.read_csv(feature_path).sort_values("order").feature.astype(str).tolist()
    coefficients = np.asarray(model.coef_, dtype=np.float64).reshape(-1)
    mean = np.asarray(normalization["mean"], dtype=np.float64).reshape(-1)
    scale = np.asarray(normalization["scale"], dtype=np.float64).reshape(-1)
    if not len(coefficients) == len(mean) == len(scale) == len(features) == 64:
        raise ValueError("V3 dimensions are not the frozen 64-feature contract")
    if not np.isfinite(np.r_[coefficients, mean, scale]).all() or np.any(scale <= 0):
        raise ValueError("V3 model contains invalid constants")
    names = "static constexpr const char *FeatureNames[FeatureCount]{\n" + \
        "\n".join(f'\t"{name}"{"," if i + 1 < len(features) else ""}'
                    for i, name in enumerate(features)) + "\n};\n"
    text = f"""// AUTO-GENERATED - DO NOT EDIT
// ENGINEERING_SHADOW_ONLY - PARTIAL_DEVELOPMENT_NOT_VALIDATED
// NONZERO_INJECTION_FORBIDDEN
// SOURCE_MANIFEST_SHA256={sha256(manifest_path)}
// SOURCE_MODEL_SHA256={sha256(model_path)}
// SOURCE_NORMALIZATION_SHA256={sha256(normalization_path)}
// FEATURE_ORDER_SHA256={sha256(feature_path)}
// GENERATOR_SHA256={sha256(Path(__file__))}
#pragma once

#include <cstddef>

namespace flap_aug_shadow::generated::v3_slow
{{
static constexpr size_t FeatureCount = 64;
static constexpr float Intercept = {cpp_float(model.intercept_)};
{array("Mean", mean)}
{array("Scale", scale)}
{array("Coefficient", coefficients)}
{names}
}} // namespace flap_aug_shadow::generated::v3_slow
"""
    args.output.resolve().parent.mkdir(parents=True, exist_ok=True)
    args.output.resolve().write_text(text, encoding="ascii")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
