#!/usr/bin/env python3
"""Apply the pre-existing deterministic V3 final-fit rule for Shadow use."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
from sklearn.linear_model import Ridge
from sklearn.preprocessing import StandardScaler


ANALYSIS_ROOT = Path("/Users/jiangjiashun/Documents/门控数据分析")
REVISION_ROOT = ANALYSIS_ROOT / "revision_analysis"
FINAL_DEV = REVISION_ROOT / "20260909_phase_conditioned_slow_trim_final_dev"
SCRIPTS = REVISION_ROOT / "scripts"
DEFAULT_OUTPUT = REVISION_ROOT / "20260910_v3_engineering_shadow_artifact"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite nonempty output: {output}")
    output.mkdir(parents=True, exist_ok=True)

    sys.path.insert(0, str(SCRIPTS))
    from phase_conditioned_slow_trim_final_dev import (  # pylint: disable=import-outside-toplevel
        Audit, CONFIDENCE_PROTOCOL, PHASE_PROTOCOL, SLOW_UPDATE_PROTOCOL,
    )

    verdicts = json.loads((FINAL_DEV / "14_verification/verdicts.json").read_text())
    if verdicts["selected_candidate"] != "BOUNDED_SCHEME_B" or verdicts["V3_SLOW_CANDIDATE_STATUS"] != "PARTIAL":
        raise ValueError("V3 development identity changed")

    temporary = Path(tempfile.mkdtemp(prefix="v3_engineering_finalfit_"))
    try:
        audit = Audit(temporary / "audit")
        audit.load_and_classify()
        if len(audit.features) != 64:
            raise ValueError("V3 feature count changed")
        flights = audit.data.flight.astype(str)
        counts = flights.value_counts()
        weights = flights.map(lambda name: 1.0 / counts[name]).to_numpy(float, copy=True)
        weights *= len(weights) / weights.sum()
        scaler = StandardScaler()
        transformed = scaler.fit_transform(audit.data[audit.features], sample_weight=weights)
        alpha = float(audit.model_provenance.selected_alpha.median())
        model = Ridge(alpha=alpha, solver="lsqr").fit(
            transformed, audit.data.target_causal_dynamic, sample_weight=weights)

        model_path = output / "V3_ENGINEERING_MODEL.joblib"
        norm_path = output / "V3_ENGINEERING_NORMALIZATION.npz"
        feature_path = output / "V3_ENGINEERING_FEATURE_ORDER.csv"
        golden_path = output / "V3_ENGINEERING_GOLDEN_VECTORS.npz"
        joblib.dump(model, model_path, compress=3)
        np.savez(norm_path, mean=scaler.mean_, scale=scaler.scale_)

        feature_rows = []
        for order, feature in enumerate(audit.features):
            delay = 0.0
            if "__lag_" in feature:
                delay = float(feature.split("__lag_")[1][:-1].replace("p", "."))
            elif feature.startswith("delta__"):
                delay = float(feature.rsplit("__", 1)[1][:-1].replace("p", "."))
            feature_rows.append({"order": order, "feature": feature, "delay_s": delay})
        pd.DataFrame(feature_rows).to_csv(feature_path, index=False, float_format="%.12g")

        indices = np.unique(np.linspace(0, len(audit.data) - 1, 64, dtype=int))
        raw = audit.data.iloc[indices][audit.features].to_numpy(float)
        prediction = model.predict(((raw - scaler.mean_) / scaler.scale_).astype(np.float64))
        np.savez(golden_path, row_index=indices, features=raw.astype(np.float64),
                 prediction=prediction.astype(np.float64), feature_names=np.asarray(audit.features))

        spec_path = output / "V3_ENGINEERING_IMPLEMENTATION_SPEC.md"
        spec_path.write_text(f"""# V3 Slow Engineering Shadow Model

- role: `ENGINEERING_SHADOW_ONLY`
- scientific status: `PARTIAL_DEVELOPMENT_NOT_VALIDATED`
- model: Ridge `M3_DYNAMIC_HISTORY_NO_RTK`
- selected formulation: `BOUNDED_SCHEME_B`
- training procedure: pre-existing deterministic full-data rule in `phase_conditioned_slow_trim_final_dev.py::maybe_freeze`
- training weights: equal total weight per development flight
- Ridge alpha: `{alpha}` (median of the existing five outer-fold selected alphas)
- target: `target_causal_dynamic`, a slow closed-loop residual corrective-demand development target
- features: 64, exact order in `V3_ENGINEERING_FEATURE_ORDER.csv`
- history: 1.0 s at 50 Hz, past/current samples only
- scaling: `StandardScaler` weighted with the same per-flight sample weights
- confidence: SCHEME_B (straight 1.0, steady 0.4, entry/exit/invalid 0)
- slow update alpha: `{SLOW_UPDATE_PROTOCOL['alpha']}`
- innovation clip: `{SLOW_UPDATE_PROTOCOL['innovation_clip']}`
- output clip: `{SLOW_UPDATE_PROTOCOL['output_clip']}`
- initialization: `{SLOW_UPDATE_PROTOCOL['initialization']}`
- state retention: Stabilized to Mission retains state
- reset: disarm, failsafe, invalid state, or continuous data loss over 2 s
- phase protocol: `{PHASE_PROTOCOL['protocol_name']}`
- validation claim: NOT PROSPECTIVE VALIDATED
- deployment authorization: NO
- nonzero actuator injection: FORBIDDEN
""", encoding="utf-8")

        source_paths = {
            "phase_conditioned_slow_trim_final_dev.py": SCRIPTS / "phase_conditioned_slow_trim_final_dev.py",
            "phase_conditioned_slow_trim_core.py": SCRIPTS / "phase_conditioned_slow_trim_core.py",
            "generator": Path(__file__).resolve(),
        }
        protocol_paths = {
            path.name: path for path in sorted((FINAL_DEV / "01_protocol").glob("*.json"))
        }
        manifest_path = output / "V3_ENGINEERING_MODEL_MANIFEST.json"
        manifest = {
            "role": "ENGINEERING_SHADOW_ONLY",
            "scientific_status": "PARTIAL_DEVELOPMENT_NOT_VALIDATED",
            "model_name": "V3_SLOW_ENGINEERING_SHADOW_MODEL",
            "model_family": "M3_DYNAMIC_HISTORY_NO_RTK",
            "selected_formulation": "BOUNDED_SCHEME_B",
            "alpha": alpha,
            "intercept": float(model.intercept_),
            "target_formulation": "target_causal_dynamic",
            "feature_count": len(audit.features),
            "history": {"sample_rate_hz": 50.0, "history_s": 1.0, "causal": True},
            "training_cohort_identity": "FINAL_PRE_FLIGHT_V3_DEVELOPMENT_22_FLIGHT_ROLE_LEDGER",
            "training_flights": sorted(flights.unique().tolist()),
            "training_sample_count": len(audit.data),
            "training_weighting": "EQUAL_TOTAL_WEIGHT_PER_FLIGHT",
            "feature_order_sha256": sha256(feature_path),
            "normalization_sha256": sha256(norm_path),
            "model_sha256": sha256(model_path),
            "golden_vectors_sha256": sha256(golden_path),
            "implementation_spec_sha256": sha256(spec_path),
            "source_script_hashes": {name: sha256(path) for name, path in source_paths.items()},
            "protocol_hashes": {name: sha256(path) for name, path in protocol_paths.items()},
            "phase_protocol": PHASE_PROTOCOL,
            "confidence_scheme_b": CONFIDENCE_PROTOCOL["schemes"]["SCHEME_B"],
            "slow_update_protocol": SLOW_UPDATE_PROTOCOL,
            "generation_timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "not_prospective_validated": True,
            "deployment_authorized": False,
            "nonzero_injection": False,
        }
        write_json(manifest_path, manifest)
    finally:
        shutil.rmtree(temporary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
