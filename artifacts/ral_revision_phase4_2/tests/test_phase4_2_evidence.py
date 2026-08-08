#!/usr/bin/env python3

import importlib.util
import pathlib
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[3]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVIDENCE = load_module(
    "phase4_2_evidence",
    ROOT / "artifacts/ral_revision_phase4_2/ulog_evidence/check_airspeed_evidence.py",
)
PREFLIGHT = load_module(
    "phase4_1_preflight",
    ROOT / "artifacts/ral_revision_phase4_1/parameter_template/preflight_check.py",
)


def selector_row(timestamp, match, source=1, final_source=1):
    return {
        "timestamp": timestamp,
        "decision_timestamp_sample": timestamp - 10_000,
        "quality_timestamp_sample": timestamp - 20_000,
        "pre_quality_device_id": 42 if match else 0,
        "quality_device_id": 42 if match else 0,
        "final_device_id": 42 if final_source in {1, 2, 3} else 0,
        "experiment_mode": 3,
        "selector_quality_enabled": True,
        "pre_quality_source": source,
        "original_selected_source": source,
        "quality_source_instance": 0 if match else 255,
        "source_identity_match": match,
        "pre_quality_output_finite": True,
        "original_sensor_valid": source in {1, 2, 3},
        "original_selection_was_fallback": source in {0, 4},
        "concurrent_original_sensor_invalid": source not in {1, 2, 3},
        "concurrent_blockage": False,
        "quality_rejected": False,
        "trigger_reason": 2 if source in {0, 4} else 0,
        "fallback_outcome": 0,
        "fallback_attempted": False,
        "fallback_available": False,
        "fallback_source": -1,
        "final_selected_source": final_source,
        "final_valid": final_source != -1,
    }


def diagnostic_row(observation_timestamp=1_500_000, buffer_timestamp=1_380_000, mode=2,
                   quality_timestamp=1_480_000, quality=0.8, nominal_r=4.0,
                   source=1, device_id=42, identity_match=True, causal=True,
                   fresh=True, quality_valid=True, adaptive_requested=True,
                   adaptive_applied=True, fallback_reason=0):
    rmax = 5.0
    r_used = nominal_r
    if adaptive_applied:
        r_used = nominal_r * (1.0 + (1.0 - quality) * (rmax - 1.0))
    return {
        "_multi_id": 0,
        "timestamp": observation_timestamp + 500_000,
        "timestamp_sample": observation_timestamp,
        "ekf_buffer_timestamp_sample": buffer_timestamp,
        "quality_timestamp_sample": quality_timestamp,
        "quality_age_us": observation_timestamp - quality_timestamp if causal else 0,
        "airspeed_q": quality,
        "quality_input_valid": quality_valid,
        "quality_source_instance": 0 if identity_match else 255,
        "quality_device_id": device_id if identity_match else 0,
        "airspeed_source": source,
        "airspeed_device_id": device_id if source in {1, 2, 3} else 0,
        "source_identity_match": identity_match,
        "nominal_r_as": nominal_r,
        "r_as_used": r_used,
        "experiment_mode": mode,
        "adaptive_r_enabled": adaptive_applied,
        "adaptive_r_requested": adaptive_requested,
        "adaptive_r_applied": adaptive_applied,
        "fallback_to_nominal_reason": fallback_reason,
        "quality_causal": causal,
        "quality_fresh_for_observation": fresh,
        "quality_observation_invalid_reason": 0 if causal and fresh and quality_valid else 3,
        "quality_fusion_gate_enabled": mode == 3 and identity_match,
        "selector_quality_enabled": mode == 3,
        "fuse_enabled": True,
    }


class BootstrapTests(unittest.TestCase):
    def test_preflight_bootstrap_succeeds_after_warmup(self):
        rows = [selector_row(1_000_000, False), selector_row(1_020_000, False), selector_row(1_040_000, True)]
        result = PREFLIGHT.analyze_identity_bootstrap(rows, 42, 0, 100_000)
        self.assertTrue(result["completed"])
        self.assertTrue(result["within_timeout"])
        self.assertEqual(result["bootstrap_sample_count"], 2)
        self.assertEqual(result["bootstrap_duration_us"], 40_000)
        self.assertEqual(result["first_matched_timestamp"], 1_040_000)
        self.assertEqual(result["post_bootstrap_mismatch_count"], 0)

    def test_bootstrap_timeout_fails(self):
        rows = [selector_row(1_000_000, False), selector_row(1_200_000, True)]
        result = EVIDENCE.analyze_bootstrap(rows, 42, 0, 100_000, None)
        self.assertTrue(result["completed"])
        self.assertFalse(result["within_timeout"])

    def test_bootstrap_never_matches(self):
        result = EVIDENCE.analyze_bootstrap(
            [selector_row(1_000_000, False), selector_row(1_020_000, False)], 42, 0, 100_000, None)
        self.assertFalse(result["completed"])
        self.assertIsNone(result["first_matched_timestamp"])

    def test_post_bootstrap_mismatch_fails(self):
        rows = [selector_row(1_000_000, False), selector_row(1_020_000, True), selector_row(1_040_000, False)]
        result = EVIDENCE.analyze_bootstrap(rows, 42, 0, 100_000, None)
        self.assertEqual(result["post_bootstrap_mismatch_count"], 1)

    def test_bootstrap_may_not_overlap_formal_interval(self):
        rows = [selector_row(1_000_000, False), selector_row(1_040_000, True)]
        result = EVIDENCE.analyze_bootstrap(rows, 42, 0, 100_000, 1_020_000)
        self.assertTrue(result["bootstrap_overlaps_valid_interval"])


class JoinTests(unittest.TestCase):
    def test_selector_join_is_exact_publication_timestamp(self):
        summary = {"checks": [], "failures": []}
        csv_tables = {}
        selectors = [selector_row(1_000_000, True), selector_row(1_020_000, True)]
        validated = [
            {"timestamp": 1_000_000, "airspeed_source": 1},
            {"timestamp": 1_020_001, "airspeed_source": 1},
        ]
        EVIDENCE.analyze_selector_join(summary, selectors, validated, 1_000_000, 0.99, csv_tables)
        self.assertEqual(summary["selector_validated_join"]["matched_count"], 1)
        self.assertEqual(summary["selector_validated_join"]["unmatched_timestamps"], [1_020_000])
        self.assertIn("selector_validated.high_match_rate", summary["failures"])

    def test_r_join_uses_buffer_not_original_or_publication_timestamp(self):
        summary = {"checks": [], "failures": []}
        csv_tables = {}
        diagnostics = [{"_multi_id": 0, "timestamp": 2_000_000, "timestamp_sample": 1_500_000,
                        "ekf_buffer_timestamp_sample": 1_380_000, "r_as_used": 4.0}]
        aid = [{"_multi_id": 0, "timestamp": 9_000_000, "timestamp_sample": 1_380_000,
                "observation_variance": 4.0}]
        EVIDENCE.analyze_r_join(summary, diagnostics, aid, 1.0, 1e-6, 1e-6, csv_tables)
        self.assertEqual(summary["r_join"]["matched_count"], 1)
        self.assertIn("ekf_buffer_timestamp_sample", summary["r_join"]["join_key"])
        self.assertFalse(summary["r_join"]["publication_timestamp_used_as_join_key"])
        self.assertEqual(summary["r_join"]["absolute_error"]["max"], 0.0)
        self.assertFalse(summary["failures"])

    def test_r_join_rejects_aid_matching_original_instead_of_buffer_timestamp(self):
        summary = {"checks": [], "failures": []}
        csv_tables = {}
        diagnostics = [{"_multi_id": 0, "timestamp": 2_000_000, "timestamp_sample": 1_500_000,
                        "ekf_buffer_timestamp_sample": 1_380_000, "r_as_used": 4.0}]
        aid = [{"_multi_id": 0, "timestamp": 9_000_000, "timestamp_sample": 1_500_000,
                "observation_variance": 4.0}]
        EVIDENCE.analyze_r_join(summary, diagnostics, aid, 1.0, 1e-6, 1e-6, csv_tables)
        self.assertEqual(summary["r_join"]["matched_count"], 0)
        self.assertIn("r_join.high_match_rate", summary["failures"])

    def test_r_join_reports_error_statistics_and_unmatched(self):
        summary = {"checks": [], "failures": []}
        csv_tables = {}
        diagnostics = [
            {"_multi_id": 0, "timestamp": 2_000_000, "timestamp_sample": 1_500_000,
             "ekf_buffer_timestamp_sample": 1_380_000, "r_as_used": 4.0},
            {"_multi_id": 0, "timestamp": 2_020_000, "timestamp_sample": 1_520_000,
             "ekf_buffer_timestamp_sample": 1_400_000, "r_as_used": 5.0},
        ]
        aid = [{"_multi_id": 0, "timestamp": 2_500_000, "timestamp_sample": 1_380_000,
                "observation_variance": 4.5}]
        EVIDENCE.analyze_r_join(summary, diagnostics, aid, 0.99, 1e-6, 1e-6, csv_tables)
        self.assertEqual(summary["r_join"]["unmatched_count"], 1)
        self.assertAlmostEqual(summary["r_join"]["absolute_error"]["max"], 0.5)
        self.assertIn("r_join.high_match_rate", summary["failures"])
        self.assertIn("r_join.value_equality", summary["failures"])


class TraceabilityTests(unittest.TestCase):
    def test_all_final_sources_map_to_required_fallback_outcome(self):
        rows = []
        for index, source in enumerate([-1, 0, 1, 2, 3, 4]):
            row = selector_row(1_000_000 + index * 20_000, True, final_source=source)
            row["fallback_attempted"] = True
            row["fallback_source"] = source
            row["fallback_available"] = source != -1
            row["fallback_outcome"] = EVIDENCE.FALLBACK_OUTCOMES[source]
            row["trigger_reason"] = 3
            rows.append(row)
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_traceability(summary, rows, 1_000_000, {})
        self.assertEqual(summary["selector_traceability"]["fallback_mapping_error_count"], 0)

    def test_fallback_mismatch_is_rejected(self):
        row = selector_row(1_000_000, True, final_source=4)
        row.update({
            "fallback_attempted": True,
            "fallback_available": True,
            "fallback_source": 4,
            "fallback_outcome": 2,
            "trigger_reason": 3,
        })
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_traceability(summary, [row], 1_000_000, {})
        self.assertIn("selector.fallback_outcome_matches_final_source", summary["failures"])

    def test_quality_and_blockage_concurrency_keeps_final_source_outcome(self):
        row = selector_row(1_000_000, True, final_source=4)
        row.update({
            "quality_rejected": True,
            "concurrent_blockage": True,
            "trigger_reason": 3,
            "fallback_attempted": True,
            "fallback_available": True,
            "fallback_source": 4,
            "fallback_outcome": 3,
        })
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_traceability(summary, [row], 1_000_000, {})
        self.assertNotIn("selector.fallback_outcome_matches_final_source", summary["failures"])
        self.assertNotIn("selector.trigger_consistency", summary["failures"])

    def test_formal_baseline_rejects_blockage_state_or_trigger(self):
        row = selector_row(1_000_000, True)
        row.update({"concurrent_blockage": True, "trigger_reason": EVIDENCE.TRIGGER_BLOCKAGE})
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_traceability(summary, [row], 1_000_000, {}, formal_blockage_enabled=False)
        self.assertIn("selector.formal_blockage_disabled", summary["failures"])


class ObservationAlignmentTests(unittest.TestCase):
    def test_quality_age_uses_original_observation_timestamp(self):
        diagnostic = diagnostic_row(observation_timestamp=1_500_000, quality_timestamp=1_400_000)
        quality = {"timestamp_sample": 1_400_000, "device_id": 42, "source_instance": 0}
        selector = selector_row(2_000_000, True)
        selector["decision_timestamp_sample"] = 1_500_000
        summary = {"checks": [], "failures": []}
        tables = {}
        EVIDENCE.analyze_ekf_join(summary, [diagnostic], [quality], [selector], 1_000_000, tables)
        self.assertEqual(tables["ekf_diagnostic_join"][0]["quality_age_recomputed_us"], 100_000)
        self.assertNotIn("ekf_diagnostic.quality_age", summary["failures"])

    def test_future_quality_is_rejected(self):
        diagnostic = diagnostic_row(quality_timestamp=1_520_000, causal=False, fresh=False,
                                    adaptive_applied=False, fallback_reason=4)
        quality = {"timestamp_sample": 1_520_000, "device_id": 42, "source_instance": 0}
        selector = selector_row(2_000_000, True)
        selector["decision_timestamp_sample"] = 1_500_000
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_ekf_join(summary, [diagnostic], [quality], [selector], 1_000_000, {})
        self.assertIn("ekf_diagnostic.no_future_or_noncausal_quality", summary["failures"])

    def test_stale_adaptive_sample_fails_closed(self):
        diagnostic = diagnostic_row(quality_timestamp=1_000_000, fresh=False,
                                    adaptive_applied=True, fallback_reason=0)
        diagnostic["quality_age_us"] = 500_000
        quality = {"timestamp_sample": 1_000_000, "device_id": 42, "source_instance": 0}
        selector = selector_row(2_000_000, True)
        selector["decision_timestamp_sample"] = 1_500_000
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_ekf_join(summary, [diagnostic], [quality], [selector], 1_000_000, {})
        self.assertIn("ekf_diagnostic.no_stale_adaptive_r", summary["failures"])


class FailClosedTests(unittest.TestCase):
    def test_missing_topics_and_fields_fail_closed(self):
        summary, _ = EVIDENCE.analyze_evidence(
            {"airspeed_quality_input": [{"timestamp": 1}]},
            {"EKF2_ASP_MODE": 0},
            0,
            {"parameters_exact": {"EKF2_ASP_MODE": 0}},
        )
        self.assertEqual(summary["result"], "FAIL")
        self.assertTrue(any(name.startswith("topic.") for name in summary["failures"]))

    def test_mode_truth_table_baseline(self):
        rows = [{
            "experiment_mode": 0,
            "adaptive_r_enabled": False,
            "adaptive_r_requested": False,
            "adaptive_r_applied": False,
            "quality_fusion_gate_enabled": False,
            "selector_quality_enabled": False,
            "r_as_used": 4.0,
            "nominal_r_as": 4.0,
            "airspeed_q": 0.2,
        }]
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, rows, 0, {}, 1e-6, 1e-6)
        self.assertFalse(summary["failures"])

    def test_variance_only_requires_observed_q_and_r_variation(self):
        rows = [
            diagnostic_row(observation_timestamp=1_500_000, quality=0.8),
            diagnostic_row(observation_timestamp=1_520_000, buffer_timestamp=1_400_000,
                           quality_timestamp=1_500_000, quality=0.3),
        ]
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, rows, 2, {"EKF2_ASP_RMAX": 5.0}, 1e-6, 1e-6)
        self.assertFalse(summary["failures"])

    def test_stale_variance_only_uses_nominal_r(self):
        rows = [
            diagnostic_row(observation_timestamp=1_500_000, quality=0.8),
            diagnostic_row(observation_timestamp=1_520_000, buffer_timestamp=1_400_000,
                           quality_timestamp=1_500_000, quality=0.3),
            diagnostic_row(observation_timestamp=1_800_000, buffer_timestamp=1_680_000,
                           quality_timestamp=1_500_000, quality=0.3, fresh=False,
                           adaptive_applied=False, fallback_reason=5),
        ]
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, rows, 2, {"EKF2_ASP_RMAX": 5.0}, 1e-6, 1e-6)
        self.assertFalse(summary["failures"])

    def test_stale_variance_only_adaptive_r_is_rejected(self):
        rows = [
            diagnostic_row(observation_timestamp=1_500_000, quality=0.8),
            diagnostic_row(observation_timestamp=1_520_000, buffer_timestamp=1_400_000,
                           quality_timestamp=1_500_000, quality=0.3),
            diagnostic_row(observation_timestamp=1_800_000, buffer_timestamp=1_680_000,
                           quality_timestamp=1_500_000, quality=0.3, fresh=False,
                           adaptive_applied=True, fallback_reason=0),
        ]
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, rows, 2, {"EKF2_ASP_RMAX": 5.0}, 1e-6, 1e-6)
        self.assertIn("mode.path_flags", summary["failures"])
        self.assertIn("mode.r_behavior", summary["failures"])

    def test_constant_r_only_applies_to_matching_sensor_one(self):
        matching = diagnostic_row(mode=1, adaptive_requested=False, adaptive_applied=False)
        matching["r_as_used"] = 8.0
        synthetic = diagnostic_row(mode=1, source=4, device_id=0, identity_match=False,
                                   adaptive_requested=False, adaptive_applied=False, fallback_reason=2)
        synthetic["r_as_used"] = synthetic["nominal_r_as"]
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, [matching, synthetic], 1, {"EKF2_ASP_RCST": 2.0}, 1e-6, 1e-6)
        self.assertFalse(summary["failures"])

    def test_constant_r_on_synthetic_is_rejected(self):
        synthetic = diagnostic_row(mode=1, source=4, device_id=0, identity_match=False,
                                   adaptive_requested=False, adaptive_applied=False, fallback_reason=2)
        synthetic["r_as_used"] = 8.0
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, [synthetic], 1, {"EKF2_ASP_RCST": 2.0}, 1e-6, 1e-6)
        self.assertIn("mode.r_behavior", summary["failures"])

    def test_formal_parameters_require_custom_blockage_disabled(self):
        summary = {"checks": [], "failures": []}
        config = {"parameters_exact": {"ASPD_QBLK_EN": 0}}
        EVIDENCE.validate_parameters(summary, {"ASPD_QBLK_EN": 1}, config, 0)
        self.assertIn("parameters.formal_blockage_disabled", summary["failures"])

    def test_fail_closed_output_always_contains_json_and_csv(self):
        summary = {"result": "FAIL", "checks": [], "failures": ["missing_topic"]}
        with tempfile.TemporaryDirectory() as directory:
            EVIDENCE.write_outputs(directory, summary, {})
            output = pathlib.Path(directory)
            self.assertTrue((output / "airspeed_evidence_summary.json").is_file())
            self.assertTrue((output / "checks.csv").is_file())
            self.assertTrue((output / "failures.csv").is_file())


if __name__ == "__main__":
    unittest.main()
