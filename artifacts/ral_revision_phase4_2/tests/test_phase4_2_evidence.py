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


def load_qgc_parameters(path):
    parameters = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        system_id, component_id, name, value, parameter_type = line.split("\t")
        if system_id != "1" or component_id != "1":
            raise ValueError(f"unexpected target for {name}")
        parameters[name] = int(value) if parameter_type == "6" else float(value)
    return parameters


def selector_row(timestamp, match, source=1, final_source=1, quality_timestamp=None):
    if quality_timestamp is None:
        quality_timestamp = timestamp - 20_000
    return {
        "timestamp": timestamp,
        "decision_timestamp_sample": timestamp - 10_000,
        "quality_timestamp_sample": quality_timestamp,
        "quality_age_us": timestamp - quality_timestamp,
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
        "qmon": False,
        "airspeed_q": quality,
        "q_raw": quality,
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


def monitoring_row(timestamp=2_000_000, quality_timestamp=1_980_000, quality=0.8):
    return {
        "_multi_id": 0,
        "timestamp": timestamp,
        "timestamp_sample": 0,
        "ekf_buffer_timestamp_sample": 0,
        "quality_timestamp_sample": quality_timestamp,
        "quality_age_us": 0xFFFFFFFF,
        "qmon": True,
        "airspeed_q": quality,
        "q_raw": quality,
        "quality_input_valid": True,
        "quality_source_instance": 0,
        "quality_device_id": 42,
        "airspeed_source": -1,
        "airspeed_device_id": 0,
        "source_identity_match": False,
        "nominal_r_as": float("nan"),
        "r_as_used": float("nan"),
        "experiment_mode": 3,
        "adaptive_r_enabled": True,
        "adaptive_r_requested": True,
        "adaptive_r_applied": False,
        "fallback_to_nominal_reason": 0,
        "quality_causal": False,
        "quality_fresh_for_observation": False,
        "quality_observation_invalid_reason": 1,
        "quality_fusion_gate_enabled": True,
        "selector_quality_enabled": True,
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


class ExperimentParameterTests(unittest.TestCase):
    PRESET_FILES = {
        "A": "A_baseline.params",
        "B": "B_constant_r.params",
        "C": "C_variance_only.params",
        "D": "D_full.params",
        "E": "E_no_pitot.params",
    }

    def test_all_qgc_presets_exactly_match_preflight_contract(self):
        preset_directory = ROOT / "artifacts/ral_revision_phase4_2/parameter_presets"
        for mode, filename in self.PRESET_FILES.items():
            with self.subTest(mode=mode):
                actual = load_qgc_parameters(preset_directory / filename)
                expected = PREFLIGHT.experiment_mode_expectations(mode)
                self.assertEqual(set(actual), set(expected))
                for name, value in expected.items():
                    self.assertTrue(PREFLIGHT.close_enough(actual[name], value), name)

    def test_a_through_d_restore_pitot_use(self):
        for mode in "ABCD":
            with self.subTest(mode=mode):
                expected = PREFLIGHT.experiment_mode_expectations(mode)
                self.assertEqual(expected["EKF2_ARSP_THR"], 8.0)
                self.assertEqual(expected["FW_USE_AIRSPD"], 1)
                self.assertEqual(expected["SYS_HAS_NUM_ASPD"], 1)

    def test_e_disables_use_but_keeps_physical_pitot_required(self):
        expected = PREFLIGHT.experiment_mode_expectations("E")
        self.assertEqual(expected["EKF2_ASP_MODE"], 0)
        self.assertEqual(expected["EKF2_ARSP_THR"], 0.0)
        self.assertEqual(expected["FW_USE_AIRSPD"], 0)
        self.assertEqual(expected["SYS_HAS_NUM_ASPD"], 1)

    def test_changing_only_mode_after_e_fails_full_contract(self):
        actual = PREFLIGHT.experiment_mode_expectations("E")
        actual["EKF2_ASP_MODE"] = 3
        expected = PREFLIGHT.experiment_mode_expectations("D")
        mismatches = {name for name, value in expected.items()
                      if not PREFLIGHT.close_enough(actual[name], value)}
        self.assertEqual(mismatches, {"EKF2_ARSP_THR", "FW_USE_AIRSPD"})

    def test_required_evidence_topics_are_mode_aware(self):
        for mode in "ABCD":
            with self.subTest(mode=mode):
                self.assertIn("estimator_aid_src_airspeed", PREFLIGHT.required_evidence_topics(mode))

        mode_e_topics = PREFLIGHT.required_evidence_topics("E")
        self.assertNotIn("estimator_aid_src_airspeed", mode_e_topics)
        self.assertIn("airspeed_quality_input", mode_e_topics)
        self.assertIn("ekf2_airspeed_quality", mode_e_topics)
        self.assertIn("airspeed_selector_quality_status", mode_e_topics)


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

    def test_monitoring_record_is_excluded_from_observation_and_r_joins(self):
        summary = {"checks": [], "failures": []}
        csv_tables = {}
        diagnostic = diagnostic_row(mode=3)
        monitoring = monitoring_row()
        quality = {"timestamp_sample": 1_480_000, "device_id": 42, "source_instance": 0}
        selector = selector_row(2_000_000, True)
        selector["decision_timestamp_sample"] = 1_500_000
        EVIDENCE.analyze_ekf_join(summary, [diagnostic, monitoring], [quality], [selector], 1_000_000, csv_tables)
        self.assertEqual(summary["ekf_diagnostic_join"]["diagnostic_count"], 1)

        aid = [{"_multi_id": 0, "timestamp": 2_100_000, "timestamp_sample": 1_380_000,
                "observation_variance": diagnostic["r_as_used"]}]
        EVIDENCE.analyze_r_join(summary, [diagnostic, monitoring], aid, 1.0, 1e-6, 1e-6, csv_tables)
        self.assertEqual(summary["r_join"]["diagnostic_count"], 1)
        self.assertNotIn(monitoring, EVIDENCE.observation_records([diagnostic, monitoring]))
        self.assertFalse(summary["failures"])

    def test_monitoring_record_contract_accepts_non_applied_adaptive_r(self):
        summary = {"checks": [], "failures": []}
        monitoring = monitoring_row()
        quality = {"timestamp_sample": monitoring["quality_timestamp_sample"]}
        EVIDENCE.analyze_monitoring_records(summary, [monitoring], [quality], 3, {})
        self.assertEqual(summary["ekf_monitoring"]["record_count"], 1)
        self.assertFalse(summary["failures"])

    def test_monitoring_record_is_excluded_from_observation_mode_assertions(self):
        rows = [
            diagnostic_row(mode=3, quality=0.8),
            diagnostic_row(observation_timestamp=1_520_000, buffer_timestamp=1_400_000,
                           mode=3, quality_timestamp=1_500_000, quality=0.3),
            monitoring_row(),
        ]
        summary = {"checks": [], "failures": []}
        EVIDENCE.analyze_mode(summary, rows, 3, {"EKF2_ASP_RMAX": 5.0}, 1e-6, 1e-6)
        self.assertFalse(summary["failures"])

    def test_monitoring_record_outside_full_mode_is_rejected(self):
        summary = {"checks": [], "failures": []}
        monitoring = monitoring_row()
        monitoring["experiment_mode"] = 2
        quality = {"timestamp_sample": monitoring["quality_timestamp_sample"]}
        EVIDENCE.analyze_monitoring_records(summary, [monitoring], [quality], 2, {})
        self.assertIn("ekf_monitoring.full_mode_scope", summary["failures"])

    def test_selector_closed_unified_gate_requires_quality_rejection(self):
        summary = {"checks": [], "failures": []}
        diagnostic = diagnostic_row(mode=3, quality_timestamp=1_980_000)
        diagnostic["fuse_enabled"] = False
        selector = selector_row(2_000_000, True, final_source=-1, quality_timestamp=1_980_000)
        selector["quality_rejected"] = True
        EVIDENCE.analyze_selector_unified_gate(summary, [selector], [diagnostic], 3, 1_000_000, {})
        self.assertEqual(summary["selector_unified_gate"]["checked_count"], 1)
        self.assertFalse(summary["failures"])

    def test_selector_rejection_with_open_unified_gate_is_rejected(self):
        summary = {"checks": [], "failures": []}
        diagnostic = diagnostic_row(mode=3, quality_timestamp=1_980_000)
        selector = selector_row(2_000_000, True, final_source=-1, quality_timestamp=1_980_000)
        selector["quality_rejected"] = True
        EVIDENCE.analyze_selector_unified_gate(summary, [selector], [diagnostic], 3, 1_000_000, {})
        self.assertIn("selector.mirrors_unified_gate", summary["failures"])

    def test_selector_open_with_closed_unified_gate_is_rejected(self):
        summary = {"checks": [], "failures": []}
        diagnostic = diagnostic_row(mode=3, quality_timestamp=1_980_000)
        diagnostic["fuse_enabled"] = False
        selector = selector_row(2_000_000, True, quality_timestamp=1_980_000)
        EVIDENCE.analyze_selector_unified_gate(summary, [selector], [diagnostic], 3, 1_000_000, {})
        self.assertIn("selector.mirrors_unified_gate", summary["failures"])

    def test_selector_native_blockage_is_excluded_from_unified_gate_assertion(self):
        summary = {"checks": [], "failures": []}
        diagnostic = diagnostic_row(mode=3, quality_timestamp=1_980_000)
        diagnostic["fuse_enabled"] = False
        selector = selector_row(2_000_000, True, quality_timestamp=1_980_000)
        selector["concurrent_blockage"] = True
        EVIDENCE.analyze_selector_unified_gate(summary, [selector], [diagnostic], 3, 1_000_000, {})
        self.assertEqual(summary["selector_unified_gate"]["checked_count"], 0)
        self.assertNotIn("selector.mirrors_unified_gate", summary["failures"])

    def test_selector_gate_state_can_join_to_qmon_by_quality_timestamp(self):
        summary = {"checks": [], "failures": []}
        monitoring = monitoring_row(timestamp=1_990_000, quality_timestamp=1_980_000)
        monitoring["fuse_enabled"] = False
        selector = selector_row(2_000_000, True, final_source=-1, quality_timestamp=1_980_000)
        selector["quality_rejected"] = True
        EVIDENCE.analyze_selector_unified_gate(summary, [selector], [monitoring], 3, 1_000_000, {})
        self.assertEqual(summary["selector_unified_gate"]["qmon_join_count"], 1)
        self.assertFalse(summary["failures"])


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
