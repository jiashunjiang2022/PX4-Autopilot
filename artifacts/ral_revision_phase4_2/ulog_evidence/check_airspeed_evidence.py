#!/usr/bin/env python3
"""Fail-closed Phase 4.3 airspeed sample/source/R evidence checker."""

import argparse
import csv
import json
import math
import os
import statistics
import sys
from collections import defaultdict


MODE_NAMES = {
    0: "BASELINE",
    1: "CONSTANT_R",
    2: "VARIANCE_ONLY",
    3: "FULL_PROPOSED",
}
MODE_VALUES = {name: value for value, name in MODE_NAMES.items()}
PHYSICAL_SOURCES = {1, 2, 3}
SOURCE_DISABLED = -1
SOURCE_GROUND_MINUS_WIND = 0
SOURCE_SYNTHETIC = 4
FALLBACK_OUTCOMES = {
    SOURCE_DISABLED: 4,
    SOURCE_GROUND_MINUS_WIND: 2,
    1: 1,
    2: 1,
    3: 1,
    SOURCE_SYNTHETIC: 3,
}
TRIGGER_NONE = 0
TRIGGER_BLOCKAGE = 4
GRID_INTERVAL_US = 20_000
FALLBACK_TO_NOMINAL_REASON_NONE = 0

REQUIRED_FIELDS = {
    "airspeed_quality_input": {
        "timestamp", "timestamp_sample", "device_id", "source_instance",
        "differential_pressure_pa", "indicated_airspeed_m_s", "input_source",
        "valid", "resampled", "measured_source_rate_hz", "rate_valid",
        "filter_source_rate_hz", "filter_cutoff_hz", "output_rate_hz",
        "gap_count", "rate_reset_counter", "reset_reason",
    },
    "airspeed_selector_quality_status": {
        "timestamp", "decision_timestamp_sample", "quality_timestamp_sample",
        "pre_quality_device_id", "quality_device_id", "final_device_id",
        "experiment_mode", "selector_quality_enabled", "pre_quality_source",
        "original_selected_source", "quality_source_instance",
        "source_identity_match", "pre_quality_output_finite",
        "original_sensor_valid", "original_selection_was_fallback",
        "concurrent_original_sensor_invalid", "concurrent_blockage",
        "quality_rejected", "trigger_reason", "fallback_outcome",
        "fallback_attempted", "fallback_available", "fallback_source",
        "final_selected_source", "final_valid",
    },
    "airspeed_validated": {"timestamp", "airspeed_source"},
    "ekf2_airspeed_quality": {
        "timestamp", "timestamp_sample", "ekf_buffer_timestamp_sample", "quality_timestamp_sample",
        "quality_age_us", "qmon", "airspeed_q", "q_raw", "quality_input_valid",
        "quality_source_instance", "quality_device_id", "airspeed_source",
        "airspeed_device_id", "source_identity_match", "nominal_r_as",
        "r_as_used", "experiment_mode", "adaptive_r_enabled", "adaptive_r_requested",
        "adaptive_r_applied", "fallback_to_nominal_reason", "quality_causal",
        "quality_fresh_for_observation", "quality_observation_invalid_reason",
        "quality_fusion_gate_enabled", "selector_quality_enabled",
        "fuse_enabled",
    },
    "estimator_aid_src_airspeed": {
        "timestamp", "timestamp_sample", "device_id", "observation_variance",
    },
    "logger_status": {"timestamp", "dropouts", "message_gaps"},
    "differential_pressure": {"timestamp", "timestamp_sample", "device_id"},
}


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def close_enough(actual, expected, absolute_tolerance, relative_tolerance):
    return finite(actual) and finite(expected) and math.isclose(
        float(actual), float(expected), rel_tol=relative_tolerance, abs_tol=absolute_tolerance)


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(percentage * len(ordered)) - 1))
    return ordered[index]


def json_value(value):
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [json_value(item) for item in value]
    if hasattr(value, "item"):
        return json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def sorted_rows(rows, key="timestamp"):
    return sorted(rows, key=lambda row: (int(row.get(key, 0)), int(row.get("_multi_id", 0))))


def is_quality_only_monitoring(row):
    return bool(row.get("qmon", False))


def observation_records(rows):
    return [row for row in rows if not is_quality_only_monitoring(row)]


def monitoring_records(rows):
    return [row for row in rows if is_quality_only_monitoring(row)]


def topic_rate(rows, sample_field="timestamp"):
    timestamps = sorted({int(row.get(sample_field, 0)) for row in rows if int(row.get(sample_field, 0)) > 0})
    deltas = [current - previous for previous, current in zip(timestamps, timestamps[1:]) if current > previous]
    duration_us = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
    return {
        "sample_count": len(rows),
        "unique_timestamp_count": len(timestamps),
        "duration_us": duration_us,
        "measured_rate_hz": ((len(timestamps) - 1) * 1e6 / duration_us) if duration_us > 0 else None,
        "mean_gap_us": statistics.fmean(deltas) if deltas else None,
        "max_gap_us": max(deltas) if deltas else None,
        "non_monotonic_or_duplicate_count": max(0, len(rows) - len(timestamps)),
    }


def add_check(summary, name, passed, detail):
    summary["checks"].append({"name": name, "passed": bool(passed), "detail": json_value(detail)})
    if not passed:
        summary["failures"].append(name)
    return bool(passed)


def require_fields(summary, topic, rows):
    if not rows:
        add_check(summary, f"topic.{topic}.present", False, {"rows": 0})
        return False
    observed = set().union(*(row.keys() for row in rows))
    missing = sorted(REQUIRED_FIELDS[topic] - observed)
    return add_check(summary, f"topic.{topic}.schema", not missing,
                     {"rows": len(rows), "missing_fields": missing})


def expected_identity(tables):
    rows = tables.get("differential_pressure", [])
    instances = {int(row.get("_multi_id", 0)) for row in rows}
    devices = {int(row.get("device_id", 0)) for row in rows if int(row.get("device_id", 0)) != 0}
    zero_seen = any(int(row.get("device_id", 0)) == 0 for row in rows)
    return {
        "device_id": next(iter(devices), 0) if len(devices) == 1 else 0,
        "source_instance": next(iter(instances), -1) if len(instances) == 1 else -1,
        "devices": sorted(devices),
        "instances": sorted(instances),
        "zero_device_seen": zero_seen,
        "valid": len(devices) == 1 and len(instances) == 1 and not zero_seen,
    }


def identity_match(row, device_id, source_instance):
    return (int(row.get("pre_quality_source", -1)) in PHYSICAL_SOURCES
            and int(row.get("pre_quality_device_id", 0)) == device_id
            and int(row.get("quality_device_id", 0)) == device_id
            and device_id != 0
            and int(row.get("quality_source_instance", 255)) == source_instance
            and bool(row.get("source_identity_match", False)))


def analyze_bootstrap(selector_rows, device_id, source_instance, timeout_us, valid_start_us):
    rows = sorted_rows(selector_rows)
    output = {
        "completed": False,
        "within_timeout": False,
        "bootstrap_sample_count": len(rows),
        "bootstrap_duration_us": 0,
        "first_matched_timestamp": None,
        "post_bootstrap_physical_sample_count": 0,
        "post_bootstrap_mismatch_count": 0,
        "post_bootstrap_mismatches": [],
        "bootstrap_overlaps_valid_interval": False,
    }
    if not rows:
        return output
    first_timestamp = int(rows[0].get("timestamp", 0))
    for index, row in enumerate(rows):
        if identity_match(row, device_id, source_instance):
            first_match = int(row.get("timestamp", 0))
            output.update({
                "completed": True,
                "bootstrap_sample_count": index,
                "bootstrap_duration_us": max(0, first_match - first_timestamp),
                "first_matched_timestamp": first_match,
                "bootstrap_overlaps_valid_interval": valid_start_us is not None and first_match > valid_start_us,
            })
            output["within_timeout"] = output["bootstrap_duration_us"] <= timeout_us
            for later in rows[index:]:
                if int(later.get("pre_quality_source", -1)) in PHYSICAL_SOURCES:
                    output["post_bootstrap_physical_sample_count"] += 1
                    if not identity_match(later, device_id, source_instance):
                        output["post_bootstrap_mismatch_count"] += 1
                        output["post_bootstrap_mismatches"].append({
                            "timestamp": int(later.get("timestamp", 0)),
                            "pre_quality_source": int(later.get("pre_quality_source", -1)),
                            "pre_quality_device_id": int(later.get("pre_quality_device_id", 0)),
                            "quality_device_id": int(later.get("quality_device_id", 0)),
                            "quality_source_instance": int(later.get("quality_source_instance", 255)),
                            "source_identity_match": bool(later.get("source_identity_match", False)),
                        })
            return output
    output["bootstrap_duration_us"] = max(0, int(rows[-1].get("timestamp", 0)) - first_timestamp)
    return output


def first_armed_timestamp(rows):
    armed = [int(row.get("timestamp", 0)) for row in rows if int(row.get("arming_state", -1)) == 2]
    return min(armed) if armed else None


def analyze_quality_input(summary, rows, bootstrap_timestamp, identity, csv_tables):
    rows = sorted_rows(rows, "timestamp_sample")
    post = [row for row in rows if bootstrap_timestamp is not None
            and int(row.get("timestamp", 0)) >= bootstrap_timestamp]
    rate = topic_rate(post, "timestamp_sample")
    deltas = [int(current.get("timestamp_sample", 0)) - int(previous.get("timestamp_sample", 0))
              for previous, current in zip(post, post[1:])]
    grid_errors = [delta for delta in deltas if delta != GRID_INTERVAL_US]
    invalid_rows = [row for row in post if not bool(row.get("valid", False)) or not bool(row.get("rate_valid", False))]
    identity_errors = [row for row in post
                       if int(row.get("device_id", 0)) != identity["device_id"]
                       or int(row.get("source_instance", 255)) != identity["source_instance"]]
    gap_increases = []
    reset_increases = []
    for previous, current in zip(post, post[1:]):
        if int(current.get("gap_count", 0)) != int(previous.get("gap_count", 0)):
            gap_increases.append(int(current.get("timestamp_sample", 0)))
        if int(current.get("rate_reset_counter", 0)) != int(previous.get("rate_reset_counter", 0)):
            reset_increases.append(int(current.get("timestamp_sample", 0)))
    unexpected_reset = [row for row in post[1:] if int(row.get("reset_reason", 0)) != 0]
    pressures = [float(row["differential_pressure_pa"]) for row in post
                 if finite(row.get("differential_pressure_pa"))]
    signed = {
        "finite_count": len(pressures),
        "negative_count": sum(value < 0 for value in pressures),
        "zero_count": sum(value == 0 for value in pressures),
        "positive_count": sum(value > 0 for value in pressures),
        "minimum_pa": min(pressures) if pressures else None,
        "maximum_pa": max(pressures) if pressures else None,
        "zero_crossing_count": sum((a < 0 < b) or (b < 0 < a) for a, b in zip(pressures, pressures[1:])),
    }
    summary["quality_input"] = {
        **rate,
        "expected_grid_us": GRID_INTERVAL_US,
        "grid_error_count": len(grid_errors),
        "invalid_or_rate_invalid_count": len(invalid_rows),
        "held_sample_count": rate["non_monotonic_or_duplicate_count"],
        "identity_error_count": len(identity_errors),
        "gap_counter_change_count": len(gap_increases),
        "rate_reset_counter_change_count": len(reset_increases),
        "unexpected_reset_reason_count": len(unexpected_reset),
        "signed_pressure": signed,
    }
    add_check(summary, "quality_input.post_bootstrap_samples", len(post) >= 2, {"rows": len(post)})
    add_check(summary, "quality_input.20ms_grid", not grid_errors,
              {"error_count": len(grid_errors), "examples_us": grid_errors[:20]})
    add_check(summary, "quality_input.no_held_sample", rate["non_monotonic_or_duplicate_count"] == 0,
              {"held_sample_count": rate["non_monotonic_or_duplicate_count"]})
    add_check(summary, "quality_input.valid", not invalid_rows, {"invalid_count": len(invalid_rows)})
    add_check(summary, "quality_input.identity_stable", not identity_errors,
              {"error_count": len(identity_errors)})
    add_check(summary, "quality_input.no_post_bootstrap_reset_or_gap",
              not gap_increases and not reset_increases and not unexpected_reset,
              {"gap_changes": gap_increases[:20], "rate_reset_changes": reset_increases[:20],
               "reset_reason_timestamps": [int(row.get("timestamp_sample", 0)) for row in unexpected_reset[:20]]})
    add_check(summary, "quality_input.signed_pressure_logged", len(pressures) == len(post), signed)
    csv_tables["quality_input"] = post


def exact_join(left_rows, right_rows, left_field, right_field=None, include_multi_id=False):
    right_field = right_field or left_field
    right_by_key = defaultdict(list)
    for row in right_rows:
        key = ((int(row.get("_multi_id", 0)), int(row.get(right_field, 0))) if include_multi_id
               else int(row.get(right_field, 0)))
        right_by_key[key].append(row)
    joined = []
    unmatched = []
    for row in left_rows:
        key = ((int(row.get("_multi_id", 0)), int(row.get(left_field, 0))) if include_multi_id
               else int(row.get(left_field, 0)))
        matches = right_by_key.get(key, [])
        if matches:
            joined.append((row, matches[0]))
        else:
            unmatched.append(row)
    return joined, unmatched


def analyze_selector_join(summary, selector_rows, validated_rows, bootstrap_timestamp, minimum_rate, csv_tables):
    post = [row for row in sorted_rows(selector_rows) if bootstrap_timestamp is not None
            and int(row.get("timestamp", 0)) >= bootstrap_timestamp]
    joined, unmatched = exact_join(post, validated_rows, "timestamp")
    source_errors = [(left, right) for left, right in joined
                     if int(left.get("final_selected_source", -1)) != int(right.get("airspeed_source", -1))]
    rate = len(joined) / len(post) if post else 0.0
    rows = []
    matched_ids = {id(left): right for left, right in joined}
    for left in post:
        right = matched_ids.get(id(left))
        rows.append({
            "selector_timestamp": int(left.get("timestamp", 0)),
            "matched": right is not None,
            "selector_final_source": int(left.get("final_selected_source", -1)),
            "validated_source": int(right.get("airspeed_source", -1)) if right else None,
            "source_match": right is not None and int(left.get("final_selected_source", -1)) == int(right.get("airspeed_source", -1)),
            "pre_quality_device_id": int(left.get("pre_quality_device_id", 0)),
            "final_device_id": int(left.get("final_device_id", 0)),
        })
    summary["selector_validated_join"] = {
        "post_bootstrap_selector_count": len(post),
        "matched_count": len(joined),
        "unmatched_count": len(unmatched),
        "match_rate": rate,
        "minimum_match_rate": minimum_rate,
        "source_mismatch_count": len(source_errors),
        "unmatched_timestamps": [int(row.get("timestamp", 0)) for row in unmatched],
    }
    add_check(summary, "selector_validated.high_match_rate", bool(post) and rate >= minimum_rate,
              summary["selector_validated_join"])
    add_check(summary, "selector_validated.source_consistency", not source_errors,
              {"source_mismatch_count": len(source_errors)})
    csv_tables["selector_validated_join"] = rows
    csv_tables["selector_unmatched"] = [{"timestamp": int(row.get("timestamp", 0))} for row in unmatched]


def analyze_ekf_join(summary, diagnostic_rows, quality_rows, selector_rows, bootstrap_timestamp, csv_tables):
    diagnostics = [row for row in sorted_rows(observation_records(diagnostic_rows), "timestamp_sample")
                   if bootstrap_timestamp is not None and int(row.get("timestamp", 0)) >= bootstrap_timestamp]
    quality_by_sample = {int(row.get("timestamp_sample", 0)): row for row in quality_rows}
    selector_by_sample = defaultdict(list)
    for row in selector_rows:
        selector_by_sample[int(row.get("decision_timestamp_sample", 0))].append(row)
    evidence_rows = []
    identity_errors = []
    age_errors = []
    future_quality_errors = []
    stale_adaptive_errors = []
    adaptive_flag_errors = []
    for diagnostic in diagnostics:
        observation_timestamp = int(diagnostic.get("timestamp_sample", 0))
        quality_timestamp = int(diagnostic.get("quality_timestamp_sample", 0))
        quality = quality_by_sample.get(quality_timestamp)
        selectors = selector_by_sample.get(observation_timestamp, [])
        selector = next((row for row in selectors
                         if int(row.get("final_selected_source", -1)) == int(diagnostic.get("airspeed_source", -1))),
                        selectors[0] if selectors else None)
        causal = bool(diagnostic.get("quality_causal", False))
        expected_age = observation_timestamp - quality_timestamp if causal and quality_timestamp <= observation_timestamp else None
        age_error = expected_age is None or expected_age != int(diagnostic.get("quality_age_us", -1))
        if quality is None or age_error:
            age_errors.append(diagnostic)
        future_quality = quality_timestamp > observation_timestamp or not causal
        if future_quality:
            future_quality_errors.append(diagnostic)
        adaptive_applied = bool(diagnostic.get("adaptive_r_applied", False))
        adaptive_requested = bool(diagnostic.get("adaptive_r_requested", False))
        quality_fresh = bool(diagnostic.get("quality_fresh_for_observation", False))
        quality_valid = bool(diagnostic.get("quality_input_valid", False))
        if adaptive_applied and (not causal or not quality_fresh or not quality_valid):
            stale_adaptive_errors.append(diagnostic)
        if bool(diagnostic.get("adaptive_r_enabled", False)) != adaptive_applied or (adaptive_applied and not adaptive_requested):
            adaptive_flag_errors.append(diagnostic)
        physical = int(diagnostic.get("airspeed_source", -1)) in PHYSICAL_SOURCES
        identity_ok = True
        if physical:
            identity_ok = (quality is not None
                           and int(diagnostic.get("quality_device_id", 0)) == int(quality.get("device_id", -1))
                           and int(diagnostic.get("quality_source_instance", 255)) == int(quality.get("source_instance", -1))
                           and int(diagnostic.get("airspeed_device_id", 0)) == int(quality.get("device_id", -1))
                           and bool(diagnostic.get("source_identity_match", False)))
        if not identity_ok:
            identity_errors.append(diagnostic)
        evidence_rows.append({
            "observation_timestamp_sample": observation_timestamp,
            "ekf_buffer_timestamp_sample": int(diagnostic.get("ekf_buffer_timestamp_sample", 0)),
            "diagnostic_publication_timestamp": int(diagnostic.get("timestamp", 0)),
            "quality_timestamp_sample": quality_timestamp,
            "quality_age_us": int(diagnostic.get("quality_age_us", 0)),
            "quality_age_recomputed_us": expected_age,
            "quality_causal": causal,
            "quality_fresh_for_observation": quality_fresh,
            "quality_observation_invalid_reason": int(diagnostic.get("quality_observation_invalid_reason", 0)),
            "adaptive_r_requested": adaptive_requested,
            "adaptive_r_applied": adaptive_applied,
            "fallback_to_nominal_reason": int(diagnostic.get("fallback_to_nominal_reason", 0)),
            "quality_row_matched": quality is not None,
            "selector_row_matched": selector is not None,
            "airspeed_source": int(diagnostic.get("airspeed_source", -1)),
            "airspeed_device_id": int(diagnostic.get("airspeed_device_id", 0)),
            "quality_device_id": int(diagnostic.get("quality_device_id", 0)),
            "quality_source_instance": int(diagnostic.get("quality_source_instance", 255)),
            "identity_consistent": identity_ok,
        })
    missing_quality = sum(not row["quality_row_matched"] for row in evidence_rows)
    missing_selector = sum(not row["selector_row_matched"] for row in evidence_rows)
    summary["ekf_diagnostic_join"] = {
        "diagnostic_count": len(diagnostics),
        "quality_unmatched_count": missing_quality,
        "selector_unmatched_count": missing_selector,
        "quality_age_error_count": len(age_errors),
        "future_or_noncausal_quality_count": len(future_quality_errors),
        "stale_adaptive_count": len(stale_adaptive_errors),
        "adaptive_flag_error_count": len(adaptive_flag_errors),
        "identity_error_count": len(identity_errors),
    }
    add_check(summary, "ekf_diagnostic.quality_observation_join", bool(diagnostics) and missing_quality == 0,
              summary["ekf_diagnostic_join"])
    add_check(summary, "ekf_diagnostic.selector_observation_join", missing_selector == 0,
              {"selector_unmatched_count": missing_selector})
    add_check(summary, "ekf_diagnostic.quality_age", not age_errors,
              {"error_count": len(age_errors), "semantics": "original observation timestamp minus quality timestamp"})
    add_check(summary, "ekf_diagnostic.no_future_or_noncausal_quality", not future_quality_errors,
              {"error_count": len(future_quality_errors)})
    add_check(summary, "ekf_diagnostic.no_stale_adaptive_r", not stale_adaptive_errors,
              {"error_count": len(stale_adaptive_errors)})
    add_check(summary, "ekf_diagnostic.adaptive_flags_consistent", not adaptive_flag_errors,
              {"error_count": len(adaptive_flag_errors)})
    add_check(summary, "ekf_diagnostic.source_identity", not identity_errors,
              {"error_count": len(identity_errors)})
    csv_tables["ekf_diagnostic_join"] = evidence_rows


def analyze_monitoring_records(summary, diagnostic_rows, quality_rows, expected_mode, csv_tables):
    rows = sorted_rows(monitoring_records(diagnostic_rows))
    quality_timestamps = {int(row.get("timestamp_sample", 0)) for row in quality_rows}
    scope_errors = []
    contract_errors = []
    freshness_errors = []
    evidence_rows = []

    for row in rows:
        publication_timestamp = int(row.get("timestamp", 0))
        quality_timestamp = int(row.get("quality_timestamp_sample", 0))
        quality_age_us = publication_timestamp - quality_timestamp if publication_timestamp >= quality_timestamp else None
        scope_ok = expected_mode == 3 and int(row.get("experiment_mode", -1)) == 3
        contract_ok = (int(row.get("timestamp_sample", -1)) == 0
                       and int(row.get("ekf_buffer_timestamp_sample", -1)) == 0
                       and int(row.get("airspeed_source", 0)) == SOURCE_DISABLED
                       and int(row.get("airspeed_device_id", -1)) == 0
                       and not finite(row.get("r_as_used"))
                       and not bool(row.get("adaptive_r_applied", False)))
        freshness_ok = (quality_timestamp > 0
                        and quality_timestamp in quality_timestamps
                        and quality_age_us is not None
                        and quality_age_us < 1_000_000)

        if not scope_ok:
            scope_errors.append(publication_timestamp)
        if not contract_ok:
            contract_errors.append(publication_timestamp)
        if not freshness_ok:
            freshness_errors.append(publication_timestamp)

        evidence_rows.append({
            "publication_timestamp": publication_timestamp,
            "quality_timestamp_sample": quality_timestamp,
            "quality_age_us": quality_age_us,
            "quality_input_valid": bool(row.get("quality_input_valid", False)),
            "airspeed_q": row.get("airspeed_q"),
            "q_raw": row.get("q_raw"),
            "fuse_enabled": bool(row.get("fuse_enabled", False)),
            "adaptive_r_enabled": bool(row.get("adaptive_r_enabled", False)),
            "adaptive_r_applied": bool(row.get("adaptive_r_applied", False)),
            "scope_ok": scope_ok,
            "contract_ok": contract_ok,
            "freshness_ok": freshness_ok,
        })

    summary["ekf_monitoring"] = {
        "record_count": len(rows),
        "scope_error_count": len(scope_errors),
        "contract_error_count": len(contract_errors),
        "freshness_error_count": len(freshness_errors),
    }
    add_check(summary, "ekf_monitoring.full_mode_scope", not scope_errors,
              {"error_count": len(scope_errors), "timestamps": scope_errors[:20]})
    add_check(summary, "ekf_monitoring.record_contract", not contract_errors,
              {"error_count": len(contract_errors), "timestamps": contract_errors[:20]})
    add_check(summary, "ekf_monitoring.quality_freshness", not freshness_errors,
              {"error_count": len(freshness_errors), "timestamps": freshness_errors[:20]})
    csv_tables["ekf_monitoring"] = evidence_rows


def analyze_selector_unified_gate(summary, selector_rows, diagnostic_rows, expected_mode,
                                  bootstrap_timestamp, csv_tables):
    diagnostics_by_quality_sample = defaultdict(list)
    for row in diagnostic_rows:
        if int(row.get("_multi_id", 0)) == 0:
            diagnostics_by_quality_sample[int(row.get("quality_timestamp_sample", 0))].append(row)

    for rows in diagnostics_by_quality_sample.values():
        rows.sort(key=lambda row: int(row.get("timestamp", 0)))

    checked_rows = []
    unmatched = []
    contract_errors = []
    state_errors = []

    for selector in sorted_rows(selector_rows):
        selector_timestamp = int(selector.get("timestamp", 0))
        quality_timestamp = int(selector.get("quality_timestamp_sample", 0))
        selector_scope = (expected_mode == 3
                          and int(selector.get("experiment_mode", -1)) == 3
                          and bool(selector.get("selector_quality_enabled", False))
                          and int(selector.get("pre_quality_source", SOURCE_DISABLED)) in PHYSICAL_SOURCES
                          and bool(selector.get("original_sensor_valid", False))
                          and bool(selector.get("pre_quality_output_finite", False))
                          and not bool(selector.get("original_selection_was_fallback", False))
                          and not bool(selector.get("concurrent_original_sensor_invalid", False))
                          and not bool(selector.get("concurrent_blockage", False))
                          and bool(selector.get("source_identity_match", False))
                          and quality_timestamp > 0
                          and int(selector.get("quality_age_us", 0xFFFFFFFF)) < 1_000_000
                          and bootstrap_timestamp is not None
                          and selector_timestamp >= bootstrap_timestamp)
        if not selector_scope:
            continue

        candidates = [row for row in diagnostics_by_quality_sample.get(quality_timestamp, [])
                      if int(row.get("timestamp", 0)) <= selector_timestamp]
        diagnostic = candidates[-1] if candidates else None
        if diagnostic is None:
            unmatched.append(selector_timestamp)
            continue

        quality_valid = bool(diagnostic.get("quality_input_valid", False))
        if not quality_valid:
            continue

        gate_contract_ok = (int(diagnostic.get("experiment_mode", -1)) == 3
                            and bool(diagnostic.get("selector_quality_enabled", False))
                            and bool(diagnostic.get("quality_fusion_gate_enabled", False)))
        quality_rejected = bool(selector.get("quality_rejected", False))
        fuse_enabled = bool(diagnostic.get("fuse_enabled", False))
        state_ok = quality_rejected == (not fuse_enabled)

        if not gate_contract_ok:
            contract_errors.append(selector_timestamp)
        if not state_ok:
            state_errors.append(selector_timestamp)

        checked_rows.append({
            "selector_timestamp": selector_timestamp,
            "quality_timestamp_sample": quality_timestamp,
            "diagnostic_timestamp": int(diagnostic.get("timestamp", 0)),
            "diagnostic_qmon": is_quality_only_monitoring(diagnostic),
            "airspeed_q": diagnostic.get("airspeed_q"),
            "q_raw": diagnostic.get("q_raw"),
            "fuse_enabled": fuse_enabled,
            "quality_rejected": quality_rejected,
            "gate_contract_ok": gate_contract_ok,
            "state_matches_unified_gate": state_ok,
        })

    summary["selector_unified_gate"] = {
        "checked_count": len(checked_rows),
        "qmon_join_count": sum(row["diagnostic_qmon"] for row in checked_rows),
        "unmatched_count": len(unmatched),
        "contract_error_count": len(contract_errors),
        "state_error_count": len(state_errors),
    }
    required = expected_mode == 3
    add_check(summary, "selector.unified_gate_join",
              not required or (bool(checked_rows) and not unmatched),
              {"required": required, "checked_count": len(checked_rows),
               "unmatched_timestamps": unmatched[:20]})
    add_check(summary, "selector.unified_gate_contract", not contract_errors,
              {"error_count": len(contract_errors), "timestamps": contract_errors[:20]})
    add_check(summary, "selector.mirrors_unified_gate", not state_errors,
              {"error_count": len(state_errors), "timestamps": state_errors[:20],
               "semantics": "quality_rejected == !fuse_enabled for eligible Full-mode physical sources"})
    csv_tables["selector_unified_gate"] = checked_rows


def analyze_r_join(summary, diagnostic_rows, aid_rows, minimum_rate, abs_tol, rel_tol, csv_tables):
    diagnostics = sorted_rows(observation_records(diagnostic_rows), "ekf_buffer_timestamp_sample")
    joined, unmatched = exact_join(diagnostics, aid_rows, "ekf_buffer_timestamp_sample", "timestamp_sample",
                                   include_multi_id=True)
    result_rows = []
    absolute_errors = []
    relative_errors = []
    mismatch_count = 0
    for diagnostic, aid in joined:
        diagnostic_r = float(diagnostic.get("r_as_used", math.nan))
        aid_r = float(aid.get("observation_variance", math.nan))
        absolute_error = abs(diagnostic_r - aid_r) if finite(diagnostic_r) and finite(aid_r) else math.inf
        relative_error = absolute_error / max(abs(diagnostic_r), abs_tol) if finite(absolute_error) else math.inf
        matches = close_enough(aid_r, diagnostic_r, abs_tol, rel_tol)
        mismatch_count += not matches
        absolute_errors.append(absolute_error)
        relative_errors.append(relative_error)
        result_rows.append({
            "observation_timestamp_sample": int(diagnostic.get("timestamp_sample", 0)),
            "ekf_buffer_timestamp_sample": int(diagnostic.get("ekf_buffer_timestamp_sample", 0)),
            "diagnostic_publication_timestamp": int(diagnostic.get("timestamp", 0)),
            "aid_publication_timestamp": int(aid.get("timestamp", 0)),
            "diagnostic_r_as_used": diagnostic_r,
            "aid_observation_variance": aid_r,
            "absolute_error": absolute_error,
            "relative_error": relative_error,
            "matches": matches,
        })
    for diagnostic in unmatched:
        result_rows.append({
            "observation_timestamp_sample": int(diagnostic.get("timestamp_sample", 0)),
            "ekf_buffer_timestamp_sample": int(diagnostic.get("ekf_buffer_timestamp_sample", 0)),
            "diagnostic_publication_timestamp": int(diagnostic.get("timestamp", 0)),
            "aid_publication_timestamp": None,
            "diagnostic_r_as_used": diagnostic.get("r_as_used"),
            "aid_observation_variance": None,
            "absolute_error": None,
            "relative_error": None,
            "matches": False,
        })
    rate = len(joined) / len(diagnostics) if diagnostics else 0.0
    finite_absolute = [value for value in absolute_errors if finite(value)]
    finite_relative = [value for value in relative_errors if finite(value)]
    summary["r_join"] = {
        "join_key": "uORB instance + ekf_buffer_timestamp_sample -> aid-source timestamp_sample",
        "publication_timestamp_used_as_join_key": False,
        "diagnostic_count": len(diagnostics),
        "matched_count": len(joined),
        "unmatched_count": len(unmatched),
        "match_rate": rate,
        "minimum_match_rate": minimum_rate,
        "value_mismatch_count": mismatch_count,
        "absolute_error": {
            "max": max(finite_absolute) if finite_absolute else None,
            "mean": statistics.fmean(finite_absolute) if finite_absolute else None,
            "p95": percentile(finite_absolute, 0.95),
        },
        "relative_error": {
            "max": max(finite_relative) if finite_relative else None,
            "mean": statistics.fmean(finite_relative) if finite_relative else None,
            "p95": percentile(finite_relative, 0.95),
        },
        "unmatched_samples": [{
            "observation_timestamp_sample": int(row.get("timestamp_sample", 0)),
            "ekf_buffer_timestamp_sample": int(row.get("ekf_buffer_timestamp_sample", 0)),
        } for row in unmatched],
    }
    add_check(summary, "r_join.high_match_rate", bool(diagnostics) and rate >= minimum_rate, summary["r_join"])
    add_check(summary, "r_join.value_equality", mismatch_count == 0,
              {"mismatch_count": mismatch_count, "absolute_tolerance": abs_tol, "relative_tolerance": rel_tol})
    csv_tables["r_join"] = result_rows


def ratio_variation(rows):
    ratios = [float(row["r_as_used"]) / float(row["nominal_r_as"]) for row in rows
              if finite(row.get("r_as_used")) and finite(row.get("nominal_r_as"))
              and float(row["nominal_r_as"]) > 0]
    qualities = [float(row["airspeed_q"]) for row in rows if finite(row.get("airspeed_q"))]
    return {
        "ratio_min": min(ratios) if ratios else None,
        "ratio_max": max(ratios) if ratios else None,
        "ratio_span": max(ratios) - min(ratios) if ratios else None,
        "quality_min": min(qualities) if qualities else None,
        "quality_max": max(qualities) if qualities else None,
        "quality_span": max(qualities) - min(qualities) if qualities else None,
    }


def constant_r_source_match(row):
    airspeed_device_id = int(row.get("airspeed_device_id", 0))
    return (int(row.get("airspeed_source", -1)) == 1
            and int(row.get("quality_source_instance", 255)) == 0
            and airspeed_device_id != 0
            and int(row.get("quality_device_id", 0)) == airspeed_device_id
            and bool(row.get("source_identity_match", False)))


def adaptive_r_sample_usable(row):
    return (constant_r_source_match(row)
            and bool(row.get("quality_causal", False))
            and bool(row.get("quality_fresh_for_observation", False))
            and bool(row.get("quality_input_valid", False))
            and finite(row.get("airspeed_q")))


def analyze_mode(summary, rows, expected_mode, parameters, abs_tol, rel_tol):
    rows = observation_records(rows)
    modes = {int(row.get("experiment_mode", -1)) for row in rows}
    adaptive_rows = [row for row in rows if bool(row.get("adaptive_r_applied", False))]
    variation = ratio_variation(adaptive_rows)
    rcst = parameters.get("EKF2_ASP_RCST")
    rmax = parameters.get("EKF2_ASP_RMAX")
    mode_ok = bool(rows) and modes == {expected_mode}
    flags_ok = False
    r_ok = False
    requested_expected = expected_mode in {2, 3}
    request_errors = [row for row in rows
                      if bool(row.get("adaptive_r_requested", False)) != requested_expected]
    alias_errors = [row for row in rows
                    if bool(row.get("adaptive_r_enabled", False)) != bool(row.get("adaptive_r_applied", False))]
    applied_errors = []
    nominal_fallback_errors = []
    adaptive_formula_errors = []
    constant_scope_errors = []

    for row in rows:
        applied = bool(row.get("adaptive_r_applied", False))
        usable = adaptive_r_sample_usable(row)

        if expected_mode in {2, 3} and applied != usable:
            applied_errors.append(row)

        if expected_mode in {2, 3} and not usable:
            nominal = close_enough(row.get("r_as_used"), row.get("nominal_r_as"), abs_tol, rel_tol)
            fallback_recorded = int(row.get("fallback_to_nominal_reason", 0)) != FALLBACK_TO_NOMINAL_REASON_NONE

            if not nominal or applied or not fallback_recorded:
                nominal_fallback_errors.append(row)

        if expected_mode in {2, 3} and usable:
            if rmax is None or not finite(row.get("airspeed_q")):
                adaptive_formula_errors.append(row)

            else:
                q = min(max(float(row["airspeed_q"]), 0.0), 1.0)
                expected_r = float(row.get("nominal_r_as", math.nan)) * (1.0 + (1.0 - q) * (max(float(rmax), 1.0) - 1.0))

                if not close_enough(row.get("r_as_used"), expected_r, abs_tol, rel_tol):
                    adaptive_formula_errors.append(row)

        if expected_mode == 1:
            matched = constant_r_source_match(row)
            expected_r = float(row.get("nominal_r_as", math.nan))

            if matched and rcst is not None:
                expected_r *= max(float(rcst), 1.0)

            if rcst is None or not close_enough(row.get("r_as_used"), expected_r, abs_tol, rel_tol):
                constant_scope_errors.append(row)

            if not matched and int(row.get("fallback_to_nominal_reason", 0)) == FALLBACK_TO_NOMINAL_REASON_NONE:
                constant_scope_errors.append(row)

    details = {
        "expected_mode": MODE_NAMES[expected_mode],
        "observed_modes": sorted(modes),
        "adaptive_request_error_count": len(request_errors),
        "adaptive_alias_error_count": len(alias_errors),
        "adaptive_application_error_count": len(applied_errors),
        "nominal_fallback_error_count": len(nominal_fallback_errors),
        "adaptive_formula_error_count": len(adaptive_formula_errors),
        "constant_scope_error_count": len(constant_scope_errors),
        **variation,
    }
    if expected_mode == 0:
        flags_ok = (not request_errors and not alias_errors
                    and all(not bool(row.get("adaptive_r_enabled", False))
                       and not bool(row.get("quality_fusion_gate_enabled", False))
                       and not bool(row.get("selector_quality_enabled", False)) for row in rows))
        r_ok = all(close_enough(row.get("r_as_used"), row.get("nominal_r_as"), abs_tol, rel_tol) for row in rows)
    elif expected_mode == 1:
        flags_ok = (not request_errors and not alias_errors
                    and all(not bool(row.get("adaptive_r_enabled", False))
                       and not bool(row.get("quality_fusion_gate_enabled", False))
                       and not bool(row.get("selector_quality_enabled", False)) for row in rows))
        r_ok = rcst is not None and not constant_scope_errors
        details["expected_constant_ratio"] = rcst
    elif expected_mode == 2:
        flags_ok = (not request_errors and not alias_errors and not applied_errors
                    and all(bool(row.get("adaptive_r_enabled", False)) == adaptive_r_sample_usable(row)
                       and not bool(row.get("quality_fusion_gate_enabled", False))
                       and not bool(row.get("selector_quality_enabled", False)) for row in rows))
        r_ok = (not nominal_fallback_errors and not adaptive_formula_errors
                and variation["quality_span"] is not None and variation["quality_span"] > 1e-4
                and variation["ratio_span"] is not None and variation["ratio_span"] > 1e-4)
    elif expected_mode == 3:
        flags_ok = (not request_errors and not alias_errors and not applied_errors
                    and all(bool(row.get("adaptive_r_enabled", False)) == adaptive_r_sample_usable(row)
                       and bool(row.get("quality_fusion_gate_enabled", False)) == bool(row.get("source_identity_match", False))
                       and bool(row.get("selector_quality_enabled", False)) for row in rows))
        r_ok = (not nominal_fallback_errors and not adaptive_formula_errors
                and variation["quality_span"] is not None and variation["quality_span"] > 1e-4
                and variation["ratio_span"] is not None and variation["ratio_span"] > 1e-4)
    summary["mode_truth_table"] = {**details, "mode_ok": mode_ok, "flags_ok": flags_ok, "r_behavior_ok": r_ok}
    add_check(summary, "mode.expected", mode_ok, details)
    add_check(summary, "mode.path_flags", flags_ok, details)
    add_check(summary, "mode.r_behavior", r_ok, details)


def transition_explained(current, previous):
    if int(current.get("final_selected_source", -1)) == int(previous.get("final_selected_source", -1)):
        return True
    recovery = (int(current.get("final_selected_source", -1)) == int(current.get("pre_quality_source", -2))
                and bool(current.get("source_identity_match", False))
                and (int(previous.get("trigger_reason", 0)) != TRIGGER_NONE
                     or int(previous.get("fallback_outcome", 0)) != 0))
    return (int(current.get("trigger_reason", 0)) != TRIGGER_NONE
            or int(current.get("fallback_outcome", 0)) != 0
            or bool(current.get("concurrent_original_sensor_invalid", False))
            or bool(current.get("concurrent_blockage", False))
            or recovery)


def analyze_traceability(summary, rows, bootstrap_timestamp, csv_tables, formal_blockage_enabled=True):
    rows = [row for row in sorted_rows(rows) if bootstrap_timestamp is not None
            and int(row.get("timestamp", 0)) >= bootstrap_timestamp]
    semantic_errors = []
    trigger_errors = []
    fallback_errors = []
    transitions = []
    for row in rows:
        pre_source = int(row.get("pre_quality_source", -1))
        original_source = int(row.get("original_selected_source", -2))
        original_fallback = bool(row.get("original_selection_was_fallback", False))
        if original_source != pre_source or original_fallback != (pre_source in {0, 4}):
            semantic_errors.append(int(row.get("timestamp", 0)))
        trigger = int(row.get("trigger_reason", 0))
        trigger_consistent = {
            0: True,
            1: not bool(row.get("original_sensor_valid", False)) and not original_fallback,
            2: original_fallback,
            3: bool(row.get("quality_rejected", False)),
            4: bool(row.get("concurrent_blockage", False)),
            5: pre_source in PHYSICAL_SOURCES and not bool(row.get("source_identity_match", False)),
        }.get(trigger, False)
        if not trigger_consistent:
            trigger_errors.append(int(row.get("timestamp", 0)))
        attempted = bool(row.get("fallback_attempted", False))
        expected_outcome = FALLBACK_OUTCOMES.get(int(row.get("final_selected_source", -99)), 4)
        if attempted:
            if (int(row.get("fallback_outcome", 0)) != expected_outcome
                    or int(row.get("fallback_source", -99)) != int(row.get("final_selected_source", -98))
                    or bool(row.get("fallback_available", False)) != (int(row.get("final_selected_source", -1)) != -1)):
                fallback_errors.append(int(row.get("timestamp", 0)))
        elif int(row.get("fallback_outcome", 0)) != 0:
            fallback_errors.append(int(row.get("timestamp", 0)))
    for previous, current in zip(rows, rows[1:]):
        if int(previous.get("final_selected_source", -1)) != int(current.get("final_selected_source", -1)):
            transitions.append({
                "timestamp": int(current.get("timestamp", 0)),
                "previous_source": int(previous.get("final_selected_source", -1)),
                "final_source": int(current.get("final_selected_source", -1)),
                "pre_quality_source": int(current.get("pre_quality_source", -1)),
                "trigger_reason": int(current.get("trigger_reason", 0)),
                "fallback_outcome": int(current.get("fallback_outcome", 0)),
                "explained": transition_explained(current, previous),
            })
    unexplained = [row for row in transitions if not row["explained"]]
    forbidden_blockage = [int(row.get("timestamp", 0)) for row in rows
                          if not formal_blockage_enabled
                          and (bool(row.get("concurrent_blockage", False))
                               or int(row.get("trigger_reason", 0)) == TRIGGER_BLOCKAGE)]
    summary["selector_traceability"] = {
        "sample_count": len(rows),
        "semantic_error_count": len(semantic_errors),
        "trigger_error_count": len(trigger_errors),
        "fallback_mapping_error_count": len(fallback_errors),
        "transition_count": len(transitions),
        "unexplained_transition_count": len(unexplained),
        "formal_blockage_error_count": len(forbidden_blockage),
        "semantic_error_timestamps": semantic_errors,
        "trigger_error_timestamps": trigger_errors,
        "fallback_error_timestamps": fallback_errors,
        "formal_blockage_error_timestamps": forbidden_blockage,
    }
    add_check(summary, "selector.original_validity_semantics", not semantic_errors,
              {"error_timestamps": semantic_errors[:20]})
    add_check(summary, "selector.trigger_consistency", not trigger_errors,
              {"error_timestamps": trigger_errors[:20]})
    add_check(summary, "selector.fallback_outcome_matches_final_source", not fallback_errors,
              {"error_timestamps": fallback_errors[:20]})
    add_check(summary, "selector.transitions_explained", not unexplained,
              {"unexplained": unexplained[:20]})
    add_check(summary, "selector.formal_blockage_disabled", not forbidden_blockage,
              {"ASPD_QBLK_EN": int(bool(formal_blockage_enabled)),
               "error_timestamps": forbidden_blockage[:20]})
    csv_tables["selector_traceability"] = rows
    csv_tables["selector_transitions"] = transitions


def analyze_selector_mode(summary, rows, expected_mode, bootstrap_timestamp):
    rows = [row for row in rows if bootstrap_timestamp is not None
            and int(row.get("timestamp", 0)) >= bootstrap_timestamp]
    mode_errors = [int(row.get("timestamp", 0)) for row in rows
                   if int(row.get("experiment_mode", -1)) != expected_mode]
    expected_enabled = expected_mode == 3
    enable_errors = [int(row.get("timestamp", 0)) for row in rows
                     if bool(row.get("selector_quality_enabled", False)) != expected_enabled]
    add_check(summary, "selector.mode_truth_table", bool(rows) and not mode_errors and not enable_errors,
              {"sample_count": len(rows), "expected_mode": expected_mode,
               "mode_error_timestamps": mode_errors[:20], "enable_error_timestamps": enable_errors[:20]})


def analyze_logger(summary, tables, ulog_dropout_count, csv_tables):
    logger_rows = tables.get("logger_status", [])
    max_dropouts = max((int(row.get("dropouts", 0)) for row in logger_rows), default=-1)
    max_message_gaps = max((int(row.get("message_gaps", 0)) for row in logger_rows), default=-1)
    rates = []
    for topic, rows in sorted(tables.items()):
        if rows and "timestamp" in rows[0]:
            rates.append({"topic": topic, **topic_rate(rows, "timestamp")})
    summary["logger"] = {
        "ulog_dropout_count": ulog_dropout_count,
        "logger_max_dropouts": max_dropouts,
        "logger_max_message_gaps": max_message_gaps,
        "topic_rates": rates,
    }
    add_check(summary, "logger.no_dropouts_or_message_gaps",
              bool(logger_rows) and ulog_dropout_count == 0 and max_dropouts == 0 and max_message_gaps == 0,
              summary["logger"])
    csv_tables["topic_rates"] = rates


def validate_parameters(summary, parameters, config, expected_mode):
    expected = dict(config.get("parameters_exact", {}))
    expected.update(config.get("parameters_exact_freeze_before_bench", {}))
    expected["EKF2_ASP_MODE"] = expected_mode
    unresolved = [name for name, value in expected.items() if value is None]
    add_check(summary, "parameters.no_unresolved_freeze", not unresolved, {"unresolved": unresolved})
    mismatches = []
    for name, expected_value in expected.items():
        if expected_value is None:
            continue
        actual = parameters.get(name)
        if actual is None or not close_enough(actual, expected_value, 1e-5, 1e-5):
            mismatches.append({"name": name, "actual": actual, "expected": expected_value})
    add_check(summary, "parameters.frozen_values", not mismatches, {"mismatches": mismatches})
    blockage_value = parameters.get("ASPD_QBLK_EN")
    blockage_disabled = blockage_value is not None and close_enough(blockage_value, 0, 0, 0)
    add_check(summary, "parameters.formal_blockage_disabled", blockage_disabled,
              {"name": "ASPD_QBLK_EN", "actual": blockage_value, "expected": 0})
    positive_errors = [name for name in config.get("parameters_positive", [])
                       if name not in parameters or float(parameters[name]) <= 0]
    add_check(summary, "parameters.positive_values", not positive_errors, {"errors": positive_errors})


def analyze_evidence(tables, parameters, expected_mode, config, ulog_dropout_count=0):
    summary = {
        "schema_version": 3,
        "expected_mode": MODE_NAMES[expected_mode],
        "checks": [],
        "failures": [],
    }
    csv_tables = {}
    validate_parameters(summary, parameters, config, expected_mode)
    schemas_ok = True
    for topic in REQUIRED_FIELDS:
        schemas_ok &= require_fields(summary, topic, tables.get(topic, []))
    if not schemas_ok:
        summary["result"] = "FAIL"
        return summary, csv_tables

    identity = expected_identity(tables)
    summary["physical_pitot_identity"] = identity
    add_check(summary, "identity.single_nonzero_physical_pitot", identity["valid"], identity)
    valid_start = config.get("formal_valid_start_us")
    if valid_start is None:
        valid_start = first_armed_timestamp(tables.get("vehicle_status", []))
    timeout_us = max(0, int(float(config.get("identity_bootstrap_timeout_s", 5.0)) * 1e6))
    bootstrap = analyze_bootstrap(tables["airspeed_selector_quality_status"], identity["device_id"],
                                  identity["source_instance"], timeout_us, valid_start)
    summary["identity_bootstrap"] = bootstrap
    bootstrap_ok = (bootstrap["completed"] and bootstrap["within_timeout"]
                    and not bootstrap["bootstrap_overlaps_valid_interval"])
    add_check(summary, "identity.bootstrap_completed", bootstrap_ok, bootstrap)
    add_check(summary, "identity.post_bootstrap_all_physical_match",
              bootstrap["post_bootstrap_physical_sample_count"] > 0
              and bootstrap["post_bootstrap_mismatch_count"] == 0,
              bootstrap)
    bootstrap_timestamp = bootstrap["first_matched_timestamp"]

    analyze_quality_input(summary, tables["airspeed_quality_input"], bootstrap_timestamp, identity, csv_tables)
    selector_rate = float(config.get("selector_validated_min_match_rate", 0.99))
    analyze_selector_join(summary, tables["airspeed_selector_quality_status"], tables["airspeed_validated"],
                          bootstrap_timestamp, selector_rate, csv_tables)
    analyze_ekf_join(summary, tables["ekf2_airspeed_quality"], tables["airspeed_quality_input"],
                     tables["airspeed_selector_quality_status"], bootstrap_timestamp, csv_tables)
    r_rate = float(config.get("r_join_min_match_rate", 0.99))
    abs_tol = float(config.get("r_absolute_tolerance", 1e-5))
    rel_tol = float(config.get("r_relative_tolerance", 1e-5))
    post_diagnostics = [row for row in tables["ekf2_airspeed_quality"]
                        if bootstrap_timestamp is not None and int(row.get("timestamp", 0)) >= bootstrap_timestamp]
    analyze_monitoring_records(summary, post_diagnostics, tables["airspeed_quality_input"], expected_mode, csv_tables)
    analyze_selector_unified_gate(summary, tables["airspeed_selector_quality_status"], post_diagnostics,
                                  expected_mode, bootstrap_timestamp, csv_tables)
    analyze_r_join(summary, post_diagnostics, tables["estimator_aid_src_airspeed"],
                   r_rate, abs_tol, rel_tol, csv_tables)
    analyze_mode(summary, post_diagnostics, expected_mode, parameters, abs_tol, rel_tol)
    analyze_selector_mode(summary, tables["airspeed_selector_quality_status"], expected_mode, bootstrap_timestamp)
    analyze_traceability(summary, tables["airspeed_selector_quality_status"], bootstrap_timestamp, csv_tables,
                         formal_blockage_enabled=int(parameters.get("ASPD_QBLK_EN", -1)) != 0)
    analyze_logger(summary, tables, ulog_dropout_count, csv_tables)
    summary["result"] = "PASS" if not summary["failures"] else "FAIL"
    return summary, csv_tables


def ulog_tables(ulog):
    tables = defaultdict(list)
    for dataset in ulog.data_list:
        row_count = len(dataset.data.get("timestamp", []))
        for index in range(row_count):
            row = {field: values[index] for field, values in dataset.data.items() if index < len(values)}
            row["_multi_id"] = dataset.multi_id
            tables[dataset.name].append(row)
    return dict(tables)


def write_csv(path, rows):
    fields = sorted(set().union(*(row.keys() for row in rows))) if rows else ["no_rows"]
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json_value(value) for key, value in row.items()})


def write_outputs(output_dir, summary, csv_tables):
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "airspeed_evidence_summary.json"), "w", encoding="utf-8") as output:
        json.dump(json_value(summary), output, indent=2, sort_keys=True)
        output.write("\n")
    output_tables = dict(csv_tables)
    output_tables["checks"] = summary.get("checks", [])
    output_tables["failures"] = [{"failure": failure} for failure in summary.get("failures", [])]
    for name, rows in sorted(output_tables.items()):
        write_csv(os.path.join(output_dir, f"{name}.csv"), rows)


def parse_mode(value):
    upper = value.upper()
    if upper in MODE_VALUES:
        return MODE_VALUES[upper]
    try:
        numeric = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"unknown mode: {value}") from exc
    if numeric not in MODE_NAMES:
        raise argparse.ArgumentTypeError(f"mode must be 0..3 or one of {', '.join(MODE_VALUES)}")
    return numeric


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", help="real PX4 ULog")
    parser.add_argument("--expected-mode", required=True, type=parse_mode)
    parser.add_argument("--config", required=True, help="run-specific frozen parameter JSON")
    parser.add_argument("--output-dir", required=True, help="directory for JSON and CSV evidence")
    args = parser.parse_args()

    summary = {"result": "FAIL", "checks": [], "failures": ["unhandled_error"]}
    csv_tables = {}
    try:
        from pyulog import ULog
        with open(args.config, encoding="utf-8") as source:
            config = json.load(source)
        ulog = ULog(args.ulog)
        summary, csv_tables = analyze_evidence(ulog_tables(ulog), dict(ulog.initial_parameters),
                                               args.expected_mode, config, len(ulog.dropouts))
    except Exception as exc:  # Evidence must still be written on parser/schema failures.
        summary = {
            "result": "FAIL",
            "checks": [],
            "failures": ["unhandled_error"],
            "error": f"{type(exc).__name__}: {exc}",
        }
    write_outputs(args.output_dir, summary, csv_tables)
    print(f"RESULT={summary['result']}")
    print(f"SUMMARY={os.path.join(args.output_dir, 'airspeed_evidence_summary.json')}")
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
