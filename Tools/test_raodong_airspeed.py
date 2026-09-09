#!/usr/bin/env python3
"""Source-boundary regression checks for the raodong upstream restoration.

Run from any directory: python3 Tools/test_raodong_airspeed.py
Requires the two pinned commits in the local Git history; no network access.
"""

import pathlib
import subprocess
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
UPSTREAM = "ec278758eda642005a479e2f8d278f2be8795eaf"
AIR = "c9af571575"


def original(ref, path):
    return subprocess.check_output(
        ["git", "show", f"{ref}:{path}"], cwd=ROOT, text=True
    )


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class RaodongAirspeedTest(unittest.TestCase):
    def test_ekf_airspeed_core_matches_official_baseline(self):
        for path in (
            "src/modules/ekf2/EKF/common.h",
            "src/modules/ekf2/EKF/estimator_interface.h",
            "src/modules/ekf2/EKF/estimator_interface.cpp",
            "src/modules/ekf2/EKF/aid_sources/airspeed/airspeed_fusion.cpp",
        ):
            with self.subTest(path=path):
                self.assertEqual((ROOT / path).read_text(), original(UPSTREAM, path))

    def test_ekf_input_uses_official_sources_values_and_timestamps(self):
        path = "src/modules/ekf2/EKF2.cpp"
        signature = "void EKF2::UpdateAirspeedSample("
        self.assertEqual(function((ROOT / path).read_text(), signature),
                         function(original(UPSTREAM, path), signature))

    def test_selector_has_no_quality_or_custom_blockage_input(self):
        source = (ROOT / "src/modules/airspeed_selector/airspeed_selector_main.cpp").read_text()
        for removed in ("_ekf2_airspeed_quality_sub", "_quality_disable_latched",
                        "update_airspeed_blockage_status", "apply_airspeed_fallback",
                        "EKF2_ASP_MODE", "ASPD_QBLK_EN"):
            with self.subTest(removed=removed):
                self.assertNotIn(removed, source)

    def test_selector_decision_matches_official_baseline(self):
        path = "src/modules/airspeed_selector/airspeed_selector_main.cpp"
        signature = "void AirspeedModule::select_airspeed_and_publish("
        current = function((ROOT / path).read_text(), signature)
        current = current.replace("\n\tpublish_source_status(airspeed_validated);", "")
        self.assertEqual(current, function(original(UPSTREAM, path), signature))

    def test_experiment_parameters_and_presets_removed(self):
        parameters = (ROOT / "src/modules/ekf2/params_airspeed.yaml").read_text()
        for suffix in ("MODE", "RCST", "RMAX", "QON", "QOFF", "TOFF", "TON", "THLD"):
            self.assertNotIn(f"EKF2_ASP_{suffix}:", parameters)
        presets = ROOT / "artifacts/ral_revision_phase4_2/parameter_presets"
        self.assertEqual(list(presets.glob("*.params")), [])

    def test_monitoring_cannot_write_ekf_input(self):
        source = (ROOT / "src/modules/ekf2/EKF2.cpp").read_text()
        self.assertIn("void EKF2::UpdateAirspeedQualityMonitoring(", source)
        if "void EKF2::UpdateAirspeedQualityMonitoring(" in source:
            monitor = function(source, "void EKF2::UpdateAirspeedQualityMonitoring(")
            self.assertNotIn("_ekf.", monitor)
            self.assertNotIn("_airspeed_validated_sub", monitor)
            self.assertNotIn("_airspeed_sub", monitor)
            self.assertIn("_ekf2_airspeed_quality_pub.publish", monitor)

    def test_requested_features_and_handover_unchanged(self):
        paths = (
            "src/drivers/encoder/as5600", "src/drivers/rpm_capture",
            "src/modules/wing_phase", "src/modules/airspeed_injector",
            "src/lib/dataman_client", "src/modules/dataman",
            "src/lib/rate_control", "src/modules/fw_rate_control",
            "src/modules/fw_lateral_longitudinal_control",
            "src/modules/logger", "boards/px4/fmu-v6c",
        )
        changes = subprocess.check_output(
            ["git", "diff", AIR, "--name-only", "--", *paths], cwd=ROOT, text=True
        )
        self.assertEqual(changes, "")


if __name__ == "__main__":
    unittest.main()
