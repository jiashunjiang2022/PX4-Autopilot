# Experiment mode truth table

`EKF2_ASP_MODE` is reboot-required. EKF2 and airspeed selector capture it at module construction, so runtime parameter changes do not mix modes. Unknown values report an error and use BASELINE behavior.

| Mode | Estimator/logging | R used | Quality fusion gate | Selector quality path |
|---|---|---|---|---|
| 0 BASELINE | on | nominal | off | off |
| 1 CONSTANT_R | on | nominal x `EKF2_ASP_RCST` | off | off |
| 2 VARIANCE_ONLY | on | q-adaptive, bounded by `EKF2_ASP_RMAX` | off | off |
| 3 FULL_PROPOSED | on | q-adaptive, bounded by `EKF2_ASP_RMAX` | on | on |

`EKF2_ASP_RCST` is a dimensionless variance multiplier, not a standard-deviation multiplier. It is reboot-required and captured at EKF2 startup. Its default 1.0 is only a safe placeholder.

The mode helper is shared by EKF2 and selector. Independent booleans are derived from the enum and are logged in `ekf2_airspeed_quality`; there are no separate user parameters that can create unsupported combinations.
