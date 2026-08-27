# RA-L Airspeed Experiment Modes

Apply exactly one complete preset before each formal run. In QGroundControl, open
**Vehicle Setup > Parameters > Tools > Load from file**, select the required
`.params` file, wait for all writes to finish, then reboot the flight controller.
Every preset explicitly writes the complete frozen parameter set.

| Mode | Purpose | Preset | `EKF2_ASP_MODE` | `EKF2_ARSP_THR` | `FW_USE_AIRSPD` | Reboot |
|---|---|---|---:|---:|---:|---|
| A | Baseline, `R=R0` | `A_baseline.params` | 0 | 8 | 1 | Required |
| B | Constant-R, `R=2.5 R0` | `B_constant_r.params` | 1 | 8 | 1 | Required |
| C | Variance-only, `R=R(q)` | `C_variance_only.params` | 2 | 8 | 1 | Required |
| D | Full unified gate | `D_full.params` | 3 | 8 | 1 | Required |
| E | No-Pitot use; Pitot remains installed/logged | `E_no_pitot.params` | 0 | 0 | 0 | Required |

All modes keep `SYS_HAS_NUM_ASPD=1` and `ASPD_QBLK_EN=0`.

## Critical E-to-A/B/C/D Warning

After Mode E, changing only `EKF2_ASP_MODE` is invalid: airspeed fusion and
controller use would remain disabled. Apply the complete A, B, C, or D preset so
that `EKF2_ARSP_THR=8` and `FW_USE_AIRSPD=1` are restored.

## Verification

After reboot, verify the core values in the NSH console:

```sh
param show EKF2_ASP_MODE
param show EKF2_ARSP_THR
param show FW_USE_AIRSPD
param show SYS_HAS_NUM_ASPD
param show ASPD_QBLK_EN
```

Then run the matching offline preflight check on the post-reboot ULog:

```sh
python3 artifacts/ral_revision_phase4_2/parameter_template/preflight_check.py run.ulg --experiment-mode A
python3 artifacts/ral_revision_phase4_2/parameter_template/preflight_check.py run.ulg --experiment-mode B
python3 artifacts/ral_revision_phase4_2/parameter_template/preflight_check.py run.ulg --experiment-mode C
python3 artifacts/ral_revision_phase4_2/parameter_template/preflight_check.py run.ulg --experiment-mode D
python3 artifacts/ral_revision_phase4_2/parameter_template/preflight_check.py run.ulg --experiment-mode E
```

The checker reports every parameter as `actual` versus `expected` and returns
nonzero on any mismatch, including the E-to-D misconfiguration.

## Stored-Parameter Caveat

PX4 parameter storage survives firmware flashing. Changing a source default does
not overwrite an existing stored value. For one-time initialization, load
`A_baseline.params`; do not use `param reset_all`, because that would erase
unrelated controller, calibration, RC, GPS, telemetry, and mission settings.

Presets are applied only by explicit user action. No boot script automatically
changes experiment parameters.
