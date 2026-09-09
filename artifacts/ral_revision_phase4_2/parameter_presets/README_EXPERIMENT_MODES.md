# raodong: official airspeed behavior

The A/B/C/D/E experiment presets were removed on this branch. Airspeed selection
and EKF fusion use the upstream baseline ec278758eda642005a479e2f8d278f2be8795eaf.
Quality is logged only; there is no EKF2_ASP_MODE or custom quality/blockage gate.

Historical experiment reports and evidence scripts elsewhere in artifacts/ and
docs/ral_revision/ describe the air branch and are not raodong acceptance checks.
See docs/raodong-airspeed.md for the current behavior and parameter migration.
