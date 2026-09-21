# Control-path non-interference audit V2

All cumulative state is written only at the existing natural integral update, reset by the existing roll diagnostic reset, exposed through read-only getters, and copied into `flap_fast_shadow`. `_rate_ctrl_status_pub_seq` is incremented only at the existing status publication site and copied to the paired diagnostic message.

| Diagnostic state / field | Writes | Reads | Publication | Control RHS | Affects output |
|---|---|---|---|---|---|
| C_pre / `rollspeed_integ_pre_imax_accum` | `RateControl::updateIntegral`, reset | getter | `flap_fast_shadow` | none | NO |
| C_acc / `rollspeed_integ_accepted_accum` | `RateControl::updateIntegral`, reset | getter | `flap_fast_shadow` | none | NO |
| C_reject / `rollspeed_integ_bound_reject_accum` | `RateControl::updateIntegral`, reset | getter | `flap_fast_shadow` | none | NO |
| update count | natural update, reset | getter | `flap_fast_shadow` | none | NO |
| publication sequence | existing publish path | local copy | `flap_fast_shadow` | none | NO |
| FlapFastShadow snapshots | message construction | logger/uORB | `flap_fast_shadow` | none | NO |

Search of the changed source shows no new diagnostic state in `rate_error`, `_rate_int` calculation, `control_u`, torque/actuator setpoints, gain compression, airspeed scaling, allocator inputs, scheduler, or Slow transfer decisions. The existing controller behavior is unchanged.
