# Diagnostic field semantics

All cumulative natural fields are scoped to the current `roll_i_reset_epoch`. `rollspeed_integ_update_count` is a uint32 count of finite natural roll integral updates and may wrap; offline consumers must compare modulo 2^32. `rate_ctrl_status_pub_seq` is a module-lifetime uint32 publication counter and is independent of integral reset; it also wraps modulo 2^32.

`rollspeed_integ_pre_imax_accum` is the cumulative post-allocator-anti-windup, pre-Imax natural increment. `rollspeed_integ_accepted_accum` is the cumulative natural increment after active integral bounds. `rollspeed_integ_bound_reject_accum` is their cumulative difference. These are controller-internal diagnostics and are not disturbance, aerodynamic torque, or required torque estimates.

The three transfer cumulative fields are intentionally absent: implementation is deferred until every accepted transfer transaction can be instrumented exactly once without changing the state machine.
