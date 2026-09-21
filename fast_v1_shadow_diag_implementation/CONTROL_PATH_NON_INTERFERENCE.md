# Control-path non-interference audit

The new state is diagnostic-only. It is not read by PID, allocator, gain compression, airspeed scaling, Slow state transitions, setpoints, or actuator publications.

| NEW_VARIABLE | WRITE_SITES | READ_SITES | AFFECTS_CONTROL_OUTPUT |
|---|---|---|---|
| `_roll_i_pre_imax_accum` | `RateControl::updateIntegral`, reset | `getRateControlStatus` | NO |
| `_roll_i_accepted_accum` | `RateControl::updateIntegral`, reset | `getRateControlStatus` | NO |
| `_roll_i_bound_reject_accum` | `RateControl::updateIntegral`, reset | `getRateControlStatus` | NO |
| `_roll_i_update_count` | `RateControl::updateIntegral`, reset | `getRateControlStatus` | NO |
| `_rate_ctrl_status_pub_seq` | existing `rate_ctrl_status` publication site | message field only | NO |

The patch does not alter `rate_error`, PID gains, integral bounds, `_rate_int`, transfer state, allocator feedback, scheduler, publication cadence, logger intervals, parameters, torque setpoints, or actuator commands. Transfer cumulative diagnostics were deliberately deferred because the state machine has multiple limited/safety/exit paths and no single safe transaction hook was added.
