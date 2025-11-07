/****************************************************************************
 *
 * Flapping main-wing RPM controller (Peripheral via Actuator SetX backend)
 * Maps RC throttle [0..1] to a fixed flapping frequency [FRP_MIN_HZ..FRP_MAX_HZ],
 * closes the loop on uORB rpm (from MAVLink RAW_RPM) and commands output via
 * VEHICLE_CMD_DO_SET_ACTUATOR on the selected Actuator Set slot.
 *
 ****************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/parameter_update.h>

#include <math.h>
#include <float.h>

using namespace time_literals;

class FlapRpmController : public ModuleBase<FlapRpmController>, public ModuleParams
{
public:
    FlapRpmController();
    ~FlapRpmController() override = default;

    static int task_spawn(int argc, char *argv[]);
    static FlapRpmController *instantiate(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);

    int print_status() override;

    void run() override; // must be public for ModuleBase run_trampoline

private:
    void updateParams() override;

    // helpers
    static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

    // uORB
    uORB::Subscription _manual_sp_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _rpm_sub{ORB_ID(rpm)};
    uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
    uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};
    uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

    // params
    int32_t _p_slot{1};
    float   _p_min_hz{5.f};
    float   _p_max_hz{20.f};
    float   _p_thr_db{0.f};
    float   _p_thr_exp{1.f};
    float   _p_sp_slew_hz{0.f};
    float   _p_thr_min{0.f};
    float   _p_thr_max{1.f};
    float   _p_kp{0.3f};
    float   _p_ki{0.1f};
    float   _p_kd{0.0f};
    float   _p_u_min{0.f};
    float   _p_u_max{1.f};
    float   _p_rate_hz{50.f};
    int32_t _p_timeout_ms{300};

    // pid state
    float _i_term{0.f};
    float _last_err{0.f};
    hrt_abstime _last_ts{0};
    hrt_abstime _last_rpm_ts{0};

    // setpoint state (for slew limiting)
    float _freq_sp_state{0.f};

    bool _should_exit{false};

    // ModuleParams parameter wrappers (generated from module.yaml)
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::FRP_SLOT>)        _prm_slot,
        (ParamFloat<px4::params::FRP_MIN_HZ>)    _prm_min_hz,
        (ParamFloat<px4::params::FRP_MAX_HZ>)    _prm_max_hz,
        (ParamFloat<px4::params::FRP_THR_DB>)    _prm_thr_db,
        (ParamFloat<px4::params::FRP_THR_EXP>)   _prm_thr_exp,
        (ParamFloat<px4::params::FRP_SP_SLEW>)   _prm_sp_slew,
        (ParamFloat<px4::params::FRP_THR_MIN>)   _prm_thr_min,
        (ParamFloat<px4::params::FRP_THR_MAX>)   _prm_thr_max,
        (ParamFloat<px4::params::FRP_KP>)        _prm_kp,
        (ParamFloat<px4::params::FRP_KI>)        _prm_ki,
        (ParamFloat<px4::params::FRP_KD>)        _prm_kd,
        (ParamFloat<px4::params::FRP_U_MIN>)     _prm_u_min,
        (ParamFloat<px4::params::FRP_U_MAX>)     _prm_u_max,
        (ParamFloat<px4::params::FRP_RATE_HZ>)   _prm_rate_hz,
        (ParamInt<px4::params::FRP_TIMEOUT_MS>)  _prm_timeout_ms
    )
};

FlapRpmController::FlapRpmController() : ModuleParams(nullptr)
{
}

void FlapRpmController::updateParams()
{
    ModuleParams::updateParams();

    _p_slot        = _prm_slot.get();
    _p_min_hz      = _prm_min_hz.get();
    _p_max_hz      = _prm_max_hz.get();
    _p_thr_db      = _prm_thr_db.get();
    _p_thr_exp     = _prm_thr_exp.get();
    _p_sp_slew_hz  = _prm_sp_slew.get();
    _p_thr_min     = _prm_thr_min.get();
    _p_thr_max     = _prm_thr_max.get();
    _p_kp          = _prm_kp.get();
    _p_ki          = _prm_ki.get();
    _p_kd          = _prm_kd.get();
    _p_u_min       = _prm_u_min.get();
    _p_u_max       = _prm_u_max.get();
    _p_rate_hz     = _prm_rate_hz.get();
    _p_timeout_ms  = _prm_timeout_ms.get();

    _p_slot = (int32_t)clampf((float)_p_slot, 1.f, 6.f);
    _p_min_hz = fmaxf(0.f, _p_min_hz);
    _p_max_hz = fmaxf(_p_min_hz, _p_max_hz);
    _p_thr_db = clampf(_p_thr_db, 0.f, 0.5f);
    _p_thr_exp = clampf(_p_thr_exp, 0.5f, 5.0f);
    _p_sp_slew_hz = clampf(_p_sp_slew_hz, 0.f, 50.f);
    _p_thr_min = clampf(_p_thr_min, 0.f, 1.f);
    _p_thr_max = clampf(_p_thr_max, 0.f, 1.f);
    if (_p_thr_max < _p_thr_min + 1e-3f) _p_thr_max = fminf(1.f, _p_thr_min + 0.01f);
    _p_u_min = clampf(_p_u_min, 0.f, 1.f);
    _p_u_max = clampf(_p_u_max, 0.f, 1.f);
    if (_p_u_max < _p_u_min) _p_u_max = _p_u_min;
    _p_rate_hz = clampf(_p_rate_hz, 5.f, 200.f);
    _p_timeout_ms = (int32_t)clampf((float)_p_timeout_ms, 50.f, 5000.f);
}

int FlapRpmController::print_status()
{
    PX4_INFO("slot=%d fmin=%.2fHz fmax=%.2fHz kp=%.3f ki=%.3f kd=%.3f u[%.2f,%.2f] rate=%.1fHz timeout=%dms",
             (int)_p_slot, (double)_p_min_hz, (double)_p_max_hz,
             (double)_p_kp, (double)_p_ki, (double)_p_kd,
             (double)_p_u_min, (double)_p_u_max, (double)_p_rate_hz, (int)_p_timeout_ms);
    return 0;
}

void FlapRpmController::run()
{
    const float dt_target = 1.f / _p_rate_hz;
    _last_ts = hrt_absolute_time();
    _last_rpm_ts = 0;
    _i_term = 0.f;
    _last_err = 0.f;

    while (!_should_exit) {
        // parameter update
        if (_param_update_sub.updated()) {
            parameter_update_s upd{}; _param_update_sub.copy(&upd);
            updateParams();
        }

        // armed state
        actuator_armed_s armed{}; (void)_armed_sub.copy(&armed);

        // throttle → freq setpoint (with deadband + expo + optional slew)
        manual_control_setpoint_s man{};
        float throttle = NAN; // unified to [0..1]
        if (_manual_sp_sub.copy(&man) && man.valid) {
            // manual_control_setpoint.throttle is typically in [-1..1].
            // Convert robustly to [0..1] and clamp.
            float thr_raw = man.throttle;
            // If already in [0..1], this still works (maps 0..1 -> 0.5..1, detect below):
            if (thr_raw < 0.f || thr_raw > 1.f) {
                // Assume [-1..1]
                throttle = clampf((thr_raw + 1.f) * 0.5f, 0.f, 1.f);
            } else {
                // Already [0..1]
                throttle = clampf(thr_raw, 0.f, 1.f);
            }
        }

        // rpm feedback
        rpm_s rpm{};
        bool rpm_ok = _rpm_sub.copy(&rpm);
        if (rpm_ok) {
            _last_rpm_ts = rpm.timestamp;
        }

        const hrt_abstime now = hrt_absolute_time();
        const float dt = fminf(1.f, (now - _last_ts) * 1e-6f);
        _last_ts = now;

        // default output (safe)
        float u = _p_u_min;

        if (armed.armed && PX4_ISFINITE(throttle)) {
            // min/max window mapping (values outside [thr_min..thr_max] saturate)
            const float t_clamped = clampf(throttle, _p_thr_min, _p_thr_max);
            float t_eff = (t_clamped - _p_thr_min) / fmaxf(1e-3f, (_p_thr_max - _p_thr_min));
            t_eff = clampf(t_eff, 0.f, 1.f);
            // expo
            const float t_shaped = powf(t_eff, _p_thr_exp);
            float freq_sp = _p_min_hz + t_shaped * (_p_max_hz - _p_min_hz);
            // slew limit (Hz/s)
            if (_p_sp_slew_hz > 0.f) {
                const float max_step = _p_sp_slew_hz * dt;
                const float delta = freq_sp - _freq_sp_state;
                const float step = clampf(delta, -max_step, max_step);
                _freq_sp_state = _freq_sp_state + step;
                freq_sp = _freq_sp_state;
            } else {
                _freq_sp_state = freq_sp;
            }

            // check feedback timeout
            const bool feedback_valid = (_last_rpm_ts != 0) && ((now - _last_rpm_ts) < (uint64_t)_p_timeout_ms * 1000ULL);

            if (feedback_valid) {
                const float freq_meas = fmaxf(0.f, rpm.rpm_estimate);
                const float err = freq_sp - freq_meas;

                // PID
                _i_term += _p_ki * err * dt;
                _i_term = clampf(_i_term, -1.f, 1.f); // simple anti-windup in normalized space
                const float d = (dt > FLT_EPSILON) ? (err - _last_err) / dt : 0.f;
                _last_err = err;

                float du = _p_kp * err + _i_term + _p_kd * d;

                // accumulate on top of previous u? Use direct form towards range
                // Here we treat du as absolute control for simplicity
                u = clampf(du, _p_u_min, _p_u_max);

            } else {
                // feedback lost -> safe output
                u = _p_u_min;
                _i_term = 0.f;
            }
        } else {
            _i_term = 0.f;
        }

        // Convert to Actuator Set value [-1,1]
        const float v = 2.f * u - 1.f;

        // publish VEHICLE_CMD_DO_SET_ACTUATOR
        vehicle_command_s cmd{};
        cmd.timestamp = now;
        cmd.param1 = NAN; cmd.param2 = NAN; cmd.param3 = NAN; cmd.param4 = NAN; cmd.param5 = NAN; cmd.param6 = NAN; cmd.param7 = 0.f;
        const int slot = (int)_p_slot;
        switch (slot) {
            case 1: cmd.param1 = v; break;
            case 2: cmd.param2 = v; break;
            case 3: cmd.param3 = v; break;
            case 4: cmd.param4 = v; break;
            case 5: cmd.param5 = v; break;
            case 6: cmd.param6 = v; break;
            default: break;
        }
        cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
        cmd.target_system = 0; // local
        cmd.target_component = 0;
        cmd.source_system = 0;
        cmd.source_component = 0;
        cmd.from_external = false;
        _vehicle_cmd_pub.publish(cmd);

        // sleep to maintain loop rate
        const uint32_t us = (uint32_t)(dt_target * 1e6f);
        px4_usleep(us);
    }
}

int FlapRpmController::task_spawn(int argc, char *argv[])
{
    // spawn thread; object will be created within run_trampoline() via instantiate()
    _task_id = px4_task_spawn_cmd("flap_rpm_controller",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1600,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        PX4_ERR("task start failed");
        return PX4_ERROR;
    }

    return PX4_OK;
}

FlapRpmController *FlapRpmController::instantiate(int argc, char *argv[])
{
    // No special CLI args for now, simply create the instance and return
    FlapRpmController *instance = new FlapRpmController();
    if (instance) {
        // initialize parameters
        instance->updateParams();
    }
    return instance;
}

int FlapRpmController::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int FlapRpmController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }
    PRINT_MODULE_USAGE_NAME("flap_rpm_controller", "examples");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int flap_rpm_controller_main(int argc, char *argv[])
{
    return FlapRpmController::main(argc, argv);
}
