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
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);

    int print_status() override;

private:
    void updateParams() override;
    void run();

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

    bool _should_exit{false};
};

FlapRpmController::FlapRpmController() : ModuleParams(nullptr)
{
}

void FlapRpmController::updateParams()
{
    // Pull parameters from storage
    param_get(param_find("FRP_SLOT"), &_p_slot);
    param_get(param_find("FRP_MIN_HZ"), &_p_min_hz);
    param_get(param_find("FRP_MAX_HZ"), &_p_max_hz);
    param_get(param_find("FRP_KP"), &_p_kp);
    param_get(param_find("FRP_KI"), &_p_ki);
    param_get(param_find("FRP_KD"), &_p_kd);
    param_get(param_find("FRP_U_MIN"), &_p_u_min);
    param_get(param_find("FRP_U_MAX"), &_p_u_max);
    param_get(param_find("FRP_RATE_HZ"), &_p_rate_hz);
    param_get(param_find("FRP_TIMEOUT_MS"), &_p_timeout_ms);

    _p_slot = (int32_t)clampf((float)_p_slot, 1.f, 6.f);
    _p_min_hz = fmaxf(0.f, _p_min_hz);
    _p_max_hz = fmaxf(_p_min_hz, _p_max_hz);
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

        // throttle → freq setpoint
        manual_control_setpoint_s man{};
        float throttle = NAN;
        if (_manual_sp_sub.copy(&man) && man.valid) {
            throttle = clampf(man.throttle, 0.f, 1.f);
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
            // map throttle to frequency setpoint
            const float freq_sp = _p_min_hz + throttle * (_p_max_hz - _p_min_hz);

            // check feedback timeout
            const bool feedback_valid = (_last_rpm_ts != 0) && ((now - _last_rpm_ts) < (uint64_t)_p_timeout_ms * 1000ULL);

            if (feedback_valid) {
                const float freq_meas = fmaxf(0.f, rpm.rpm_estimate); // assume rpm in Hz or RPM? user sends Hz via RAW_RPM.frequency
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
    FlapRpmController *instance = new FlapRpmController();
    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = px4_task_spawn_cmd("flap_rpm_controller",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1600,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)nullptr);

    if (_task_id < 0) {
        PX4_ERR("task start failed");
        delete instance;
        _object.store(nullptr);
        return PX4_ERROR;
    }

    return PX4_OK;
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

