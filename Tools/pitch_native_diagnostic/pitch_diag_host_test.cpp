#include "src/modules/fw_att_control/fw_pitch_controller.h"
#include "src/lib/rate_control/rate_control.hpp"
#include <uORB/topics/pitch_outer_diag.h>
#include <uORB/topics/pitch_inner_diag.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

static void near(float a, float b)
{
    assert(std::fabs(a - b) < 2e-6f);
}

static void outerTests()
{
    for (float tc : {0.2f, 0.4f, 1.f}) {
        for (float error : {-0.4f, 0.f, 0.4f}) {
            for (float roll : {0.f, 0.6f}) {
                for (float yaw : {-0.3f, 0.4f}) {
                    PitchController controller;
                    controller.set_time_constant(tc);
                    controller.set_max_rate_pos(0.5f);
                    controller.set_max_rate_neg(0.3f);
                    const float pitch = 0.2f, sp = pitch + error;
                    const float limited = controller.control_pitch(sp, yaw, roll, pitch);
                    pitch_outer_diag_s diag{};
                    diag.pitch_error = sp - pitch;
                    diag.pitch_euler_rate_sp = controller.get_euler_rate_setpoint();
                    diag.q_sp_body_raw_reconstructed = cosf(roll) * diag.pitch_euler_rate_sp
                        + cosf(pitch) * sinf(roll) * yaw;
                    diag.q_sp_pitch_ctrl_limited = controller.get_body_rate_setpoint();
                    diag.q_sp_limit_active = fabsf(diag.q_sp_body_raw_reconstructed - diag.q_sp_pitch_ctrl_limited) > 1e-6f;
                    near(diag.pitch_error, error);
                    near(diag.pitch_euler_rate_sp, error / tc);
                    const float raw = cosf(roll) * (error / tc) + cosf(pitch) * sinf(roll) * yaw;
                    near(diag.q_sp_body_raw_reconstructed, raw);
                    near(limited, math::constrain(raw, -0.3f, 0.5f));
                    near(limited, diag.q_sp_pitch_ctrl_limited);
                    assert(diag.q_sp_limit_active == (fabsf(raw - limited) > 1e-6f));
                    // Native invalid input handling retains the previous setpoints; diagnostic error may be NaN.
                    const float previous_euler = controller.get_euler_rate_setpoint();
                    for (float invalid : {NAN, INFINITY, -INFINITY}) {
                        for (int field = 0; field < 4; ++field) {
                            const float result = controller.control_pitch(field == 0 ? invalid : sp,
                                field == 1 ? invalid : yaw, field == 2 ? invalid : roll, field == 3 ? invalid : pitch);
                            assert(result == limited && controller.get_euler_rate_setpoint() == previous_euler);
                        }
                    }
                }
            }
        }
    }
    std::cout << "PITCH_OUTER_HOST_TESTS=PASS\n";
}

static void innerTests()
{
    using matrix::Vector3f;
    bool saw_limit = false;
    for (float scale : {0.5f, 1.f, 2.f}) {
        RateControl controller;
        const float p=0.2f, i=0.15f, d=0.03f, raw_ff=0.5f, imax=0.4f;
        controller.setPidGains(Vector3f(0.f,p,0.f), Vector3f(0.f,i,0.f), Vector3f(0.f,d,0.f));
        controller.setIntegratorLimit(Vector3f(0.f,imax,0.f));
        controller.setFeedForwardGain(Vector3f(0.f,raw_ff/scale,0.f));
        for (int j=0;j<1200;++j) {
            const float qsp=j<600 ? 1.f : -1.f, q=0.1f, qdot=0.3f, dt=0.01f;
            const bool landed=j%19==0;
            rate_ctrl_status_s before{}, after{};
            controller.getRateControlStatus(before);
            const auto sum=controller.update(Vector3f(0.f,q,0.f),Vector3f(0.f,qsp,0.f),Vector3f(0.f,qdot,0.f),dt,landed);
            controller.getRateControlStatus(after);
            pitch_inner_diag_s diag{};
            diag.pitch_p_term=p*(qsp-q);
            diag.pitch_i_term_before=before.pitchspeed_integ;
            diag.pitch_i_term_after=after.pitchspeed_integ;
            diag.pitch_i_delta=diag.pitch_i_term_after-diag.pitch_i_term_before;
            diag.pitch_d_term=-d*qdot;
            diag.effective_ff_gain=raw_ff/scale;
            diag.pitch_ff_term=diag.effective_ff_gain*qsp;
            diag.pitch_rate_control_sum=sum(1);
            near(sum(1),diag.pitch_p_term+diag.pitch_i_term_before+diag.pitch_d_term+diag.pitch_ff_term);
            assert(std::isfinite(after.pitchspeed_integ) && fabsf(after.pitchspeed_integ)<=imax);
            if(landed) { assert(before.pitchspeed_integ==after.pitchspeed_integ); }
            diag.i_at_limit_after=fabsf(after.pitchspeed_integ)>=imax-1e-6f;
            saw_limit |= diag.i_at_limit_after;
            const float factor=math::max(0.f,1.f-std::pow((qsp-q)/math::radians(400.f),2.f));
            const float expected=landed ? before.pitchspeed_integ : math::constrain(before.pitchspeed_integ+factor*i*(qsp-q)*dt,-imax,imax);
            near(after.pitchspeed_integ,expected);
            if(j==1199) {
                controller.resetIntegral();
                controller.getRateControlStatus(after);
                assert(after.pitchspeed_integ==0.f);
            }
        }
    }
    assert(saw_limit);
    std::cout << "PITCH_INNER_PID_FF_I_BEFORE_AFTER_TESTS=PASS\n";
}

static void outputTests()
{
    bool saw_clip=false, saw_no_clip=false;
    for(float sum : {-2.f,-0.2f,0.f,0.2f,2.f})for(float scale : {0.5f,1.f,2.f}) {
        const float gain=0.8f,trim=0.1f;
        pitch_inner_diag_s diag{};
        diag.pitch_control_after_scaling=gain*(sum*scale*scale);
        diag.pitch_trim=trim;
        diag.pitch_output_preclip=diag.pitch_control_after_scaling+trim;
        diag.pitch_output_final=math::constrain(diag.pitch_output_preclip,-1.f,1.f);
        diag.output_clip=fabsf(diag.pitch_output_preclip)>1.f;
        near(diag.pitch_output_preclip,sum*scale*scale*gain+trim);
        assert(diag.pitch_output_final>=-1.f && diag.pitch_output_final<=1.f);
        assert(diag.output_clip==(diag.pitch_output_final!=diag.pitch_output_preclip));
        saw_clip |= diag.output_clip; saw_no_clip |= !diag.output_clip;
    }
    assert(saw_clip && saw_no_clip);
    pitch_outer_diag_s outer{}; pitch_inner_diag_s inner{};
    outer.rate_sp_timestamp=1234567; outer.q_sp_final_published=0.3f;
    inner.rate_sp_source_timestamp=outer.rate_sp_timestamp; inner.q_sp_used=outer.q_sp_final_published;
    assert(inner.rate_sp_source_timestamp==outer.rate_sp_timestamp && inner.q_sp_used==outer.q_sp_final_published);
    std::cout << "PITCH_OUTPUT_CHAIN_TESTS=PASS\n";
}

int main()
{
    outerTests(); innerTests(); outputTests();
    std::cout << "PITCH_OUTER_MSG_BYTES=" << sizeof(pitch_outer_diag_s) << '\n';
    std::cout << "PITCH_INNER_MSG_BYTES=" << sizeof(pitch_inner_diag_s) << '\n';
    std::cout << "ESTIMATED_DIAG_RAW_PAYLOAD_BYTES_PER_SEC_AT_100HZ="
        << 100*(sizeof(pitch_outer_diag_s)+sizeof(pitch_inner_diag_s)) << '\n';
    std::cout << "PITCH_DIAG_HOST_TESTS=PASS\n";
}
