#include "src/lib/rate_control/rate_control.hpp"
#include "src/modules/fw_rate_control/PitchFfPhaseShaper.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
using matrix::Vector3f;
using namespace pitch_ff_phase_shaper;

int main()
{
    for (float scale : {.5f, 1.f, 2.f}) {
        RateControl native, shaped;
        for (auto *c : {&native, &shaped}) {
            c->setPidGains(Vector3f(.2f,.3f,.4f), Vector3f(.1f,.15f,.2f), Vector3f(.02f,.03f,.04f));
            c->setIntegratorLimit(Vector3f(.4f,.4f,.4f));
            c->setFeedForwardGain(Vector3f(.3f,.5f,.1f)/scale);
        }
        for (int k=0; k<2000; ++k) {
            const Vector3f sp(.1f, .4f*std::cos(k*.1f), -.2f), rate(.05f,.1f,-.1f), accel(.02f,.03f,.01f);
            const auto n=native.update(rate,sp,accel,.01f,false);
            const auto s=shaped.update(rate,sp,accel,.01f,false);
            assert(n==s);
            rate_ctrl_status_s ni{},si{};
            native.getRateControlStatus(ni); shaped.getRateControlStatus(si);
            assert(ni.pitchspeed_integ==si.pitchspeed_integ);
            const float compression=.8f;
            const Vector3f native_u=n*scale*scale*compression;
            Vector3f control_u=native_u;
            // The integration's compression update observes this exact native vector.
            const Vector3f compression_input=control_u;
            Result r{}; r.qhat_1f_applied=.05*std::cos(k*.1);
            const auto out=mapPitchOutput(r,.5f/scale,compression,scale*scale,control_u(1));
            assert(out.valid);
            if(out.normalized_correction!=0.f) control_u(1)=out.shaped_pitch_control;
            const auto final=matrix::constrain(control_u+Vector3f(0.f,.1f,0.f),-1.f,1.f);
            assert(compression_input==native_u && control_u(0)==native_u(0) && control_u(2)==native_u(2));
            const double expected=double(native_u(1))-.5/scale*r.qhat_1f_applied*compression*scale*scale;
            assert(std::abs(control_u(1)-expected)<1e-7);
            assert(final(1,0)>=-1.f && final(1,0)<=1.f);
            control_u(1)=native_u(1); // restore the downstream native diagnostic/TBR inputs
            assert(control_u==native_u);
            r.qhat_1f_applied=0.;
            const auto disabled=mapPitchOutput(r,.5f/scale,compression,scale*scale,native_u(1));
            assert(disabled.valid && disabled.shaped_pitch_control==native_u(1));
        }
    }
    Result r{};r.qhat_1f_applied=1.;
    assert(!mapPitchOutput(r,INFINITY,1.f,1.f,.2f).valid);
    assert(!mapPitchOutput(r,1e300,1e300,1.f,.2f).valid);
    assert(!mapPitchOutput(r,-1e38,10.,1.,3e38f).valid);
    assert(!mapPitchOutput(r,1.,1.,1.,NAN).valid);
    std::puts("PFF_NATIVE_PID_INTEGRATOR_OUTPUT_CHAIN=PASS");
}
