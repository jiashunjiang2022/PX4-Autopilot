// These tests catch missing gates, double fade, wrong harmonic basis, stale
// restart, unbounded adaptation and accidental 2f output shaping.
#include "src/modules/fw_rate_control/PitchFfPhaseShaper.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
#include <cstring>
using namespace pitch_ff_phase_shaper;
constexpr double pi = 3.14159265358979323846;

static Input validInput()
{
    Input i{};
    i.enable = i.armed = i.supported = i.mission = i.controller_active = true;
    i.phase_valid = i.phase_fresh = i.numeric_inputs_valid = true;
    i.dt = .01;
    i.flap_frequency_hz = 4.;
    i.amplitude_bound = 4.;
    return i;
}

static Result excite(PitchFfPhaseShaper &s, Config c, Input &i, int count = 1000)
{
    Result r{};
    for (int k = 0; k < count; ++k) {
        i.phase_rad = std::fmod(i.phase_rad + 2*pi*i.flap_frequency_hz*i.dt, 2*pi);
        i.q_sp = .1*std::cos(i.phase_rad+.3) + .04*std::sin(2*i.phase_rad);
        r = s.update(c, i);
    }
    return r;
}

int main()
{
    for (double cycles : {1., 2., 4., 8.}) assert(PitchFfPhaseShaper::validConfig({.5, cycles}));
    for (double cycles : {.99, 8.01}) assert(!PitchFfPhaseShaper::validConfig({.5, cycles}));
    Config c{};
    Input i = validInput();
    PitchFfPhaseShaper s;
    i.enable = false;
    for (int k=0; k<1000; ++k) {
        i.q_sp = (k%2) ? NAN : 1000.;
        auto r = s.update(c,i);
        assert(r.qhat_1f_applied == 0. && !r.active);
    }
    std::puts("PFF_DISABLED_NATIVE_PARITY=PASS");
    i=validInput(); c.alpha=0.;
    auto r=excite(s,c,i);
    assert(r.active && r.qhat_1f_applied==0. && std::abs(r.coeff_cos_1f)>.01);
    std::puts("PFF_ALPHA_ZERO=PASS");

    // Bounds obtained independently from the exact Python winner on this trace:
    // pure/task/sweep RMSE .006351895/.006352296/.005594180;
    // maximum error .008981160/.009040665/.008131893 (rounding only).
    for (int kind=0; kind<3; ++kind) {
        s.reset(); i=validInput(); c.alpha=1.;
        double square=0., max_error=0.;
        for (int k=0; k<3000; ++k) {
            i.flap_frequency_hz = kind==2 ? 2.5+2.5*k/3000. : 4.;
            i.phase_rad=std::fmod(i.phase_rad+2*pi*i.flap_frequency_hz*i.dt,2*pi);
            const double target=.1*std::cos(i.phase_rad+.3);
            i.q_sp=target+(kind==1 ? .03*std::sin(.2*k*.01) : 0.);
            r=s.update(c,i);
            if (k>=2500) {
                double e=r.qhat_1f_applied-target;
                square+=e*e; max_error=std::fmax(max_error,std::abs(e));
            }
        }
        const double rmse_bound[]={.006351896,.006352297,.005594181};
        const double max_bound[]={.008981161,.009040666,.008131894};
        assert(std::sqrt(square/500)<rmse_bound[kind] && max_error<max_bound[kind]);
    }
    std::puts("PFF_PURE_TASK_SWEEP_RECONSTRUCTION=PASS");

    for (bool stale : {false,true}) {
        s.reset(); i=validInput(); r=excite(s,c,i);
        double previous=r.qhat_1f_applied;
        if (stale) i.phase_fresh=false; else i.phase_valid=false;
        for(int k=0;k<200;++k) {
            r=s.update(c,i);
            assert(!r.active && std::abs(r.qhat_1f_applied-previous)<=.0080000001);
            previous=r.qhat_1f_applied;
        }
        assert(r.qhat_1f_applied==0. && r.coeff_cos_1f==0. && r.coeff_cos_2f_internal==0.);
        i.phase_valid=i.phase_fresh=true; r=s.update(c,i);
        assert(!r.active && !r.warmup_complete && r.qhat_1f_applied==0.);
    }
    assert(phaseFresh(100000,150000) && !phaseFresh(100000,150001));
    assert(!phaseFresh(150001,150000) && !phaseFresh(0,150000));
    std::puts("PFF_DROPOUT_STALE_RECOVERY=PASS");

    for (double fault : {NAN, INFINITY}) {
        for (int field=0; field<4; ++field) {
            s.reset(); i=validInput(); excite(s,c,i);
            if(field==0)i.q_sp=fault;
            if(field==1)i.phase_rad=fault;
            if(field==2)i.flap_frequency_hz=fault;
            if(field==3)i.dt=fault;
            r=s.update(c,i);
            assert(!r.active && r.qhat_1f_applied==0. && r.coeff_cos_1f==0.);
        }
    }
    for(double bad_dt : {0.,-.01,.100001}) {
        s.reset(); i=validInput(); excite(s,c,i); i.dt=bad_dt;
        assert(s.update(c,i).qhat_1f_applied==0.);
    }
    for (int gate=0;gate<6;++gate) {
        s.reset(); i=validInput(); excite(s,c,i);
        if(gate==0)i.armed=false;
        if(gate==1)i.mission=false;
        if(gate==2)i.landed=true;
        if(gate==3)i.supported=false;
        if(gate==4)i.controller_active=false;
        if(gate==5)i.numeric_inputs_valid=false;
        r=s.update(c,i); assert(!r.active && r.qhat_1f_applied==0.);
    }
    for(int cfg=0;cfg<4;++cfg) {
        Config bad{};
        if(cfg==0)bad.alpha=NAN;
        if(cfg==1)bad.alpha=1.01;
        if(cfg==2)bad.adapt_cycles=0.;
        if(cfg==3)bad.adapt_cycles=INFINITY;
        i=validInput(); assert(s.update(bad,i).qhat_1f_applied==0.);
    }
    std::puts("PFF_NUMERIC_CONFIG_AND_GATES_FAIL_NATIVE=PASS");

    s.reset(); i=validInput(); i.q_sp=1e100;
    r=s.update(c,i);
    assert(!r.active && r.qhat_1f_applied==0. && r.coeff_cos_1f==0.);
    i=validInput(); i.amplitude_bound=.01;
    for(int k=0;k<5000;++k) {
        r=excite(s,c,i,1);
        assert(std::isfinite(r.coeff_cos_1f) && std::hypot(r.coeff_cos_1f,r.coeff_sin_1f)<=.01);
    }
    std::puts("PFF_AMPLITUDE_FAULT_CONTAINMENT=PASS");
    // Numerical state corruption must reset immediately, even if phase drops.
    double corrupt=NAN;
    std::memcpy(&s,&corrupt,sizeof(corrupt));
    i=validInput();i.phase_valid=false;r=s.update(c,i);
    assert(r.reset_reason==ResetReason::NONFINITE_STATE && r.qhat_1f_applied==0.);

    PitchFfPhaseShaper half, full; Config h{.5,1.}, f{1.,1.};
    i=validInput(); Input j=i;
    auto a=excite(half,h,i),b=excite(full,f,j);
    assert(a.coeff_cos_1f==b.coeff_cos_1f && a.coeff_cos_2f_internal==b.coeff_cos_2f_internal);
    assert(std::abs(a.qhat_1f_applied-.5*b.qhat_1f_applied)<1e-15);
    assert(a.qhat_1f_applied==a.alpha_effective*a.qhat_1f);
    full.reset();j=validInput();auto repeated=excite(full,f,j);
    assert(repeated.coeff_cos_1f==b.coeff_cos_1f && repeated.qhat_1f_applied==b.qhat_1f_applied);
    std::puts("PFF_ALPHA_AND_RESET_DETERMINISM=PASS");
    const auto correction=mapCorrection(b, .3, .8, 1.44);
    assert(std::abs(correction.ff_delta_rate_control+.3*b.qhat_1f_applied)<1e-15);
    assert(std::abs(correction.normalized_correction-.8*1.44*correction.ff_delta_rate_control)<1e-15);
    assert(mapCorrection(b, NAN, .8, 1.44).normalized_correction==0.);
    assert(mapCorrection(b, .3, .8, INFINITY).normalized_correction==0.);
    std::printf("PITCH_FF_HELPER_BYTES=%zu\nPITCH_PFF_HOST_TESTS=PASS\n",sizeof(s));
}
