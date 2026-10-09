#include "src/modules/fw_rate_control/RollTbr.hpp"
#include "src/lib/rate_control/rate_control.hpp"
#include <cassert>
#include <complex>
#include <iostream>
#include <iomanip>
#include <limits>

using namespace roll_tbr;

static Input eligible(float error = 1.f, float dt = 0.01f)
{
    Input i{};
    i.enable = 1;
    i.armed = i.supported = i.mission = i.controller_active = i.inputs_valid = true;
    i.landed = false;
    i.dt = i.measured_dt = dt;
    i.setpoint = error;
    return i;
}

static void zero(const Result &r)
{
    assert(!r.active && r.output == 0.f && r.filtered == 0.f && r.raw == 0.f && r.capped == 0.f);
}

static void failAndReenter(RollTbr &t, const Config &c, Input bad, ResetReason reason)
{
    for (int j = 0; j < 1000; ++j) { assert(t.update(c, eligible(10)).active); }
    const auto failed = t.update(c, bad);
    zero(failed);
    assert(failed.reset_reason == reason);
    RollTbr fresh;
    const auto a = t.update(c, eligible()), b = fresh.update(c, eligible());
    assert(a.output == b.output && a.filtered == b.filtered);
}

static void coreTests()
{
    Config c{};
    RollTbr t;
    auto i = eligible();
    i.enable = 0;
    failAndReenter(t, c, i, ResetReason::Disabled);
    i = eligible(); i.armed = false;
    failAndReenter(t, c, i, ResetReason::Disarmed);
    i = eligible(); i.supported = false;
    failAndReenter(t, c, i, ResetReason::Unsupported);
    i = eligible(); i.mission = false;
    failAndReenter(t, c, i, ResetReason::NotMission);
    i = eligible(); i.landed = true;
    failAndReenter(t, c, i, ResetReason::Landed);
    i = eligible(); i.controller_active = false;
    failAndReenter(t, c, i, ResetReason::ControllerInactive);
    i = eligible(); i.integral_reset = true;
    failAndReenter(t, c, i, ResetReason::IntegralReset);
    i = eligible(); i.inputs_valid = false;
    failAndReenter(t, c, i, ResetReason::NonfiniteInput);
    for (float bad : {0.f, -0.01f, 0.001f, 0.041f, NAN, INFINITY}) {
        i = eligible(); i.dt = bad;
        failAndReenter(t, c, i, ResetReason::InvalidDt);
        i = eligible(); i.measured_dt = bad;
        failAndReenter(t, c, i, ResetReason::InvalidDt);
    }
    for (float bad : {NAN, INFINITY, -INFINITY}) {
        i = eligible(); i.setpoint = bad;
        failAndReenter(t, c, i, ResetReason::NonfiniteInput);
        i = eligible(); i.rate = bad;
        failAndReenter(t, c, i, ResetReason::NonfiniteInput);
    }
    i = eligible(std::numeric_limits<float>::max()); i.rate = -i.setpoint;
    failAndReenter(t, c, i, ResetReason::NonfiniteInput);
    i = eligible(); i.enable = 2;
    zero(t.update(c, i));
    assert(t.update(c, i).reset_reason == ResetReason::InvalidConfig);
    for (int field = 0; field < 4; ++field) {
        for (float bad : {-1.f, NAN, INFINITY, 100.f}) {
            Config invalid = c;
            switch (field) {
            case 0: invalid.fc = bad; break;
            case 1: invalid.k = bad; break;
            case 2: invalid.max = bad; break;
            case 3: invalid.slew = bad; break;
            }
            const auto r = t.update(invalid, eligible());
            zero(r);
            assert(!r.config_valid && r.reset_reason == ResetReason::InvalidConfig);
        }
    }
    for (int field : {0, 2, 3}) {
        Config invalid = c;
        if (field == 0) { invalid.fc = 0.f; }
        if (field == 2) { invalid.max = 0.f; }
        if (field == 3) { invalid.slew = 0.f; }
        zero(t.update(invalid, eligible()));
    }
    c.k = 0.f;
    assert(t.update(c, eligible()).active && t.update(c, eligible()).output == 0.f);
    c = Config{};
    t.reset();
    for (int j = 0; j < 1000; ++j) { assert(t.update(c, eligible(0)).output == 0.f); }
    Result converged{};
    for (int j = 0; j < 3000; ++j) { converged = t.update(c, eligible()); }
    assert(std::fabs(converged.filtered - 1.f) < 2e-6f);
    assert(std::fabs(converged.output - 0.01f) < 2e-8f);
    c.k = 0.02f;
    auto changed = t.update(c, eligible());
    assert(changed.reset_reason == ResetReason::ConfigChanged);
    RollTbr fresh;
    auto first = fresh.update(c, eligible());
    assert(changed.filtered == first.filtered && changed.output == first.output);

    c = Config{};
    RollTbr pos, neg;
    float previous = 0.f;
    bool cap = false, slew = false, crossed = false;
    for (int j = 0; j < 4000; ++j) {
        const float dt = (j % 3 == 0) ? 0.002f : ((j % 3 == 1) ? 0.04f : 0.01f);
        const float e = j < 2000 ? 20.f : -20.f;
        const auto a = pos.update(c, eligible(e, dt));
        const auto b = neg.update(c, eligible(-e, dt));
        assert(a.active && b.active);
        assert(a.output == -b.output && a.filtered == -b.filtered && a.capped == -b.capped);
        assert(std::fabs(a.capped) <= c.max && std::fabs(a.output) <= c.max + 1e-7f);
        assert(std::fabs(a.output - previous) <= c.slew * dt + 1e-8f);
        cap |= a.cap_active; slew |= a.slew_active;
        crossed |= j > 2000 && a.output < 0.f;
        previous = a.output;
    }
    assert(cap && slew && crossed && std::fabs(previous + c.max) < 1e-6f);
    std::cerr << "TBR_CORE_GATING_RESET_CAP_SLEW_SYMMETRY=PASS\n";
}

static void disabledParity()
{
    using matrix::Vector3f;
    RateControl baseline, candidate;
    RollTbr t;
    const Config config{};
    for (RateControl *r : {&baseline, &candidate}) {
        r->setPidGains(Vector3f(.2f, .3f, .4f), Vector3f(.1f, .12f, .15f), Vector3f(.05f, .03f, .02f));
        r->setIntegratorLimit(Vector3f(.3f, .4f, .5f));
        r->setFeedForwardGain(Vector3f(.1f, .15f, .2f));
    }
    for (int j = 0; j < 10000; ++j) {
        const float n = static_cast<float>(j);
        const Vector3f rate(std::sin(n*.01f), std::cos(n*.02f), std::sin(n*.03f));
        const Vector3f sp(std::sin(n*.007f)*10.f, .5f, -.5f);
        const Vector3f accel(.2f, -.3f, .4f);
        const float dt = .002f + static_cast<float>(j % 39)*.001f;
        for (RateControl *r : {&baseline, &candidate}) {
            r->setPositiveSaturationFlag(0, j % 7 == 0);
            r->setNegativeSaturationFlag(1, j % 11 == 0);
            if (j % 100 == 0) { r->resetIntegral(); }
        }
        const auto a = baseline.update(rate, sp, accel, dt, j % 17 == 0);
        const auto b = candidate.update(rate, sp, accel, dt, j % 17 == 0);
        auto i = eligible(sp(0)-rate(0), dt); i.enable = 0;
        const auto residual = t.update(config, i);
        zero(residual);
        const float scale = .2f + static_cast<float>(j % 20)*.1f;
        const Vector3f gain(.9f, .8f, .7f), trim(.1f, -.2f, .05f);
        const auto native_a = gain.emult(a*scale*scale);
        const auto native_b = gain.emult(b*scale*scale);
        auto out_a = matrix::constrain(native_a + trim, -1.f, 1.f);
        auto out_b = matrix::constrain(native_b + trim, -1.f, 1.f);
        if (residual.active && std::fabs(residual.output) > 0.f) {
            out_b(0, 0) = math::constrain(native_b(0, 0) + trim(0) + residual.output, -1.f, 1.f);
        }
        out_a(2, 0) = math::constrain(out_a(2, 0) + .2f*out_a(0, 0), -1.f, 1.f);
        out_b(2, 0) = math::constrain(out_b(2, 0) + .2f*out_b(0, 0), -1.f, 1.f);
        for (int axis = 0; axis < 3; ++axis) {
            assert(std::memcmp(&out_a(axis, 0), &out_b(axis, 0), sizeof(float)) == 0);
        }
        rate_ctrl_status_s ia{}, ib{};
        baseline.getRateControlStatus(ia); candidate.getRateControlStatus(ib);
        assert(ia.rollspeed_integ == ib.rollspeed_integ && ia.pitchspeed_integ == ib.pitchspeed_integ
               && ia.yawspeed_integ == ib.yawspeed_integ);
    }
    // Native signed zero and nonfinite paths must never perform a disabled addition.
    for (float native : {-0.f, 0.f, NAN, INFINITY, -INFINITY}) {
        const float before = native;
        auto i = eligible(); i.enable = 0;
        const auto r = t.update(config, i);
        if (r.active && std::fabs(r.output) > 0.f) { native += r.output; }
        assert(std::memcmp(&before, &native, sizeof(float)) == 0);
    }
    std::cerr << "TBR_DISABLED_NATIVE_PARITY_10000_CYCLES=PASS\n";
}

static void frequencyResponse()
{
    std::cout << "dt_s,frequency_hz,measured_gain,analytical_gain,phase_deg,analytical_phase_deg\n";
    constexpr double pi = 3.14159265358979323846;
    for (float dt : {.002f, .004f, .01f, .02f, .04f}) {
        for (double f : {.0625, .125, .25, .375, .625, 1., 2., 3., 3.868, 4.4375, 5.}) {
            RollTbr core;
            const Config c{};
            const int warmup = static_cast<int>(20./dt);
            // Least-squares fit handles noninteger samples per period without DFT leakage.
            const int count = static_cast<int>(std::ceil(8./(f*dt)));
            double ss=0, cc=0, sc=0, ys=0, yc=0;
            for (int j = -warmup; j < count; ++j) {
                const double angle = 2*pi*f*j*dt;
                const double s=std::sin(angle), co=std::cos(angle);
                const auto r = core.update(c, eligible(static_cast<float>(s)*.001f, dt));
                assert(r.active && !r.cap_active && !r.slew_active);
                if (j >= 0) {
                    ss+=s*s; cc+=co*co; sc+=s*co;
                    ys+=r.filtered*s/.001; yc+=r.filtered*co/.001;
                }
            }
            const double determinant=ss*cc-sc*sc;
            const double a=(ys*cc-yc*sc)/determinant, b=(yc*ss-ys*sc)/determinant;
            const double gain=std::hypot(a,b), phase=std::atan2(b,a)*180/pi;
            const double alpha=dt/(1./(2*pi*c.fc)+dt);
            const auto h=alpha/(1.-(1.-alpha)*std::exp(std::complex<double>(0,-2*pi*f*dt)));
            assert(std::fabs(gain-std::abs(h)) < 2e-5);
            assert(std::fabs(phase-std::arg(h)*180/pi) < .003);
            std::cout << std::setprecision(9) << dt << ',' << f << ',' << gain << ',' << std::abs(h)
                      << ',' << phase << ',' << std::arg(h)*180/pi << '\n';
        }
    }
    std::cerr << "TBR_FREQUENCY_RESPONSE_55_CASES=PASS\n";
}

int main()
{
    coreTests();
    disabledParity();
    frequencyResponse();
    std::cerr << "TBR_HOST_TESTS=PASS\n";
}
