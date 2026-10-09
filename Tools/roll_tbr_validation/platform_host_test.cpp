#include "src/drivers/encoder/as5600/AS5600Math.hpp"
#include "src/modules/wing_phase/WingPhaseMath.hpp"
#include "src/modules/fw_lateral_longitudinal_control/FwControlHandover.hpp"
#include <cassert>
#include <limits>
#include <iostream>

int main()
{
    assert(as5600::flap_frequency_from_rpm(480.f, 8.f) == 1.f);
    assert(as5600::flap_frequency_from_rpm(-480.f, 8.f) == 1.f);
    assert(as5600::flap_frequency_from_rpm(0.f, 8.f) == 0.f);
    for (float v : {0.f, -1.f, NAN, INFINITY}) {
        assert(std::isnan(as5600::flap_frequency_from_rpm(480.f, v)));
    }
    assert(std::isnan(as5600::flap_frequency_from_rpm(INFINITY, 8.f)));
    assert(!wing_phase::compute_phase(100, 0, 32768.f, false).valid);
    assert(!wing_phase::compute_phase(100, 0, 0.f, true).valid);
    auto phase = wing_phase::compute_phase(8192, 0, 32768.f, true);
    assert(phase.valid && std::fabs(phase.phase_rad - 1.5707963f) < 1e-6f);
    phase = wing_phase::compute_phase(-8192, 0, 32768.f, true);
    assert(phase.valid && std::fabs(phase.phase_rad - 4.712389f) < 1e-6f);
    const wing_phase::EncoderSample previous{1000, 10}, current{2000, 30};
    auto interpolation = wing_phase::interpolate_count_at_timestamp(previous, current, 1500);
    assert(interpolation.valid && interpolation.total_count == 20);
    assert(!wing_phase::interpolate_count_at_timestamp(previous, current, 999).valid);
    assert(!wing_phase::interpolate_count_at_timestamp(current, previous, 1500).valid);

    fw_control::AutoControlHandover h;
    fixed_wing_longitudinal_setpoint_s idle{};
    idle.timestamp = 100;
    idle.altitude = idle.height_rate = idle.equivalent_airspeed = NAN;
    assert(fw_control::isIdleLongitudinalSetpoint(idle));
    auto fresh = idle;
    fresh.altitude = 40.f;
    h.updateControlMode(false, 90);
    assert(!h.pending());
    h.updateControlMode(true, 100);
    assert(h.pending());
    assert(!h.acceptSetpoint(false, fresh));
    fresh.timestamp = 99;
    assert(!h.acceptSetpoint(true, fresh));
    idle.timestamp = 1000000000; // No timeout releases an idle setpoint.
    assert(!h.acceptSetpoint(true, idle) && h.pending());
    fresh.timestamp = 100; // Equal timestamp is fresh under donor semantics.
    assert(h.acceptSetpoint(true, fresh) && !h.pending());
    h.updateControlMode(true, 200); // Auto-to-auto does not rearm.
    assert(!h.pending() && h.acceptSetpoint(true, idle));
    h.updateControlMode(false, 300);
    h.updateControlMode(true, 400);
    assert(h.pending() && !h.acceptSetpoint(true, fresh));
    fresh = idle;
    fresh.timestamp = 400;
    fresh.pitch_direct = 0.2f;
    assert(h.acceptSetpoint(true, fresh));
    h.updateControlMode(false, 500);
    h.updateControlMode(true, 600);
    h.updateControlMode(false, 601);
    assert(!h.pending());
    std::cout << "PLATFORM_HOST_TESTS=PASS\n";
}
