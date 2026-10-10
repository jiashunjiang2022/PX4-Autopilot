#include "src/modules/fw_rate_control/PitchFfPhaseShaper.hpp"
#include <iostream>
#include <iomanip>
#include <string>
using namespace pitch_ff_phase_shaper;
int main()
{
    PitchFfPhaseShaper shaper;
    std::cout << std::setprecision(17);
    std::string q, p, f, dt;
    int reset, valid, enable;
    double alpha, adapt;
    while (std::cin >> reset >> q >> p >> valid >> f >> dt >> enable >> alpha >> adapt) {
        if(reset) shaper.reset();
        Input in{};
        in.enable=enable;in.armed=in.supported=in.mission=in.controller_active=true;
        in.phase_fresh=true;in.phase_valid=valid;in.amplitude_bound=4.;
        in.q_sp=std::stod(q);in.phase_rad=std::stod(p);in.flap_frequency_hz=std::stod(f);in.dt=std::stod(dt);
        auto r=shaper.update({alpha,adapt},in);
        std::cout<<r.coeff_cos_1f<<','<<r.coeff_sin_1f<<','<<r.coeff_cos_2f_internal<<','<<r.coeff_sin_2f_internal<<','
          <<r.baseline<<','<<r.qhat_1f_reference<<','<<r.qhat_2f_reference_internal<<','<<r.qhat_1f_applied<<','
          <<r.estimation_error<<','<<alpha*r.fade<<','<<r.fade<<','<<r.warm_cycles<<','
          <<r.warmup_complete<<','<<r.active<<','<<r.fading<<','<<static_cast<int>(r.reset_reason)<<'\n';
    }
}
