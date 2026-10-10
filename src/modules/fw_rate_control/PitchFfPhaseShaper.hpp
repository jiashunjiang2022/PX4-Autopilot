#pragma once

#include <cmath>
#include <cstdint>
#include <float.h>

namespace pitch_ff_phase_shaper {

enum class ResetReason : uint8_t {
	NONE, DISABLED, DISARMED, UNSUPPORTED, NOT_MISSION, LANDED, CONTROLLER_INACTIVE,
	PHASE_INVALID, PHASE_STALE, NONFINITE_Q_SP, NONFINITE_PHASE, INVALID_FREQUENCY,
	INVALID_DT, INVALID_CONFIG, NONFINITE_STATE
};

// Binary64 arithmetic preserves the authoritative Python winner, including
// the normalization epsilon. This is an O(1), 11-state estimator.
struct Config { double alpha{0.5}; double adapt_cycles{1.}; };

struct Input {
	bool enable{false}; bool armed{false}; bool supported{false}; bool mission{false};
	bool landed{false}; bool controller_active{false}; bool phase_valid{false};
	bool phase_fresh{false}; bool numeric_inputs_valid{true};
	double dt{0.}; double phase_rad{0.}; double flap_frequency_hz{0.}; double q_sp{0.};
	double amplitude_bound{1.};
};

struct Result {
	double coeff_cos_1f{0.}, coeff_sin_1f{0.};
	double coeff_cos_2f_internal{0.}, coeff_sin_2f_internal{0.};
	double baseline{0.};
	// Valid qhat is unfaded: applied = alpha_effective * qhat_1f.
	// During dropout qhat is the reference's slewed residual and
	// alpha_effective is alpha, so the exact residual is applied once.
	double qhat_1f{0.}, qhat_2f_internal{0.}, qhat_1f_applied{0.};
	double qhat_1f_reference{0.}, qhat_2f_reference_internal{0.};
	double estimation_error{0.}, alpha_effective{0.}, fade{0.}, warm_cycles{0.};
	bool active{false}, warmup_complete{false}, fading{false};
	ResetReason reset_reason{ResetReason::NONE};
};

struct Correction {
	double ff_delta_rate_control{0.};
	double normalized_correction{0.};
};

struct OutputMapping {
	bool valid{false};
	double ff_delta_rate_control{0.};
	double normalized_correction{0.};
	double shaped_pitch_control{0.};
};

inline bool phaseFresh(uint64_t phase_timestamp, uint64_t now_timestamp)
{
	return phase_timestamp != 0 && now_timestamp >= phase_timestamp &&
		(now_timestamp - phase_timestamp) <= 50000;
}

inline Correction mapCorrection(const Result &r, double effective_ff_gain, double gain_compression_pitch,
				       double airspeed_scaling_sq)
{
	Correction out{};
	if (std::isfinite(effective_ff_gain) && std::isfinite(gain_compression_pitch) &&
		std::isfinite(airspeed_scaling_sq) && std::isfinite(r.qhat_1f_applied)) {
		out.ff_delta_rate_control = -effective_ff_gain * r.qhat_1f_applied;
		out.normalized_correction = gain_compression_pitch * airspeed_scaling_sq * out.ff_delta_rate_control;
		if (!std::isfinite(out.normalized_correction) || !std::isfinite(out.ff_delta_rate_control)) { out = {}; }
	}
	return out;
}

inline OutputMapping mapPitchOutput(const Result &r, double effective_ff_gain, double gain_compression_pitch,
					 double airspeed_scaling_sq, double native_pitch_control, double trim = 0.)
{
	OutputMapping out{};
	if (!std::isfinite(native_pitch_control) || !std::isfinite(trim) || !std::isfinite(effective_ff_gain)
	    || !std::isfinite(gain_compression_pitch) || !std::isfinite(airspeed_scaling_sq)
	    || !std::isfinite(r.qhat_1f_applied)) { return out; }
	out.ff_delta_rate_control = -effective_ff_gain * r.qhat_1f_applied;
	out.normalized_correction = gain_compression_pitch * airspeed_scaling_sq * out.ff_delta_rate_control;
	out.shaped_pitch_control = native_pitch_control + out.normalized_correction;
	constexpr double limit = static_cast<double>(FLT_MAX);
	out.valid = std::isfinite(out.ff_delta_rate_control) && std::fabs(out.ff_delta_rate_control) <= limit
		    && std::isfinite(out.normalized_correction) && std::fabs(out.normalized_correction) <= limit
		    && std::isfinite(out.shaped_pitch_control) && std::fabs(out.shaped_pitch_control) <= limit
		    && std::fabs(out.shaped_pitch_control + trim) <= limit;
	if (!out.valid) { out = {}; }
	return out;
}

class PitchFfPhaseShaper {
public:
	void reset() { _a1=_b1=_a2=_b2=_baseline=0.; _previous_phase=NAN; _warm_cycles=0.;
		_fade=0.; _correction1=_correction2=0.; _was_valid=false; }

	Result update(const Config &config, const Input &in)
	{
		if (!validConfig(config) || !std::isfinite(in.amplitude_bound) || in.amplitude_bound<=0.) {
			return hardReset(ResetReason::INVALID_CONFIG);
		}
		if (!stateSane(in.amplitude_bound)) return hardReset(ResetReason::NONFINITE_STATE);
		ResetReason reason = ResetReason::NONE;
		if (!in.enable) reason=ResetReason::DISABLED;
		else if (!in.armed) reason=ResetReason::DISARMED;
		else if (!in.supported) reason=ResetReason::UNSUPPORTED;
		else if (!in.mission) reason=ResetReason::NOT_MISSION;
		else if (in.landed) reason=ResetReason::LANDED;
		else if (!in.controller_active) reason=ResetReason::CONTROLLER_INACTIVE;
		else if (!in.numeric_inputs_valid || !std::isfinite(in.q_sp)) reason=ResetReason::NONFINITE_Q_SP;
		else if (!std::isfinite(in.phase_rad)) reason=ResetReason::NONFINITE_PHASE;
		else if (!std::isfinite(in.flap_frequency_hz) || in.flap_frequency_hz <= .5) reason=ResetReason::INVALID_FREQUENCY;
		else if (!std::isfinite(in.dt) || !(in.dt > 0.) || in.dt > .1) reason=ResetReason::INVALID_DT;
		else if (!in.phase_valid) reason=ResetReason::PHASE_INVALID;
		else if (!in.phase_fresh) reason=ResetReason::PHASE_STALE;
		if (reason == ResetReason::PHASE_INVALID || reason == ResetReason::PHASE_STALE) {
			return invalidResult(reason, in.dt, config.alpha);
		}
		if (reason != ResetReason::NONE) return hardReset(reason);

		if (_was_valid && std::isfinite(_previous_phase) && in.phase_rad < _previous_phase - 3.14159265358979323846) _warm_cycles += 1.;
		_previous_phase=in.phase_rad; _was_valid=true;
		const double tau=::fmax(config.adapt_cycles/in.flap_frequency_hz, in.dt);
		const double c1=std::cos(in.phase_rad), s1=std::sin(in.phase_rad);
		const double c2=std::cos(2.*in.phase_rad), s2=std::sin(2.*in.phase_rad);
		const double beta0=1.-std::exp(-in.dt/::fmax(.5,2./in.flap_frequency_hz));
		_baseline += beta0*(in.q_sp-_baseline);
		const double estimate=_a1*c1+_b1*s1+_a2*c2+_b2*s2;
		const double error=in.q_sp-_baseline-estimate;
		const double mu=::fmin(.25,2.*in.dt/tau);
		const double leak=in.dt/::fmax(50.*tau,1e-6);
		constexpr double norm=2.+1e-9;
		_a1=(1.-leak)*_a1+mu*error*c1/norm; _b1=(1.-leak)*_b1+mu*error*s1/norm;
		_a2=(1.-leak)*_a2+mu*error*c2/norm; _b2=(1.-leak)*_b2+mu*error*s2/norm;
		if (_warm_cycles >= 1.) _fade=::fmin(1.,_fade+in.dt*in.flap_frequency_hz);
		else _fade=0.;
		const double q1_unfaded=_a1*c1+_b1*s1;
		const double q2_unfaded=_a2*c2+_b2*s2;
		const double alpha_effective=config.alpha*_fade;
		_correction1=_fade*q1_unfaded; _correction2=_fade*q2_unfaded;
		if (!stateSane(in.amplitude_bound)) return hardReset(ResetReason::NONFINITE_STATE);
		Result out=makeResult(error, true, ResetReason::NONE, config.alpha);
		out.qhat_1f=q1_unfaded; out.qhat_2f_internal=q2_unfaded;
		out.qhat_1f_applied=alpha_effective*q1_unfaded; out.alpha_effective=alpha_effective;
		out.fading=(_fade>0. && _fade<1.); return out;
	}

	static bool validConfig(const Config &c) { return std::isfinite(c.alpha) && c.alpha>=0. && c.alpha<=1. &&
		std::isfinite(c.adapt_cycles) && c.adapt_cycles>=1. && c.adapt_cycles<=8.; }

private:
	double _a1{0.},_b1{0.},_a2{0.},_b2{0.},_baseline{0.},_previous_phase{NAN};
	double _warm_cycles{0.},_fade{0.},_correction1{0.},_correction2{0.}; bool _was_valid{false};
	bool stateSane(double bound) const {
		return std::isfinite(_a1)&&std::isfinite(_b1)&&std::isfinite(_a2)&&std::isfinite(_b2)&&
			std::isfinite(_baseline)&&std::isfinite(_warm_cycles)&&std::isfinite(_fade)&&
			std::isfinite(_correction1)&&std::isfinite(_correction2)&&
			(!_was_valid||std::isfinite(_previous_phase))&&
			::hypot(_a1,_b1)<=bound&&::hypot(_a2,_b2)<=bound&&std::fabs(_baseline)<=bound;
	}
	Result makeResult(double error,bool valid,ResetReason reason,double alpha) const {
		Result r{}; r.coeff_cos_1f=_a1;r.coeff_sin_1f=_b1;r.coeff_cos_2f_internal=_a2;r.coeff_sin_2f_internal=_b2;
		r.baseline=_baseline;r.estimation_error=error;r.fade=_fade;r.warm_cycles=_warm_cycles;
		r.warmup_complete=_warm_cycles>=1.;r.active=valid&&r.warmup_complete&&_fade>0.;r.reset_reason=reason;
		r.qhat_1f=_correction1;r.qhat_2f_internal=_correction2;r.alpha_effective=alpha*_fade;r.qhat_1f_applied=alpha*_correction1;
		r.qhat_1f_reference=_correction1;r.qhat_2f_reference_internal=_correction2;
		return r;
	}
	Result hardReset(ResetReason reason) { reset(); return makeResult(0.,false,reason,0.); }
	Result invalidResult(ResetReason reason,double dt,double alpha) {
		const double step=.8*dt;
		_correction1 -= ::fmax(-step,::fmin(step,_correction1)); _correction2 -= ::fmax(-step,::fmin(step,_correction2));
		_a1=_b1=_a2=_b2=_baseline=0.; _previous_phase=NAN; _warm_cycles=0.; _fade=0.; _was_valid=false;
		Result r=makeResult(0.,false,reason,alpha); r.alpha_effective=alpha;
		r.qhat_1f_applied=r.alpha_effective*r.qhat_1f;
		r.fading=std::fabs(_correction1)>0.||std::fabs(_correction2)>0.; return r;
	}
};
}
