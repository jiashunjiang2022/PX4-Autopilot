#pragma once

#include <cmath>
#include <float.h>

namespace flap_aug_shadow
{

struct V4ShadowResult final {
	bool enabled{false};
	bool config_mapping_valid{false};
	bool tail_roll_signal_valid{false};
	float b_prior{0.f};
	bool b_prior_valid{false};
	float delta_b_shadow{0.f};
	float u_trim_candidate_shadow{0.f};
	float u_controller_eq{0.f};
	bool u_controller_eq_valid{false};
	float tail_roll_realized{0.f};
	float tail_pitch_realized{0.f};
	float tail_yaw_realized{0.f};
	float roll_torque_equiv_prealloc{0.f};
	float allocation_consistency_error{0.f};
	bool zero_aug_mapping_valid{false};
	bool nonzero_aug_mapping_valid{false};
	float u_aug_eq_shadow{0.f};
	float maneuver_hat{0.f};
	bool maneuver_hat_valid{false};
	float r_total_proxy_shadow{0.f};
	bool r_total_proxy_valid{false};
	float innovation_shadow{0.f};
	bool total_mapping_valid{false};
	bool delta_update_implemented{false};
};

class V4ShadowState final
{
public:
	static constexpr float StorageSanityBound = 2.f;

	void reset() { _delta_b_shadow = 0.f; }

	V4ShadowResult update(bool enabled, float configured_b_prior, bool config_mapping_valid,
			     float c0, float c1, float c2, float roll_torque_setpoint,
			     bool maneuver_valid, float maneuver_hat,
			     float actual_slow_injection, float actual_fast_roll_injection,
			     float actual_fast_pitch_injection)
	{
		V4ShadowResult result{};
		result.enabled = enabled;
		result.config_mapping_valid = config_mapping_valid;
		result.tail_roll_signal_valid = std::isfinite(c0) && std::isfinite(c1);
		result.b_prior_valid = std::isfinite(configured_b_prior)
				       && fabsf(configured_b_prior) <= StorageSanityBound;
		result.b_prior = result.b_prior_valid ? configured_b_prior : 0.f;
		result.delta_b_shadow = std::isfinite(_delta_b_shadow) ? _delta_b_shadow : 0.f;
		result.u_trim_candidate_shadow = constrain_storage(result.b_prior + result.delta_b_shadow);
		result.tail_roll_realized = result.tail_roll_signal_valid ? (c1 - c0) * 0.5f : 0.f;
		result.tail_pitch_realized = result.tail_roll_signal_valid ? (c0 + c1) * 0.5f : 0.f;
		result.tail_yaw_realized = std::isfinite(c2) ? c2 : 0.f;
		result.u_controller_eq = result.tail_roll_realized;
		result.roll_torque_equiv_prealloc = std::isfinite(roll_torque_setpoint) ? roll_torque_setpoint / 1.1f : 0.f;
		result.allocation_consistency_error = result.tail_roll_signal_valid
							? result.tail_roll_realized - result.roll_torque_equiv_prealloc : 0.f;
		result.maneuver_hat_valid = maneuver_valid && std::isfinite(maneuver_hat);
		result.maneuver_hat = result.maneuver_hat_valid ? maneuver_hat : 0.f;
		result.u_controller_eq_valid = result.config_mapping_valid && result.tail_roll_signal_valid;
		const auto zero_injection = [](float value) {
			return std::isfinite(value) && fabsf(value) <= FLT_EPSILON;
		};
		result.zero_aug_mapping_valid = result.config_mapping_valid && result.tail_roll_signal_valid
							&& zero_injection(actual_slow_injection)
							&& zero_injection(actual_fast_roll_injection)
							&& zero_injection(actual_fast_pitch_injection);
		result.nonzero_aug_mapping_valid = false;
		result.u_aug_eq_shadow = 0.f;
		result.r_total_proxy_valid = result.zero_aug_mapping_valid && result.maneuver_hat_valid;
		result.r_total_proxy_shadow = result.r_total_proxy_valid
							? result.u_controller_eq - result.maneuver_hat : 0.f;
		result.innovation_shadow = 0.f;
		result.total_mapping_valid = result.zero_aug_mapping_valid;
		result.delta_update_implemented = false;
		return result;
	}

private:
	static float constrain_storage(float value)
	{
		if (!std::isfinite(value)) {
			return 0.f;
		}

		return value < -StorageSanityBound ? -StorageSanityBound
		       : (value > StorageSanityBound ? StorageSanityBound : value);
	}

	float _delta_b_shadow{0.f};
};

} // namespace flap_aug_shadow
