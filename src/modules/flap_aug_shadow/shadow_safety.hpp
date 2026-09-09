#pragma once

namespace flap_aug_shadow
{

struct ActualInjection final {
	static constexpr int SlowMicrounits = 0;
	static constexpr int FastRollMicrounits = 0;
	static constexpr int FastPitchMicrounits = 0;
	static constexpr float Slow = static_cast<float>(SlowMicrounits) * 1e-6f;
	static constexpr float FastRoll = static_cast<float>(FastRollMicrounits) * 1e-6f;
	static constexpr float FastPitch = static_cast<float>(FastPitchMicrounits) * 1e-6f;
};

static_assert(ActualInjection::SlowMicrounits == 0);
static_assert(ActualInjection::FastRollMicrounits == 0);
static_assert(ActualInjection::FastPitchMicrounits == 0);

} // namespace flap_aug_shadow
