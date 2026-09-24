#pragma once

#include <cmath>

class FastFFPlus
{
public:
	static constexpr float DELTA_MIN = -0.2f;
	static constexpr float DELTA_MAX = 0.2f;

	struct Result { bool enabled{}, active{}, valid{}; float base{}, effective{}; };

	static Result evaluate(bool enabled, bool mission_state, float base, float delta)
	{
		Result r{};
		r.enabled = enabled;
		r.base = base;
		r.valid = std::isfinite(base) && std::isfinite(delta) && delta >= DELTA_MIN && delta <= DELTA_MAX
			   && std::isfinite(base + delta);
		r.active = enabled && mission_state && r.valid;
		r.effective = r.active ? base + delta : base;
		return r;
	}
};
