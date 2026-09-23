#pragma once

#include <cmath>
#include <cstdint>

// Control state only. Never writes predictor history, native I, Slow or compression.
class FastControl
{
public:
	struct Config {
		bool enabled{false};
		float k{1.f}, max{.005f}, on{.15f}, off{.18f}, slew{.05f};
	};
	struct Frame {
		float prediction{0.f}, t{0.f};
		uint64_t timestamp{0};
		uint32_t seq{0};
		bool valid{false};
	};
	struct Decision {
		float prediction{}, raw{}, saturated{}, model_t{}, live_t{}, model_gate{}, live_gate{}, gated{}, bounded{}, target{}, final{}, limit{};
		uint64_t age{};
		uint32_t seq{};
		uint64_t decision_time{}, frame_time{};
		float dt{};
		bool enabled{}, valid{}, nonfinite{}, stale{}, reset{}, domain{}, state{};
	};
	void invalidate()
	{
		_frame.valid = false;
		_ready_count = 0;
		_actual = 0.f;
	}
	void complete(const Frame &frame, float live_t, uint64_t now, bool new_frame)
	{
		_live = live_t;
		_live_time = now;
		if (new_frame) {
			_frame = frame;
			if (!frame.valid || !std::isfinite(frame.prediction) || !std::isfinite(frame.t)) {
				invalidate();
			} else if (_ready_count < 9) {
				++_ready_count;
			}
		}
	}
	Decision step(const Config &c, uint64_t now, float dt, bool state_ok, bool reset)
	{
		Decision d{};
		d.decision_time = now;
		d.frame_time = _frame.timestamp;
		d.dt = dt;
		d.enabled = c.enabled;
		d.state = !state_ok;
		d.reset = reset;
		d.prediction = _frame.prediction;
		d.model_t = _frame.t;
		d.live_t = _live;
		d.seq = _frame.seq;
		d.age = now >= _frame.timestamp ? now - _frame.timestamp : UINT64_MAX;
		d.stale = d.age > 100000 || now < _live_time || now - _live_time > 100000;
		const bool config_ok = std::isfinite(c.k) && c.k >= 0.f && c.k <= 1.f
				       && std::isfinite(c.max) && c.max >= 0.f && c.max <= .005f
				       && std::isfinite(c.on) && std::isfinite(c.off) && c.on >= 0.f
				       && c.on <= .15f && c.off <= .18f && c.on < c.off
				       && std::isfinite(c.slew) && c.slew >= 0.f && c.slew <= .05f;
		d.nonfinite = !std::isfinite(_frame.prediction) || !std::isfinite(_frame.t)
			      || !std::isfinite(_live) || !std::isfinite(dt);
		if (!c.enabled || !state_ok || reset) {
			invalidate();
			return d;
		}
		if (!config_ok || d.nonfinite || d.stale || dt <= 0.f || dt > .1f) {
			invalidate();
			return d;
		}
		if (!_frame.valid || _ready_count < 9) {
			_actual = 0.f;
			return d;
		}
		d.model_gate = gate(_frame.t, c.on, c.off);
		d.live_gate = gate(_live, c.on, c.off);
		d.raw = c.k * _frame.prediction;
		if (!std::isfinite(d.raw)) {
			d.nonfinite = true;
			_actual = 0.f;
			return d;
		}
		d.saturated = clamp(d.raw, -c.max, c.max);
		d.limit = c.max * minimum(d.model_gate, d.live_gate);
		d.gated = d.saturated * minimum(d.model_gate, d.live_gate);
		d.domain = d.model_gate <= 0.f || d.live_gate <= 0.f;
		if (d.domain) {
			_actual = 0.f;
			return d;
		}
		const float lo = maximum(-.20f - _frame.t, -.20f - _live);
		const float hi = minimum(.20f - _frame.t, .20f - _live);
		d.bounded = clamp(d.gated, lo, hi);
		d.target = d.bounded;
		_actual += clamp(d.target - _actual, -c.slew * dt, c.slew * dt);
		// Safety contraction overrides slew: never retain excess authority on a fading gate.
		_actual = clamp(clamp(_actual, -d.limit, d.limit), lo, hi);
		d.final = _actual;
		d.valid = true;
		return d;
	}
private:
	static float minimum(float a, float b) { return a < b ? a : b; }
	static float maximum(float a, float b) { return a > b ? a : b; }
	static float clamp(float x, float lo, float hi)
	{
		return maximum(lo, minimum(hi, x));
	}
	static float gate(float t, float on, float off)
	{
		const float a = std::fabs(t);
		return a <= on ? 1.f : (a >= off ? 0.f : (off - a) / (off - on));
	}
	Frame _frame{};
	float _live{0.f}, _actual{0.f};
	uint64_t _live_time{0};
	unsigned _ready_count{0};
};
