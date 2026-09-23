#pragma once

#include <cmath>
#include <cstdint>
#include "FastV2CandidateModels.hpp"

class FastV1ShadowModel
{
public:
	struct Output {
		float abs4{0.f};
		float delta4{0.f};
		bool valid{false};
		uint32_t frame_seq{0};
		uint32_t missed_frames{0};
		uint64_t frame_timestamp_us{0};
		uint32_t frame_dt_us{0};
		float t_lag2{0.f}; float t_lag4{0.f}; float t_lag8{0.f};
		float p_sp{0.f}; float p_error{0.f};
		float v2c_abs4{0.f}; float v2c_delta4{0.f};
		float control_model_t{0.f};
		bool control_delta_valid{false};
	};

	void reset() { resetFeatureHistory(); _next_frame = 0; _seq = 0; _missed = 0; _last_frame_timestamp = 0; _out = {}; }
	void resetFeatureHistory() { for (float &v : _ring) { v = 0.f; } _count = 0; _head = 0; _out.abs4 = 0.f; _out.delta4 = 0.f; _out.valid = false; _out.control_delta_valid = false; _out.t_lag2 = _out.t_lag4 = _out.t_lag8 = 0.f; _out.p_sp = _out.p_error = 0.f; }

	bool update(uint64_t now, float total_equivalent_i_raw, float p_sp, float p, Output &out)
	{
		if (!_schedule_initialized) { _next_frame = now; _schedule_initialized = true; }
		if (now < _next_frame) { out = _out; return false; }
		const uint64_t late = now - _next_frame;
		const uint32_t elapsed = static_cast<uint32_t>(late / 50000);
		_missed += elapsed;
		_next_frame += static_cast<uint64_t>(elapsed + 1) * 50000;
		++_seq;
		_out.control_delta_valid = false;
		out = _out;
		out.frame_seq = _seq;
		out.missed_frames = _missed;
		out.frame_timestamp_us = now;
		const uint32_t frame_dt = _last_frame_timestamp ? static_cast<uint32_t>(now - _last_frame_timestamp) : 0;
		out.frame_dt_us = frame_dt;
		_last_frame_timestamp = now;
		if (!std::isfinite(total_equivalent_i_raw) || !std::isfinite(p_sp) || !std::isfinite(p)) {
			resetFeatureHistory(); out = _out; out.frame_seq = _seq; out.missed_frames = _missed; out.frame_timestamp_us = now; out.frame_dt_us = frame_dt; _out = out; return true;
		}
		_ring[_head] = total_equivalent_i_raw; _head = (_head + 1) % 9; if (_count < 9) { ++_count; }
		if (_count < 9) {
			out.abs4 = 0.f; out.delta4 = 0.f; out.valid = false;
			out.t_lag2 = out.t_lag4 = out.t_lag8 = 0.f; out.p_sp = 0.f; out.p_error = 0.f;
			out.v2c_abs4 = out.v2c_delta4 = 0.f;
			_out = out; return true;
		}
		const float e = p_sp - p;
		const float t2 = at_lag(2), t4 = at_lag(4), t8 = at_lag(8);
		out.t_lag2 = t2; out.t_lag4 = t4; out.t_lag8 = t8; out.p_sp = p_sp; out.p_error = e;
		const float a[4] = {t2, t4, p_sp, e};
		const float d[4] = {t2 - t4, t4 - t8, p_sp, e};
		out.abs4 = predict(a, ABS_MEAN, ABS_SCALE, ABS_COEF, ABS_INTERCEPT);
		out.delta4 = predict(d, DELTA_MEAN, DELTA_SCALE, DELTA_COEF, DELTA_INTERCEPT);
		out.v2c_abs4 = fast_v2c::abs4::predict(a); out.v2c_delta4 = fast_v2c::delta4::predict(d);
		out.control_model_t = total_equivalent_i_raw;
		out.control_delta_valid = std::isfinite(t2) && std::isfinite(t4) && std::isfinite(t8)
			&& std::isfinite(e) && std::isfinite(d[0]) && std::isfinite(d[1]) && std::isfinite(out.v2c_delta4);
		out.valid = std::isfinite(out.abs4) && std::isfinite(out.delta4);
		_out = out;
		return true;
	}

private:
	inline static constexpr float ABS_MEAN[4] = {0.14029402297f, 0.14029048743f, 0.08042428347f, 0.09129474919f};
	inline static constexpr float ABS_SCALE[4] = {0.06555452939f, 0.06555911964f, 0.43864007199f, 0.63251224637f};
	inline static constexpr float ABS_COEF[4] = {0.03305716109f, -0.03463494343f, 0.00294182368f, 0.00235692766f};
	static constexpr float ABS_INTERCEPT = 0.00001644354452f;
	inline static constexpr float DELTA_MEAN[4] = {0.00000353553930f, 0.00000319667966f, 0.08042428347f, 0.09129474919f};
	inline static constexpr float DELTA_SCALE[4] = {0.00422011550f, 0.00612298096f, 0.43864007199f, 0.63251224637f};
	inline static constexpr float DELTA_COEF[4] = {0.00250028473f, 0.00185120207f, 0.00137961693f, 0.00228613405f};
	static constexpr float DELTA_INTERCEPT = 0.00001644354452f;
	float _ring[9]{}; uint8_t _head{0}; uint8_t _count{0}; uint64_t _next_frame{0}; uint64_t _last_frame_timestamp{0}; bool _schedule_initialized{false};
	uint32_t _seq{0}, _missed{0}; Output _out{};
	// _head points to the next write slot after the current frame was pushed.
	// Therefore lag N is N+1 slots behind _head.
	float at_lag(uint8_t lag) const { return _ring[(_head + 9 - (lag + 1)) % 9]; }
	static float predict(const float *x, const float *m, const float *s, const float *c, float b)
	{ float y = b; for (int i=0; i<4; ++i) { y += c[i] * ((x[i] - m[i]) / s[i]); } return y; }
};
