#pragma once

#include <cmath>
#include <cstdint>

#include <lib/rate_control/rate_control.hpp>

struct BumplessRollITransferTestAccess;

class BumplessRollITransfer final
{
public:
	enum class State : uint8_t {
		Disabled = 0,
		Eligible = 1,
		TransferIn = 2,
		TransferHold = 3,
		NormalTransferOut = 4,
		SafetyExit = 5,
		RecoveryReconciliation = 6
	};

	enum class Reason : uint8_t {
		None = 0,
		NormalDisable = 1,
		PilotAbort = 2,
		Failsafe = 3,
		ControlInvalid = 4,
		Nonfinite = 5,
		ResidualLimit = 6,
		TotalEquivalentLimit = 7,
		ResetMismatch = 8
	};

	struct Inputs {
		bool enabled{false};
		bool eligible{false};
		bool pilot_abort{false};
		bool failsafe{false};
		bool hard_reset{false};
		bool rates_enabled{false};
		bool config_valid{false};
		float dt{0.f};
		float imax_raw{0.f};
		float cap_raw{0.f};
		float slew_raw_per_s{0.f};
		float safety_slew_raw_per_s{0.f};
		float g_current{0.f};
	};

	struct Result {
		State state{State::Disabled};
		Reason reason{Reason::None};
		float residual_i_before_raw{0.f};
		float requested_delta_i_raw{0.f};
		float accepted_delta_i_raw{0.f};
		float residual_i_raw{0.f};
		float transferred_i_before_raw{0.f};
		float requested_delta_s_raw{0.f};
		float accepted_delta_s_raw{0.f};
		float transferred_i_raw{0.f};
		float total_equivalent_i_raw{0.f};
		float unmatched_delta_raw{0.f};
		float g_current{0.f};
		float effective_slow_torque{0.f};
		float effective_slow_tail{0.f};
		float fast_actual_roll{0.f};
		float fast_actual_pitch{0.f};
		float delta_b{0.f};
		bool normal_exit{false};
		bool safety_exit{false};
		bool recovery{false};
		bool reset_mismatch{false};
		bool reset_required{false};
		bool transfer_limited{false};
		bool normal_limited{false};
		bool legacy_b2a_active{false};
	};

	void synchronizeReset(uint32_t reset_epoch)
	{
		_transferred_i_raw = 0.f;
		_latched_target_raw = 0.f;
		_expected_reset_epoch = reset_epoch;
		_state = State::Disabled;
		_last_result = {};
	}

	Result update(const Inputs &in, RateControl &rate_control)
	{
		Result out{};
		out.residual_i_before_raw = rate_control.rollIntegralRaw();
		out.residual_i_raw = out.residual_i_before_raw;
		out.transferred_i_before_raw = _transferred_i_raw;
		out.transferred_i_raw = _transferred_i_raw;
		out.g_current = PX4_ISFINITE(in.g_current) ? in.g_current : 0.f;

		if (rate_control.rollIntegralResetEpoch() != _expected_reset_epoch) {
			return enterRecovery(out, Reason::ResetMismatch, true);
		}

		const bool state_finite = PX4_ISFINITE(out.residual_i_before_raw) && PX4_ISFINITE(_transferred_i_raw)
					  && PX4_ISFINITE(in.imax_raw) && in.imax_raw >= 0.f;
		const float total_before = out.residual_i_before_raw + _transferred_i_raw;

		if (!state_finite || !PX4_ISFINITE(total_before)) {
			return enterRecovery(out, Reason::Nonfinite, false);
		}

		if (fabsf(out.residual_i_before_raw) > in.imax_raw) {
			return enterRecovery(out, Reason::ResidualLimit, false);
		}

		if (fabsf(total_before) > in.imax_raw) {
			return enterRecovery(out, Reason::TotalEquivalentLimit, false);
		}

		if (in.hard_reset) {
			if (fabsf(_transferred_i_raw) > StateEpsilon) {
				return enterRecovery(out, Reason::ControlInvalid, false);
			}

			_state = State::Disabled;
			return finalize(out);
		}

		const bool parameter_valid = PX4_ISFINITE(in.dt) && in.dt > 0.f && PX4_ISFINITE(in.cap_raw)
					     && in.cap_raw >= 0.f && PX4_ISFINITE(in.slew_raw_per_s)
					     && in.slew_raw_per_s >= 0.f && PX4_ISFINITE(in.safety_slew_raw_per_s)
					     && in.safety_slew_raw_per_s >= 0.f && PX4_ISFINITE(in.g_current)
					     && in.g_current > 0.f;
		const bool urgent_exit = in.pilot_abort || in.failsafe || !in.rates_enabled || !in.config_valid;

		if (urgent_exit) {
			if (fabsf(_transferred_i_raw) > 0.f) {
				_state = State::SafetyExit;
				out.reason = in.pilot_abort ? Reason::PilotAbort : (in.failsafe ? Reason::Failsafe : Reason::ControlInvalid);
				applyTransfer(out, rate_control, -towardZeroStep(_transferred_i_raw,
						in.safety_slew_raw_per_s * positiveDt(in.dt)), true);
			} else {
				_state = State::Disabled;
			}

			return finalize(out);
		}

		if (!parameter_valid && (in.enabled || fabsf(_transferred_i_raw) > 0.f)) {
			return enterRecovery(out, Reason::Nonfinite, false);
		}

		if (_state == State::NormalTransferOut) {
			out.reason = Reason::NormalDisable;
			const float requested_delta_s = towardZeroStep(_transferred_i_raw, in.slew_raw_per_s * in.dt);
			applyTransfer(out, rate_control, -requested_delta_s, false);

			if (fabsf(_transferred_i_raw) <= StateEpsilon) {
				_transferred_i_raw = 0.f;
				_state = State::Disabled;
			}

			return finalize(out);
		}

		if (_state == State::SafetyExit) {
			out.reason = _last_result.reason;
			applyTransfer(out, rate_control, -towardZeroStep(_transferred_i_raw,
					in.safety_slew_raw_per_s * in.dt), true);

			if (fabsf(_transferred_i_raw) <= StateEpsilon) {
				_transferred_i_raw = 0.f;
				_state = State::Disabled;
			}

			return finalize(out);
		}

		const bool active_gate = in.enabled && in.eligible && in.cap_raw > 0.f;

		if (active_gate) {
			if (_state == State::Disabled) {
				_latched_target_raw = signedMinimum(out.residual_i_before_raw, in.cap_raw);
				_state = State::Eligible;
				return finalize(out);
			}

			if (_state == State::Eligible) {
				_state = State::TransferIn;
				return finalize(out);
			}

			if (_state == State::TransferIn) {
				const float requested_delta_s = towardTargetStep(_transferred_i_raw, _latched_target_raw,
								in.slew_raw_per_s * in.dt);
				applyTransfer(out, rate_control, -requested_delta_s, false);

				if (fabsf(_latched_target_raw - _transferred_i_raw) <= StateEpsilon) {
					_state = State::TransferHold;
				}
			}

			return finalize(out);
		}

		if (fabsf(_transferred_i_raw) > 0.f) {
			if (in.enabled && !in.eligible) {
				_state = State::SafetyExit;
				out.reason = Reason::ControlInvalid;
				applyTransfer(out, rate_control, -towardZeroStep(_transferred_i_raw,
						in.safety_slew_raw_per_s * in.dt), true);

			} else {
				_state = State::NormalTransferOut;
				out.reason = Reason::NormalDisable;
				const float requested_delta_s = towardZeroStep(_transferred_i_raw, in.slew_raw_per_s * in.dt);
				applyTransfer(out, rate_control, -requested_delta_s, false);

				if (fabsf(_transferred_i_raw) <= StateEpsilon) {
					_transferred_i_raw = 0.f;
				}
			}

		} else {
			_state = State::Disabled;
		}

		return finalize(out);
	}

	State state() const { return _state; }
	float transferredRaw() const { return _transferred_i_raw; }
	float latchedTargetRaw() const { return _latched_target_raw; }
	const Result &lastResult() const { return _last_result; }

	static float composeRawRoll(float baseline_raw, float transferred_i_raw)
	{
		return baseline_raw + transferred_i_raw;
	}

private:
	friend struct BumplessRollITransferTestAccess;
	static constexpr float StateEpsilon = 1e-7f;
	static constexpr float TorquePerTail = 1.1f;

	static float positiveDt(float dt) { return PX4_ISFINITE(dt) && dt > 0.f ? dt : 0.f; }

	static float signedMinimum(float value, float magnitude)
	{
		const float limited = fminf(fabsf(value), magnitude);
		return copysignf(limited, value);
	}

	static float towardTargetStep(float value, float target, float max_step)
	{
		const float error = target - value;
		return fmaxf(-max_step, fminf(error, max_step));
	}

	static float towardZeroStep(float value, float max_step)
	{
		return towardTargetStep(value, 0.f, max_step);
	}

	void applyTransfer(Result &out, RateControl &rate_control, float requested_delta_i, bool safety)
	{
		out.requested_delta_i_raw = requested_delta_i;
		out.requested_delta_s_raw = -requested_delta_i;
		const RateControl::RollITransferMode mode = _state == State::TransferIn
				? RateControl::RollITransferMode::TowardZero : RateControl::RollITransferMode::PairPreserving;
		const auto accepted = rate_control.applyRollITransfer({requested_delta_i, _transferred_i_raw, mode});
		out.accepted_delta_i_raw = accepted.accepted_delta_raw;
		out.transfer_limited = accepted.limited_at_zero || accepted.limited_by_imax || !accepted.valid;

		if (!accepted.valid && !safety) {
			_state = State::RecoveryReconciliation;
			out.reason = Reason::ControlInvalid;
			out.recovery = true;
			out.reset_required = true;
			return;
		}

		out.accepted_delta_s_raw = safety ? out.requested_delta_s_raw : -accepted.accepted_delta_raw;
		_transferred_i_raw += out.accepted_delta_s_raw;
		out.unmatched_delta_raw = accepted.accepted_delta_raw + out.accepted_delta_s_raw;
	}

	Result enterRecovery(Result &out, Reason reason, bool reset_mismatch)
	{
		_state = State::RecoveryReconciliation;
		out.reason = reason;
		out.recovery = true;
		out.reset_mismatch = reset_mismatch;
		out.reset_required = true;
		return finalize(out);
	}

	Result finalize(Result &out)
	{
		out.state = _state;
		out.residual_i_raw = out.residual_i_before_raw + out.accepted_delta_i_raw;
		out.transferred_i_raw = PX4_ISFINITE(_transferred_i_raw) ? _transferred_i_raw : 0.f;
		out.total_equivalent_i_raw = out.residual_i_raw + out.transferred_i_raw;
		out.effective_slow_torque = out.g_current * out.transferred_i_raw;
		out.effective_slow_tail = out.effective_slow_torque / TorquePerTail;
		out.normal_exit = _state == State::NormalTransferOut;
		out.safety_exit = _state == State::SafetyExit;
		out.recovery = out.recovery || _state == State::RecoveryReconciliation;
		_last_result = out;
		return out;
	}

	State _state{State::Disabled};
	float _transferred_i_raw{0.f};
	float _latched_target_raw{0.f};
	uint32_t _expected_reset_epoch{0};
	Result _last_result{};
};
