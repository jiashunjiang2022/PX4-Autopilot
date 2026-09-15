#pragma once

#include <cmath>
#include <cstddef>
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
		float headroom_release_ratio{0.f};
		bool adapt_enabled{false};
		float hold_cap_raw{0.f};
		float residual_reserve_raw{0.f};
		float adapt_tau_s{0.f};
		float adapt_slew_raw_per_s{0.f};
		float entry_window_s{0.f};
		float gate_window_s{0.f};
		float gate_std_raw{0.f};
		float gate_same_sign_fraction{0.f};
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
		float headroom_release_ratio_latched{0.f};
		float headroom_release_ratio_effective{0.f};
		float total_limit_raw{0.f};
		float residual_lower_raw{0.f};
		float residual_upper_raw{0.f};
		float headroom_positive_raw{0.f};
		float headroom_negative_raw{0.f};
		float exit_authority_decay_raw{0.f};
		bool normal_exit{false};
		bool safety_exit{false};
		bool recovery{false};
		bool reset_mismatch{false};
		bool reset_required{false};
		bool transfer_limited{false};
		bool normal_limited{false};
		bool legacy_b2a_active{false};
		bool adapt_enabled{false};
		bool entry_est_valid{false};
		bool adapt_gate{false};
		bool adapt_limited{false};
		bool adapt_releasing{false};
		bool adapt_reversal{false};
		float entry_est_raw{0.f};
		float adapt_t_hat_raw{0.f};
		float adapt_target_raw{0.f};
		float adapt_std_raw{0.f};
		float adapt_sign_fraction{0.f};
		float adapt_hold_cap_raw{0.f};
		float adapt_reserve_raw{0.f};
	};

	void synchronizeReset(uint32_t reset_epoch)
	{
		_transferred_i_raw = 0.f;
		_latched_target_raw = 0.f;
		_latched_headroom_release_ratio = 0.f;
		_adaptive_episode_enabled = false;
		_entry_estimate_raw = 0.f;
		_entry_estimate_valid = false;
		_adapt_t_hat_raw = 0.f;
		_adapt_t_hat_valid = false;
		clearEntryHistory();
		clearGateHistory();
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
		_last_imax_raw = in.imax_raw;

		if (rate_control.rollIntegralResetEpoch() != _expected_reset_epoch) {
			return enterRecovery(out, Reason::ResetMismatch, true);
		}

		const float effective_headroom_ratio = effectiveHeadroomReleaseRatio();
		const bool state_finite = PX4_ISFINITE(out.residual_i_before_raw) && PX4_ISFINITE(_transferred_i_raw)
					  && PX4_ISFINITE(in.imax_raw) && in.imax_raw >= 0.f;
		const float total_before = out.residual_i_before_raw + _transferred_i_raw;
		const RateControl::RollILimits current_limits = RateControl::computeRollILimits(in.imax_raw,
				_transferred_i_raw, effective_headroom_ratio);

		if (!state_finite || !PX4_ISFINITE(total_before) || !current_limits.valid) {
			return enterRecovery(out, Reason::Nonfinite, false);
		}

		if (fabsf(out.residual_i_before_raw) > in.imax_raw) {
			return enterRecovery(out, Reason::ResidualLimit, false);
		}

		if (fabsf(total_before) > current_limits.total_limit + StateEpsilon) {
			return enterRecovery(out, Reason::TotalEquivalentLimit, false);
		}

		if (in.hard_reset) {
			if (fabsf(_transferred_i_raw) > StateEpsilon) {
				return enterRecovery(out, Reason::ControlInvalid, false);
			}

			_state = State::Disabled;
			_adaptive_episode_enabled = false;
			_entry_estimate_valid = false;
			_entry_estimate_raw = 0.f;
			clearGateHistory();
			return finalize(out);
		}

		const bool parameter_valid = PX4_ISFINITE(in.dt) && in.dt > 0.f && PX4_ISFINITE(in.cap_raw)
					     && in.cap_raw >= 0.f && PX4_ISFINITE(in.slew_raw_per_s)
					     && in.slew_raw_per_s >= 0.f && PX4_ISFINITE(in.safety_slew_raw_per_s)
					     && in.safety_slew_raw_per_s >= 0.f && PX4_ISFINITE(in.g_current)
					     && in.g_current > 0.f;
		const bool adaptive_valid = adaptiveParamsValid(in);
		const bool headroom_parameter_valid = PX4_ISFINITE(in.headroom_release_ratio)
				&& in.headroom_release_ratio >= 0.f && in.headroom_release_ratio <= 1.f;
		const bool urgent_exit = in.pilot_abort || in.failsafe || !in.rates_enabled || !in.config_valid;

		if (urgent_exit) {
			if (fabsf(_transferred_i_raw) > 0.f) {
				_state = State::SafetyExit;
				out.reason = in.pilot_abort ? Reason::PilotAbort : (in.failsafe ? Reason::Failsafe : Reason::ControlInvalid);
				applyTransfer(out, rate_control, towardZeroStep(_transferred_i_raw,
						in.safety_slew_raw_per_s * positiveDt(in.dt)), true);
			} else {
				_state = State::Disabled;
			}

			return finalize(out);
		}

		if ((!parameter_valid || (_state == State::Disabled && !headroom_parameter_valid))
		    && (in.enabled || fabsf(_transferred_i_raw) > 0.f)) {
			return enterRecovery(out, Reason::Nonfinite, false);
		}

		if (_state == State::NormalTransferOut) {
			out.reason = Reason::NormalDisable;
			const float requested_delta_s = towardZeroStep(_transferred_i_raw, in.slew_raw_per_s * in.dt);
			applyTransfer(out, rate_control, requested_delta_s, false);

			if (fabsf(_transferred_i_raw) <= StateEpsilon) {
				_transferred_i_raw = 0.f;
				_state = State::Disabled;
				_adaptive_episode_enabled = false;
				_entry_estimate_valid = false;
				_entry_estimate_raw = 0.f;
				clearGateHistory();
			}

			return finalize(out);
		}

		if (_state == State::SafetyExit) {
			out.reason = _last_result.reason;
			applyTransfer(out, rate_control, towardZeroStep(_transferred_i_raw,
					in.safety_slew_raw_per_s * in.dt), true);

			if (fabsf(_transferred_i_raw) <= StateEpsilon) {
				_transferred_i_raw = 0.f;
				_state = State::Disabled;
				_adaptive_episode_enabled = false;
				_entry_estimate_valid = false;
				_entry_estimate_raw = 0.f;
				clearGateHistory();
			}

			return finalize(out);
		}

		const bool active_gate = in.enabled && in.eligible && in.cap_raw > 0.f;
		if (!in.eligible && _state == State::Disabled && in.adapt_enabled && parameter_valid && adaptive_valid) {
			observePreEntrySample(out.residual_i_before_raw, in.dt, in);
		}

		if (active_gate) {
			if (_state == State::Disabled) {
				_latched_headroom_release_ratio = math::constrain(in.headroom_release_ratio, 0.f, 1.f);
				_adaptive_episode_enabled = in.adapt_enabled && adaptive_valid;
				if (!_adaptive_episode_enabled) {
					_latched_target_raw = signedMinimum(out.residual_i_before_raw, in.cap_raw);
					_entry_estimate_valid = false;
					_entry_estimate_raw = 0.f;
				}
				if (_adaptive_episode_enabled) {
					_entry_estimate_raw = computeEntryMedian(in);
					_entry_estimate_valid = _entry_sample_count >= entrySampleCountRequired(in);
					if (!_entry_estimate_valid) {
						_entry_estimate_raw = signedMinimum(out.residual_i_before_raw, in.cap_raw);
					}
					_latched_target_raw = signedMinimum(_entry_estimate_raw, in.cap_raw);
					clearEntryHistory();
				}
				_state = State::Eligible;
				return finalize(out);
			}

			if (_state == State::Eligible) {
				_state = State::TransferIn;
				return finalize(out);
			}

			bool entered_hold = false;
			if (_state == State::TransferIn) {
				if (fabsf(out.residual_i_before_raw) <= StateEpsilon
				    || valuesOppose(out.residual_i_before_raw, _latched_target_raw)) {
					return cancelTransferEpisode(out);
				}

				const float requested_delta_s = towardTargetStep(_transferred_i_raw, _latched_target_raw,
								in.slew_raw_per_s * in.dt);
				applyTransfer(out, rate_control, requested_delta_s, false, false);

				if (fabsf(_latched_target_raw - _transferred_i_raw) <= StateEpsilon) {
					_state = State::TransferHold;
					entered_hold = true;
					initializeAdaptiveHold(out.residual_i_before_raw, _transferred_i_raw);
				}
			}

			if (_state == State::TransferHold && _adaptive_episode_enabled && !entered_hold) {
				updateAdaptiveHold(out, rate_control, in);
			}

			if (_state == State::TransferHold
			    && !_adaptive_episode_enabled
			    && valuesOppose(out.residual_i_before_raw, _transferred_i_raw)) {
				return cancelTransferEpisode(out);
			}

			return finalize(out);
		}

		if (fabsf(_transferred_i_raw) > 0.f) {
			if (in.enabled && !in.eligible) {
				_state = State::SafetyExit;
				out.reason = Reason::ControlInvalid;
				applyTransfer(out, rate_control, towardZeroStep(_transferred_i_raw,
						in.safety_slew_raw_per_s * in.dt), true);

			} else {
				_state = State::NormalTransferOut;
				out.reason = Reason::NormalDisable;
				const float requested_delta_s = towardZeroStep(_transferred_i_raw, in.slew_raw_per_s * in.dt);
				applyTransfer(out, rate_control, requested_delta_s, false);

				if (fabsf(_transferred_i_raw) <= StateEpsilon) {
					_transferred_i_raw = 0.f;
					_state = State::Disabled;
					_adaptive_episode_enabled = false;
					_entry_estimate_valid = false;
					_entry_estimate_raw = 0.f;
					clearGateHistory();
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

	static bool valuesOppose(float first, float second)
	{
		return (first > StateEpsilon && second < -StateEpsilon)
		       || (first < -StateEpsilon && second > StateEpsilon);
	}

	float effectiveHeadroomReleaseRatio() const
	{
		return _state == State::TransferHold || _state == State::NormalTransferOut || _state == State::SafetyExit
		       ? _latched_headroom_release_ratio : 0.f;
	}

	void clearEntryHistory()
	{
		_entry_sample_count = 0;
		_entry_sample_head = 0;
		_entry_sample_elapsed = 0.f;
	}

	void clearGateHistory()
	{
		_gate_sample_count = 0;
		_gate_sample_head = 0;
		_gate_sample_elapsed = 0.f;
		_gate_reference_raw = 0.f;
		_last_adapt_std_raw = 0.f;
		_last_adapt_sign_fraction = 0.f;
	}

	bool adaptiveParamsValid(const Inputs &in) const
	{
		return PX4_ISFINITE(in.hold_cap_raw) && in.hold_cap_raw >= 0.f
		       && PX4_ISFINITE(in.residual_reserve_raw) && in.residual_reserve_raw >= 0.f
		       && PX4_ISFINITE(in.adapt_tau_s) && in.adapt_tau_s > 0.f
		       && PX4_ISFINITE(in.adapt_slew_raw_per_s) && in.adapt_slew_raw_per_s >= 0.f
		       && PX4_ISFINITE(in.entry_window_s) && in.entry_window_s > 0.f
		       && PX4_ISFINITE(in.gate_window_s) && in.gate_window_s > 0.f
		       && PX4_ISFINITE(in.gate_std_raw) && in.gate_std_raw >= 0.f
		       && PX4_ISFINITE(in.gate_same_sign_fraction) && in.gate_same_sign_fraction >= 0.f
		       && in.gate_same_sign_fraction <= 1.f;
	}

	size_t entrySampleCountRequired(const Inputs &in) const
	{
		const float count = ceilf(in.entry_window_s * AdaptiveSampleRate);
		return static_cast<size_t>(math::constrain(count, 1.f, static_cast<float>(EntryHistoryCapacity)));
	}

	size_t gateSampleCountRequired(const Inputs &in) const
	{
		const float count = ceilf(in.gate_window_s * AdaptiveSampleRate);
		return static_cast<size_t>(math::constrain(ceilf(count), 1.f, static_cast<float>(GateHistoryCapacity)));
	}

	void observePreEntrySample(float sample, float dt, const Inputs &in)
	{
		if (!PX4_ISFINITE(sample) || !PX4_ISFINITE(dt) || dt <= 0.f) {
			return;
		}

		_entry_sample_elapsed += dt;
		if (_entry_sample_elapsed + SampleTimeEpsilon < AdaptiveSamplePeriod) {
			return;
		}

		_entry_sample_elapsed = fmaxf(0.f, _entry_sample_elapsed - AdaptiveSamplePeriod);
		if (_entry_sample_elapsed >= AdaptiveSamplePeriod) {
			_entry_sample_elapsed = fmodf(_entry_sample_elapsed, AdaptiveSamplePeriod);
		}
		_entry_samples[_entry_sample_head] = sample;
		_entry_sample_head = (_entry_sample_head + 1) % EntryHistoryCapacity;
		_entry_sample_count = math::min(_entry_sample_count + 1, EntryHistoryCapacity);
		(void)in;
	}

	float computeEntryMedian(const Inputs &in) const
	{
		if (_entry_sample_count == 0) {
			return 0.f;
		}

		const size_t sample_count = math::min(_entry_sample_count, entrySampleCountRequired(in));
		float sorted[EntryHistoryCapacity]{};
		for (size_t i = 0; i < sample_count; i++) {
			const size_t index = (_entry_sample_head + EntryHistoryCapacity - sample_count + i) % EntryHistoryCapacity;
			sorted[i] = _entry_samples[index];
		}
		for (size_t i = 1; i < sample_count; i++) {
			const float value = sorted[i];
			size_t j = i;
			while (j > 0 && sorted[j - 1] > value) {
				sorted[j] = sorted[j - 1];
				j--;
			}
			sorted[j] = value;
		}
		const size_t middle = sample_count / 2;
		const float median = (sample_count % 2 == 0)
				? 0.5f * (sorted[middle - 1] + sorted[middle]) : sorted[middle];
		return PX4_ISFINITE(median) ? median : 0.f;
	}

	void addGateSample(float sample)
	{
		if (!PX4_ISFINITE(sample)) {
			return;
		}
		if (_gate_sample_count < GateHistoryCapacity) {
			_gate_sample_count++;
		}
		_gate_samples[_gate_sample_head] = sample;
		_gate_sample_head = (_gate_sample_head + 1) % GateHistoryCapacity;
	}

	void observeGateSample(float sample, float dt)
	{
		if (!PX4_ISFINITE(sample) || !PX4_ISFINITE(dt) || dt <= 0.f) {
			return;
		}

		_gate_sample_elapsed += dt;
		if (_gate_sample_elapsed + SampleTimeEpsilon < AdaptiveSamplePeriod) {
			return;
		}

		_gate_sample_elapsed = fmaxf(0.f, _gate_sample_elapsed - AdaptiveSamplePeriod);
		if (_gate_sample_elapsed >= AdaptiveSamplePeriod) {
			_gate_sample_elapsed = fmodf(_gate_sample_elapsed, AdaptiveSamplePeriod);
		}
		addGateSample(sample);
	}

	float computeGateStd(size_t sample_count) const
	{
		if (sample_count == 0) {
			return 0.f;
		}

		float sum = 0.f;
		float sum_sq = 0.f;
		for (size_t i = 0; i < sample_count; i++) {
			const size_t index = (_gate_sample_head + GateHistoryCapacity - sample_count + i) % GateHistoryCapacity;
			const float sample = _gate_samples[index];
			sum += sample;
			sum_sq += sample * sample;
		}
		const float n = static_cast<float>(sample_count);
		const float mean = sum / n;
		return sqrtf(fmaxf(0.f, sum_sq / n - mean * mean));
	}

	float computeGateSignFraction(size_t sample_count, float reference) const
	{
		if (sample_count == 0 || !PX4_ISFINITE(reference) || fabsf(reference) <= StateEpsilon) {
			return 0.f;
		}

		size_t same_sign_count = 0;
		for (size_t i = 0; i < sample_count; i++) {
			const size_t index = (_gate_sample_head + GateHistoryCapacity - sample_count + i) % GateHistoryCapacity;
			if (_gate_samples[index] * reference > 0.f) {
				same_sign_count++;
			}
		}
		return static_cast<float>(same_sign_count) / static_cast<float>(sample_count);
	}

	void initializeAdaptiveHold(float residual_raw, float transferred_raw)
	{
		_adapt_t_hat_raw = _entry_estimate_valid ? _entry_estimate_raw : residual_raw + transferred_raw;
		_adapt_t_hat_valid = PX4_ISFINITE(_adapt_t_hat_raw);
		clearGateHistory();
		_gate_reference_raw = _adapt_t_hat_raw;
		_last_adapt_target_raw = transferred_raw;
	}

	float adaptiveTarget(const Inputs &in) const
	{
		if (!_adapt_t_hat_valid || !PX4_ISFINITE(_adapt_t_hat_raw)) {
			return 0.f;
		}
		const float magnitude = math::constrain(fmaxf(fabsf(_adapt_t_hat_raw) - in.residual_reserve_raw,
									 0.f), 0.f, in.hold_cap_raw);
		return copysignf(magnitude, _adapt_t_hat_raw);
	}

	void updateAdaptiveHold(Result &out, RateControl &rate_control, const Inputs &in)
	{
		out.adapt_enabled = true;
		if (!adaptiveParamsValid(in) || !PX4_ISFINITE(out.total_equivalent_i_raw)) {
			out.adapt_limited = true;
			return;
		}
		const float dt = positiveDt(in.dt);
		const float total = out.residual_i_before_raw + _transferred_i_raw;
		if (!_adapt_t_hat_valid) {
			initializeAdaptiveHold(out.residual_i_before_raw, _transferred_i_raw);
		}
		const float alpha = dt / (in.adapt_tau_s + dt);
		_adapt_t_hat_raw += alpha * (total - _adapt_t_hat_raw);
		_adapt_t_hat_valid = PX4_ISFINITE(_adapt_t_hat_raw);
		if (valuesOppose(_adapt_t_hat_raw, _gate_reference_raw)) {
			clearGateHistory();
		}
		if (fabsf(_adapt_t_hat_raw) > StateEpsilon) {
			_gate_reference_raw = _adapt_t_hat_raw;
		}
		observeGateSample(total, dt);
		const size_t required_samples = gateSampleCountRequired(in);
		const size_t window_samples = math::min(_gate_sample_count, required_samples);
		_last_adapt_std_raw = computeGateStd(window_samples);
		_last_adapt_sign_fraction = computeGateSignFraction(window_samples, _adapt_t_hat_raw);
		_last_adapt_hold_cap_raw = in.hold_cap_raw;
		_last_adapt_reserve_raw = in.residual_reserve_raw;
		const float target = adaptiveTarget(in);
		_last_adapt_target_raw = target;
		out.adapt_gate = _gate_sample_count >= required_samples
				&& _last_adapt_std_raw <= in.gate_std_raw
				&& _last_adapt_sign_fraction >= in.gate_same_sign_fraction;

		const bool reversal = valuesOppose(_transferred_i_raw, target);
		out.adapt_reversal = reversal;
		const bool releasing = fabsf(target) < fabsf(_transferred_i_raw) || reversal;
		out.adapt_releasing = releasing;
		if (reversal && fabsf(_transferred_i_raw) > StateEpsilon) {
			const float requested = towardZeroStep(_transferred_i_raw, in.adapt_slew_raw_per_s * dt);
			applyTransfer(out, rate_control, requested, false, true);
			return;
		}
		if (!releasing && !_adapt_t_hat_valid) {
			return;
		}
		if (!releasing && !out.adapt_gate) {
			return;
		}
		const float requested = towardTargetStep(_transferred_i_raw, target, in.adapt_slew_raw_per_s * dt);
		if (fabsf(requested) > StateEpsilon) {
			applyTransfer(out, rate_control, requested, false, true);
		}
	}

	Result cancelTransferEpisode(Result &out)
	{
		_latched_target_raw = 0.f;

		if (fabsf(_transferred_i_raw) > StateEpsilon) {
			_state = State::NormalTransferOut;
			out.reason = Reason::NormalDisable;

		} else {
			_transferred_i_raw = 0.f;
			_adaptive_episode_enabled = false;
			_entry_estimate_valid = false;
			_entry_estimate_raw = 0.f;
			clearGateHistory();
			_state = State::Disabled;
		}

		return finalize(out);
	}

	void applyTransfer(Result &out, RateControl &rate_control, float requested_delta_s, bool safety,
			   bool adaptive_hold = false)
	{
		const float requested_delta_i = -requested_delta_s;
		out.requested_delta_i_raw = requested_delta_i;
		out.requested_delta_s_raw = requested_delta_s;
		const RateControl::RollITransferMode mode = _state == State::TransferIn
				? RateControl::RollITransferMode::TowardZero : RateControl::RollITransferMode::PairPreserving;
		const float headroom_release_ratio = effectiveHeadroomReleaseRatio();
		const float transferred_after_requested = _transferred_i_raw + requested_delta_s;
		const auto accepted = rate_control.applyRollITransfer({requested_delta_i, _transferred_i_raw, mode,
				headroom_release_ratio, transferred_after_requested, true});
		out.accepted_delta_i_raw = accepted.accepted_delta_raw;
		out.transfer_limited = accepted.limited_at_zero || accepted.limited_by_imax || !accepted.valid;

		if (!accepted.valid && !safety) {
			_state = State::RecoveryReconciliation;
			out.reason = Reason::ControlInvalid;
			out.recovery = true;
			out.reset_required = true;
			return;
		}

		out.accepted_delta_s_raw = (_state == State::TransferIn || adaptive_hold)
				? -accepted.accepted_delta_raw : out.requested_delta_s_raw;
		_transferred_i_raw += out.accepted_delta_s_raw;
		out.unmatched_delta_raw = accepted.accepted_delta_raw + out.accepted_delta_s_raw;
		if (adaptive_hold) {
			out.adapt_limited = fabsf(out.accepted_delta_s_raw - out.requested_delta_s_raw) > StateEpsilon;
		}

		if (_state == State::NormalTransferOut) {
			out.exit_authority_decay_raw = out.unmatched_delta_raw;
			out.normal_limited = fabsf(out.unmatched_delta_raw) > StateEpsilon;
		}
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
		out.headroom_release_ratio_latched = _latched_headroom_release_ratio;
		out.headroom_release_ratio_effective = effectiveHeadroomReleaseRatio();
		const RateControl::RollILimits limits = RateControl::computeRollILimits(
				_last_imax_raw, out.transferred_i_raw, out.headroom_release_ratio_effective);

		if (limits.valid) {
			out.total_limit_raw = limits.total_limit;
			out.residual_lower_raw = limits.lower;
			out.residual_upper_raw = limits.upper;
			const RateControl::RollILimits legacy_limits = RateControl::computeRollILimits(
					_last_imax_raw, out.transferred_i_raw, 0.f);
			out.headroom_positive_raw = limits.upper - legacy_limits.upper;
			out.headroom_negative_raw = legacy_limits.lower - limits.lower;
		}
		out.effective_slow_torque = out.g_current * out.transferred_i_raw;
		out.effective_slow_tail = out.effective_slow_torque / TorquePerTail;
		out.normal_exit = _state == State::NormalTransferOut;
		out.safety_exit = _state == State::SafetyExit;
		out.recovery = out.recovery || _state == State::RecoveryReconciliation;
		out.adapt_enabled = _adaptive_episode_enabled;
		out.entry_est_valid = _entry_estimate_valid;
		out.entry_est_raw = PX4_ISFINITE(_entry_estimate_raw) ? _entry_estimate_raw : 0.f;
		out.adapt_t_hat_raw = PX4_ISFINITE(_adapt_t_hat_raw) ? _adapt_t_hat_raw : 0.f;
		out.adapt_target_raw = PX4_ISFINITE(_last_adapt_target_raw) ? _last_adapt_target_raw : 0.f;
		out.adapt_std_raw = PX4_ISFINITE(_last_adapt_std_raw) ? _last_adapt_std_raw : 0.f;
		out.adapt_sign_fraction = PX4_ISFINITE(_last_adapt_sign_fraction) ? _last_adapt_sign_fraction : 0.f;
		out.adapt_hold_cap_raw = _last_adapt_hold_cap_raw;
		out.adapt_reserve_raw = _last_adapt_reserve_raw;
		_last_result = out;
		return out;
	}

	State _state{State::Disabled};
	static constexpr float AdaptiveSamplePeriod = 0.05f;
	static constexpr float AdaptiveSampleRate = 20.f;
	static constexpr float SampleTimeEpsilon = 1e-6f;
	static constexpr size_t EntryHistoryCapacity = 200;
	static constexpr size_t GateHistoryCapacity = 200;
	float _entry_samples[EntryHistoryCapacity]{};
	size_t _entry_sample_count{0};
	size_t _entry_sample_head{0};
	float _entry_sample_elapsed{0.f};
	float _gate_samples[GateHistoryCapacity]{};
	size_t _gate_sample_count{0};
	size_t _gate_sample_head{0};
	float _gate_sample_elapsed{0.f};
	float _gate_reference_raw{0.f};
	float _entry_estimate_raw{0.f};
	bool _entry_estimate_valid{false};
	bool _adaptive_episode_enabled{false};
	float _adapt_t_hat_raw{0.f};
	bool _adapt_t_hat_valid{false};
	float _last_adapt_std_raw{0.f};
	float _last_adapt_sign_fraction{0.f};
	float _last_adapt_target_raw{0.f};
	float _last_adapt_hold_cap_raw{0.f};
	float _last_adapt_reserve_raw{0.f};
	float _transferred_i_raw{0.f};
	float _latched_target_raw{0.f};
	float _latched_headroom_release_ratio{0.f};
	float _last_imax_raw{0.f};
	uint32_t _expected_reset_epoch{0};
	Result _last_result{};
};
