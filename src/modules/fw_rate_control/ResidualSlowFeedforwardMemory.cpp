// SPDX-License-Identifier: BSD-3-Clause
#include "ResidualSlowFeedforwardMemory.hpp"
#include <cmath>
#include <cassert>

ResidualSlowFeedforwardMemory::ResidualSlowFeedforwardMemory(RateControl &native) :
	_native(native), _epoch(native.rollIntegralResetEpoch())
{
}

bool ResidualSlowFeedforwardMemory::configValid(const Config &c) const
{
	return PX4_ISFINITE(c.imax) && c.imax >= 0.f && c.imax <= 1.f
	       && PX4_ISFINITE(c.bmax) && c.bmax >= 0.f && c.bmax <= c.imax
	       && PX4_ISFINITE(c.reserve) && c.reserve >= 0.f
	       && PX4_ISFINITE(c.tau) && c.tau > 0.f
	       && PX4_ISFINITE(c.gain) && c.gain >= 0.f
	       && PX4_ISFINITE(c.slew) && c.slew > 0.f
	       && PX4_ISFINITE(c.window) && c.window >= .05f && c.window <= 6.f
	       && PX4_ISFINITE(c.gstd) && c.gstd >= 0.f
	       && PX4_ISFINITE(c.gsign) && c.gsign >= 0.f && c.gsign <= 1.f;
}

bool ResidualSlowFeedforwardMemory::configure(const Config &c)
{
	if (!configValid(c)) {
		_config_valid = false;
		recover(Reason::InvalidConfig);
		return false; // Retain previous valid limits, never install NaN/negative limits.
	}
	const bool changed = fabsf(c.imax - _config.imax) > 0.f || fabsf(c.bmax - _config.bmax) > 0.f
	                     || fabsf(c.reserve - _config.reserve) > 0.f || fabsf(c.tau - _config.tau) > 0.f
	                     || fabsf(c.gain - _config.gain) > 0.f || fabsf(c.slew - _config.slew) > 0.f
	                     || fabsf(c.window - _config.window) > 0.f || fabsf(c.gstd - _config.gstd) > 0.f
	                     || fabsf(c.gsign - _config.gsign) > 0.f;
	_config = c;
	_config_valid = true;
	if (changed) {
		invalidateEstimate();
	}
// Check the new contract BEFORE installing its native limit.
	if (!checkPair()) {
		return true;
	}
	_native.setRollIntegralLimit(c.imax);
	setContext();
	return true;
}

void ResidualSlowFeedforwardMemory::setContext()
{
	_native.setRollITransferContext(_b, 0.f, true); // V4 has no HR configuration.
}

void ResidualSlowFeedforwardMemory::invalidateHistory()
{
	_count = 0;
	_head = 0;
	_result.gate_valid = false;
	_result.learn = false;
	_result.stddev = 0.f;
	_result.positive_fraction = 0.f;
	_result.same_sign_fraction = 0.f;
	_result.gate_time = 0;
// Keep last consumed timestamp: clearing eligibility cannot replay a sample.
}

void ResidualSlowFeedforwardMemory::invalidateEstimate()
{
	invalidateHistory();
	_estimate_valid = false;
	_q_hat = 0.f;
	refreshResidual();
}

void ResidualSlowFeedforwardMemory::recover(Reason reason)
{
	_b = 0.f;
	_native.resetIntegral(0);
	_epoch = _native.rollIntegralResetEpoch();
	_handback = false;
	invalidateEstimate();
	_native.setRollIntegralLimit(_config.imax);
	setContext();
	_result.recovery = true;
	_result.apply = false;
	_result.reason = reason;
	_result.requested = 0.f;
	_result.accepted = 0.f;
}

bool ResidualSlowFeedforwardMemory::checkPair()
{
	if (_native.rollIntegralResetEpoch() != _epoch) {
		recover(Reason::EpochMismatch);
		return false;
	}
	const float i = _native.rollIntegralRaw();
	if (!PX4_ISFINITE(_b) || !PX4_ISFINITE(i)) {
		recover(Reason::NonfinitePair);
		return false;
	}
	const auto bounds = RateControl::computeRollILimits(_config.imax, _b, 0.f);
	if (!bounds.valid || fabsf(_b) > _config.bmax
	    || i < bounds.lower - Tolerance || i > bounds.upper + Tolerance) {
		recover(Reason::IllegalPair);
		return false;
	}
	return true;
}

void ResidualSlowFeedforwardMemory::refreshResidual()
{
	_result.residual = _estimate_valid ? _q_hat - _b : 0.f;
	_result.innovation = _estimate_valid
	                     ? copysignf(fmaxf(fabsf(_result.residual) - _config.reserve, 0.f), _result.residual) : 0.f;
}

bool ResidualSlowFeedforwardMemory::transfer(float requested)
{
	_result.requested = requested;
	_result.accepted = 0.f;
	if (!checkPair()) {
		return false;
	}
	if (!PX4_ISFINITE(requested)) {
		return false;
	}
	const float before = _b;
	const float i = _native.rollIntegralRaw();
	const float lower = fmaxf(-_config.bmax - before, i - _config.imax);
	const float upper = fminf(_config.bmax - before, i + _config.imax);
	float d = math::constrain(requested, lower, upper);
// Numerical boundary tolerances must never create a reversed request.
	if ((requested >= 0.f && d < 0.f) || (requested <= 0.f && d > 0.f)) {
		d = 0.f;
	}
	if ((before > 0.f && d < -before) || (before < 0.f && d > -before)) {
		d = -before;
	}
	const float b_post = before + d;
	if (fabsf(b_post - before) <= 0.f) {
		return true; // A capped/rounded-to-zero request must not rewrite native I.
	}
// Preserve the representable total rather than accumulating subtraction drift.
	const float total = i + before;
	const float i_post = total - b_post;
	const auto bounds = RateControl::computeRollILimits(_config.imax, b_post, 0.f);
	if (!PX4_ISFINITE(b_post) || !PX4_ISFINITE(i_post) || !bounds.valid
	    || fabsf(b_post) > _config.bmax || fabsf(i_post) > _config.imax + Tolerance
	    || i_post < bounds.lower - Tolerance || i_post > bounds.upper + Tolerance
	    || fabsf((i_post - i) + (b_post - before)) > Tolerance
	    || fabsf((i_post + b_post) - total) > Tolerance) {
		recover(Reason::TransactionRejected);
		return false;
	}
// Single joint commit checks snapshots again. No V3 mutating transfer call.
	if (!_native.commitRollMemoryPair(_b, before, i, _epoch, _config.imax,
	                                  b_post, i_post, _config.bmax)) {
		recover(Reason::TransactionRejected);
		return false;
	}
	_result.accepted = _b - before;
	refreshResidual();
	statistics(); // Reinterpret existing q samples; no time/count changes.
	_result.gate_valid = _result.gate_valid && _result.stddev <= _config.gstd
	                     && _result.same_sign_fraction >= _config.gsign;
	if (fabsf(before) > 0.f && fabsf(_b) <= 0.f) {
		invalidateHistory(); // Zero reached: next sample begins an entirely new window.
	}
	assert(PX4_ISFINITE(_b) && PX4_ISFINITE(_native.rollIntegralRaw()));
	assert(fabsf(_b) <= _config.bmax + Tolerance);
	assert(fabsf(_native.rollIntegralRaw()) <= _config.imax + Tolerance);
	assert(fabsf(_b + _native.rollIntegralRaw()) <= _config.imax + Tolerance);
	assert(fabsf((_b + _native.rollIntegralRaw()) - total) <= Tolerance);
	return true;
}

void ResidualSlowFeedforwardMemory::handback(float dt)
{
	invalidateHistory();
	_result.learn = false;
	if (!PX4_ISFINITE(dt) || dt <= 0.f) {
		return;
	}
// Scheduler stalls must not turn an exit into an unbounded instantaneous step.
	const float step = _config.slew * fminf(dt, .15f);
	transfer(math::constrain(-_b, -step, step));
	if (fabsf(_b) <= 0.f) {
		_handback = false;
		invalidateEstimate();
	}
}

void ResidualSlowFeedforwardMemory::append(float q, uint64_t timestamp)
{
	const uint64_t window = static_cast<uint64_t>(_config.window * 1000000.f);
// Retain the boundary sample bracketing the full real-time interval.
	while (_count > 1) {
		const size_t second = (_head + 1) % Capacity;
		if (timestamp - _history[second].time < window) {
			break;
		}
		_head = second;
		--_count;
	}
	if (_count == Capacity) {
		_head = (_head + 1) % Capacity;
		--_count;
	}
	_history[(_head + _count) % Capacity] = {q, timestamp};
	++_count;
}

void ResidualSlowFeedforwardMemory::statistics()
{
	if (_count == 0) {
		_result.stddev = _result.positive_fraction = _result.same_sign_fraction = 0.f;
		return;
	}
	double mean = 0.;
	for (size_t j = 0; j < _count; ++j) {
		mean += static_cast<double>(_history[(_head + j) % Capacity].q);
	}
	mean /= _count;
	double variance = 0.;
	size_t positive = 0, negative = 0;
	for (size_t j = 0; j < _count; ++j) {
		const float q = _history[(_head + j) % Capacity].q;
		const double delta = static_cast<double>(q) - mean;
		variance += delta * delta;
		positive += q - _b > 0.f;
		negative += q - _b < 0.f;
	}
	_result.stddev = sqrtf(static_cast<float>(variance / _count));
	_result.positive_fraction = static_cast<float>(positive) / _count;
	_result.same_sign_fraction = static_cast<float>(_result.residual > 0.f ? positive : negative) / _count;
}

ResidualSlowFeedforwardMemory::Result ResidualSlowFeedforwardMemory::update(const Inputs &in)
{
	_result.recovery = false;
	_result.reason = Reason::None;
	_result.learn = false;
	_result.requested = _result.accepted = 0.f;
	if (!_config_valid) {
		recover(Reason::InvalidConfig);
		return result();
	}
	if (!checkPair()) {
		return result();
	}
	if (fabsf(_native.rollIntegralLimit() - _config.imax) > 0.f || !PX4_ISFINITE(_native.rollIntegralLimit())) {
		recover(Reason::IllegalPair);
		return result();
	}
	if (in.hard_reset || !in.control_valid) {
		recover(in.hard_reset ? Reason::HardReset : Reason::ControlInvalid);
		return result();
	}
	setContext();
	_result.apply = true;
	if (_estimate_valid && !PX4_ISFINITE(_q_hat)) {
		invalidateEstimate();
		_result.reason = Reason::EstimatorInvalid;
		return result();
	}
	if (!in.enabled || in.pilot_abort || in.failsafe || !in.mission) {
		_handback = fabsf(_b) > 0.f;
		_result.reason = in.pilot_abort ? Reason::PilotAbort :
		                 (in.failsafe ? Reason::Failsafe : Reason::Disabled);
	}
	if (_handback) {
		handback(in.control_dt);
		return result();
	}
	if (!in.enabled || !in.mission || in.pilot_abort || in.failsafe) {
		invalidateHistory();
		return result();
	}
	if (!in.allocator_fresh) {
		invalidateHistory();
		_result.reason = Reason::AllocatorUnknown;
	}
	if (in.maneuver || !in.stable || !in.evidence_valid) {
		invalidateHistory();
	}
// Timeout checked on ticks too, before fresh-sample acceptance.
	if (_have_evidence && in.now > _last_evidence && in.now - _last_evidence > MaxGap) {
		invalidateEstimate();
		_result.reason = Reason::EvidenceGap;
	}
	if (!in.fresh_sample || !in.evidence_valid || in.timestamp > in.now
	    || (_have_evidence && in.timestamp <= _last_evidence)
	    || in.now - in.timestamp > MaxGap) {
		return result();
	}
	const uint64_t elapsed = _have_evidence ? in.timestamp - _last_evidence : 0;
	if (_have_evidence && elapsed > MaxGap) {
		invalidateEstimate();
	}
	const float dt = elapsed > 0 && elapsed <= MaxGap ? elapsed * 1e-6f : .05f;
	_last_evidence = in.timestamp;
	_have_evidence = true;
	const float q = _native.rollIntegralRaw() + _b;
	if (!_estimate_valid) {
		_q_hat = q;
		_estimate_valid = true;
	} else {
		_q_hat += dt / (_config.tau + dt) * (q - _q_hat);
	}
	if (!PX4_ISFINITE(_q_hat)) {
		invalidateEstimate();
		return result();
	}
	refreshResidual();
	if (!in.allocator_fresh || in.maneuver || !in.stable) {
		return result();
	}
	append(q, in.timestamp);
	statistics();
	const uint64_t span = in.timestamp - _history[_head].time;
	_result.gate_valid = span >= static_cast<uint64_t>(_config.window * 1000000.f)
	                     && _result.stddev <= _config.gstd && _result.same_sign_fraction >= _config.gsign;
	_result.gate_time = _history[_head].time;
	if (!_result.gate_valid || fabsf(_result.innovation) <= 0.f) {
		return result();
	}
	if ((in.positive_saturation && _result.innovation > 0.f)
	    || (in.negative_saturation && _result.innovation < 0.f)) {
		return result();
	}
	_result.learn = true;
	const float max_step = _config.slew * dt;
	const float drive = _config.gain * _result.innovation * dt;
	if (!PX4_ISFINITE(drive) || !PX4_ISFINITE(max_step)) {
		_result.learn = false;
		return result();
	}
	transfer(math::constrain(drive, -max_step, max_step));
	return result();
}

ResidualSlowFeedforwardMemory::Result ResidualSlowFeedforwardMemory::result() const
{
	Result out = _result;
	out.b = _b;
	out.i = _native.rollIntegralRaw();
	out.q_hat = _q_hat;
	out.estimate_valid = _estimate_valid;
	out.epoch = _epoch;
	out.samples = _count;
	out.evidence_time = _last_evidence;
	out.handback = _handback;
	return out;
}

float ResidualSlowFeedforwardMemory::composeOutput(float native_raw, float g, float trim) const
{
	return math::constrain(g * composeRaw(native_raw) + trim, -1.f, 1.f);
}
