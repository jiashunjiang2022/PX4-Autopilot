// SPDX-License-Identifier: BSD-3-Clause
#include "ResidualSlowFeedforwardMemoryContract.hpp"

// Fixture-only access. Direct writes here seed inputs/faults; no control law.
struct ResidualSlowFeedforwardMemoryTestAccess {
	static void seed(ResidualSlowFeedforwardMemory &m, RateControl &rc, const v4_contract::State &s)
	{
		rc._rate_int(0) = s.i;
		rc._roll_i_reset_epoch = s.native_epoch;
		m._b = s.b;
		m._q_hat = s.q_hat;
		m._estimate_valid = s.estimate_valid;
		m._epoch = s.epoch;
		m._last_evidence = s.evidence_time;
		m._have_evidence = s.evidence_time != 0;
		m._result.gate_time = s.gate_time;
		for (size_t j = 0; j < 3; ++j) {
			m._history[j].q = s.history[j];
		}
		m._count = s.samples;
		m.refreshResidual();
		m.setContext();
	}
	static void nativeInput(RateControl &rc, float i)
	{
		rc._rate_int(0) = i;
	}
	static void pair(ResidualSlowFeedforwardMemory &m, float d)
	{
		m.transfer(d);
	}
	static void stats(ResidualSlowFeedforwardMemory &m)
	{
		m._count = 3;
		m.statistics();
	}
	static ResidualSlowFeedforwardMemory::Config config(const ResidualSlowFeedforwardMemory &m)
	{
		return m._config;
	}
	static void history(const ResidualSlowFeedforwardMemory &m, v4_contract::State &s)
	{
		for (size_t j = 0; j < 3; ++j) {
			s.history[j] = m._history[j].q;
		}
	}
};
namespace v4_contract
{
namespace
{
ResidualSlowFeedforwardMemory::Inputs input(const Input &in)
{
	ResidualSlowFeedforwardMemory::Inputs out;
	out.now = in.time;
	out.timestamp = in.time;
	out.control_dt = .05f;
	out.enabled = in.enabled;
	out.mission = in.mission;
	out.maneuver = in.maneuver;
// Routing conversion only. Core deliberately consumes freshness as a boolean.
	out.allocator_fresh = in.allocator_time <= in.time && in.time - in.allocator_time <= 100000;
	out.positive_saturation = in.positive_sat;
	out.negative_saturation = in.negative_sat;
	return out;
}
}
Subject::Subject(const State &s)
{
	_native.setPidGains(matrix::Vector3f(), matrix::Vector3f(1.f, 0.f, 0.f), matrix::Vector3f());
	_native.setFeedForwardGain(matrix::Vector3f());
	_native.setIntegratorLimit(matrix::Vector3f(s.imax, .3f, .4f));
	ResidualSlowFeedforwardMemory::Config c;
	c.imax = s.imax;
	c.bmax = s.bmax;
	c.reserve = s.reserve;
	c.tau = s.tau;
	c.gain = s.gain;
	c.slew = s.slew;
	c.gstd = s.gstd;
	c.gsign = s.gsign;
	c.window = s.window;
	_memory.configure(c);
	ResidualSlowFeedforwardMemoryTestAccess::seed(_memory, _native, s);
}
State Subject::state() const
{
	State s;
	const auto r = _memory.result();
	const auto c = ResidualSlowFeedforwardMemoryTestAccess::config(_memory);
	s.b = r.b;
	s.i = r.i;
	s.q_hat = r.q_hat;
	s.imax = c.imax;
	s.bmax = c.bmax;
	s.reserve = c.reserve;
	s.tau = c.tau;
	s.gain = c.gain;
	s.slew = c.slew;
	s.gstd = c.gstd;
	s.gsign = c.gsign;
	s.window = c.window;
	s.estimate_valid = r.estimate_valid;
	s.gate_valid = r.gate_valid;
	s.apply = r.apply;
	s.recovery = r.recovery;
	s.learn = r.learn;
	s.samples = r.samples;
	s.epoch = r.epoch;
	s.native_epoch = _native.rollIntegralResetEpoch();
	s.evidence_time = r.evidence_time;
	s.gate_time = r.gate_time;
	s.residual = r.residual;
	s.innovation = r.innovation;
	s.accepted = r.accepted;
	s.stddev = r.stddev;
	s.positive_fraction = r.positive_fraction;
	ResidualSlowFeedforwardMemoryTestAccess::history(_memory, s);
	return s;
}
void Subject::sample(const Input &in, float i)
{
	ResidualSlowFeedforwardMemoryTestAccess::nativeInput(_native, i);
	auto v = input(in);
	v.fresh_sample = true;
	_memory.update(v);
}
void Subject::tick(const Input &in)
{
	_memory.update(input(in));
}
void Subject::requestPair(float d)
{
	ResidualSlowFeedforwardMemoryTestAccess::pair(_memory, d);
}
void Subject::naturalIntegrate(float error, float dt)
{
	_native.update(matrix::Vector3f(), matrix::Vector3f(error, 0.f, 0.f), matrix::Vector3f(), dt, false);
}
void Subject::disableStep(float dt)
{
	ResidualSlowFeedforwardMemory::Inputs in;
	in.control_dt = dt;
	_memory.update(in);
}
void Subject::limits(float imax, float bmax)
{
	auto c = ResidualSlowFeedforwardMemoryTestAccess::config(_memory);
	c.imax = imax;
	c.bmax = bmax;
	_memory.configure(c);
}
void Subject::externalReset()
{
	_native.resetIntegral(0);
}
void Subject::recomputeHistoryStatistics()
{
	ResidualSlowFeedforwardMemoryTestAccess::stats(_memory);
}
float Subject::output(float pdff, float g, float trim) const
{
	return _memory.composeOutput(pdff + _native.rollIntegralRaw(), g, trim);
}
}

