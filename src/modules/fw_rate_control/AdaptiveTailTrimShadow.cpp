#include "AdaptiveTailTrimShadow.hpp"
// Buildable TDD scaffolding. No actuation, no claimed implementation.
void AdaptiveTailTrimEstimator::reset() { _samples[0] = _hat = _elapsed = _reference = 0.f; _head = _count = 0; _initialized = false; _result = {}; }
AdaptiveTailTrimEstimator::Result AdaptiveTailTrimEstimator::update(float, float, bool, bool, Config) { return {}; }
bool AdaptiveTailTrimManeuverGate::update(float, float, float, bool) { return false; }
CausalTailPitchEnvelope::Result CausalTailPitchEnvelope::update(float, float, uint64_t, uint64_t, float, bool) { return {}; }
void AdaptiveTailTrimShadow::configure(Config c) { _config = c; }
void AdaptiveTailTrimShadow::reset(uint32_t e) { _epoch = e; _offset = _target = 0.f; _last_time = 0; _enabled = _reversal = _rearm = false; _estimator.reset(); _maneuver.reset(); _pitch.reset(); (void)_core.state(); }
AdaptiveTailTrimShadow::Result AdaptiveTailTrimShadow::update(Inputs) { return {}; }
