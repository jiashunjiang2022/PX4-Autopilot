// Test-only state access and thin production adapter; no control algorithm here.
#pragma once
#include <array>
#include <cstdint>
#include "../ResidualSlowFeedforwardMemory.hpp"
namespace v4_contract
{
struct State {
	float b{}, i{}, q_hat{};
	float imax{.20f}, bmax{.10f}, reserve{.05f}, tau{5.f}, gain{.1f}, slew{.01f};
	float gstd{.025f}, gsign{.90f}, window{3.f}, legacy_hr{};
	bool estimate_valid{true}, gate_valid{}, apply{}, recovery{}, learn{};
	unsigned samples{}, epoch{}, native_epoch{};
	uint64_t evidence_time{}, gate_time{};
	std::array<float, 3> history{};
	float residual{}, innovation{}, accepted{}, stddev{}, positive_fraction{};
};
struct Input {
	uint64_t time{}, allocator_time{};
	bool enabled{true}, mission{true}, maneuver{}, positive_sat{}, negative_sat{};
};
class Subject
{
public:
// Test-only seeding/access, not a production interface.
	explicit Subject(const State &);
	State state() const;
	void sample(const Input &, float native_i);
	void tick(const Input &); // No new plant evidence.
// Lower-level accepted-pair test seam, after learning permission/slew selection.
// Still enforces BMAX/IMAX/total, actual deltas and reversal zero barrier.
// Must not authorize another learning transaction or refresh evidence.
	void requestPair(float delta_b);
	void naturalIntegrate(float rate_error, float dt);
	void disableStep(float dt);
	void limits(float imax, float bmax);
	void externalReset();
	void recomputeHistoryStatistics();
	float output(float pdff, float g, float trim) const;
private:
	RateControl _native;
	ResidualSlowFeedforwardMemory _memory{_native};
};
}
