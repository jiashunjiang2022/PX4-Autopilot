// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cstddef>
#include <cstdint>
#include <lib/rate_control/rate_control.hpp>

// Core only: no subscriptions, parameters, publications or flight routing.
// Owner must serialize core, native integration, reset and configuration calls.
class ResidualSlowFeedforwardMemory final
{
public:
	struct Config {
		float imax{.20f}, bmax{.10f}, reserve{.05f}, tau{5.f};
		float gain{.1f}, slew{.01f}, window{3.f}, gstd{.025f}, gsign{.90f};
	};
	struct Inputs {
		uint64_t now{}, timestamp{};
		float control_dt{};
		bool fresh_sample{}, evidence_valid{true}, allocator_fresh{};
		bool positive_saturation{}, negative_saturation{};
		bool enabled{}, mission{}, stable{true}, maneuver{};
		bool pilot_abort{}, failsafe{}, hard_reset{}, control_valid{true};
	};
	enum class Reason : uint8_t {
		None, InvalidConfig, NonfinitePair, IllegalPair, EpochMismatch,
		HardReset, ControlInvalid, TransactionRejected, EstimatorInvalid,
		EvidenceGap, AllocatorUnknown, Disabled, PilotAbort, Failsafe
	};
	struct Result {
		float b{}, i{}, q_hat{}, residual{}, innovation{}, requested{}, accepted{};
		float stddev{}, positive_fraction{}, same_sign_fraction{};
		bool estimate_valid{}, gate_valid{}, learn{}, apply{}, recovery{}, handback{};
		uint32_t epoch{};
		size_t samples{};
		uint64_t evidence_time{}, gate_time{};
		Reason reason{Reason::None};
	};
	explicit ResidualSlowFeedforwardMemory(RateControl &native);
	bool configure(const Config &config);
	Result update(const Inputs &inputs);
	Result result() const;
	float composeRaw(float native_raw) const
	{
		return native_raw + _b;
	}
	float composeOutput(float native_raw, float g, float trim) const;
	static constexpr float Tolerance = 2e-7f;
	static constexpr uint64_t MaxGap = 150000;
private:
	friend struct ResidualSlowFeedforwardMemoryTestAccess;
	static constexpr size_t Capacity = 128;
	struct Evidence {
		float q{};
		uint64_t time{};
	};
	bool configValid(const Config &) const;
	bool checkPair();
	bool transfer(float requested);
	void handback(float dt);
	void recover(Reason reason);
	void holdInvalidConfig();
	void invalidateHistory();
	void invalidateEstimate();
	void statistics();
	void append(float q, uint64_t timestamp);
	void refreshResidual();
	void setContext();
	RateControl &_native;
	Config _config{};
	float _b{}, _q_hat{};
	bool _estimate_valid{}, _config_valid{true}, _handback{};
	uint32_t _epoch{};
	uint64_t _last_evidence{};
	bool _have_evidence{};
	Evidence _history[Capacity] {};
	size_t _head{}, _count{};
	Result _result{};
};
