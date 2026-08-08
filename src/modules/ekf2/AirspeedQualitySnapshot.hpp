#pragma once

#include <cstddef>
#include <cstdint>

namespace airspeed_quality
{

enum class SnapshotSelectionStatus : uint8_t {
	Fresh = 0,
	Stale,
	NoHistory,
	FutureOnly,
};

template<typename T>
struct SnapshotSelection {
	const T *value{nullptr};
	uint64_t timestamp_sample{0};
	uint32_t age_us{0};
	SnapshotSelectionStatus status{SnapshotSelectionStatus::NoHistory};

	bool causal() const { return value != nullptr; }
	bool fresh() const { return status == SnapshotSelectionStatus::Fresh; }
};

template<typename T, size_t Capacity>
class SnapshotRing
{
public:
	static_assert(Capacity > 0, "snapshot ring capacity must be positive");

	void reset()
	{
		_next = 0;
		_count = 0;
	}

	bool push(uint64_t timestamp_sample, const T &value)
	{
		if ((timestamp_sample == 0) || (_count > 0 && timestamp_sample <= newest_timestamp())) {
			return false;
		}

		_entries[_next] = {timestamp_sample, value};
		_next = (_next + 1) % Capacity;
		_count = _count < Capacity ? _count + 1 : Capacity;
		return true;
	}

	SnapshotSelection<T> select(uint64_t observation_timestamp, uint32_t maximum_age_us) const
	{
		if (_count == 0) {
			return {};
		}

		for (size_t offset = 0; offset < _count; ++offset) {
			const size_t index = (_next + Capacity - 1 - offset) % Capacity;
			const Entry &entry = _entries[index];

			if (entry.timestamp_sample <= observation_timestamp) {
				const uint64_t age = observation_timestamp - entry.timestamp_sample;
				return {
					&entry.value,
					entry.timestamp_sample,
					age > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age),
					age <= maximum_age_us ? SnapshotSelectionStatus::Fresh : SnapshotSelectionStatus::Stale,
				};
			}
		}

		SnapshotSelection<T> result{};
		result.status = SnapshotSelectionStatus::FutureOnly;
		return result;
	}

	size_t size() const { return _count; }
	bool empty() const { return _count == 0; }

	uint64_t newest_timestamp() const
	{
		return _count > 0 ? _entries[(_next + Capacity - 1) % Capacity].timestamp_sample : 0;
	}

private:
	struct Entry {
		uint64_t timestamp_sample{0};
		T value{};
	};

	Entry _entries[Capacity]{};
	size_t _next{0};
	size_t _count{0};
};

} // namespace airspeed_quality
