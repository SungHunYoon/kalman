#pragma once

#include "filter/filter_snapshot.hpp"
#include "performance/filter_stats.hpp"
#include "protocol/sensor_update.hpp"
#include "telemetry/telemetry_packet.hpp"

#include <cstdint>

class TelemetryBuilder {
public:
	TelemetryPacket build(const SensorUpdate& update,
		const FilterSnapshot& filter,
		const GpsUpdateResult& gps,
		const FilterTimingSnapshot& timing,
		std::uint64_t accepted_gps,
		std::uint64_t rejected_gps);

private:
	std::uint64_t sequence_ = 0;
};
