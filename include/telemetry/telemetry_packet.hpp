#pragma once

#include "filter/fixed_math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

enum TelemetryFlags : std::uint16_t {
	TELEMETRY_INITIALIZED = 1u << 0,
	TELEMETRY_GPS_PRESENT = 1u << 1,
	TELEMETRY_GPS_ACCEPTED = 1u << 2,
	TELEMETRY_GPS_REJECTED = 1u << 3
};

struct TelemetryPacket {
	std::uint16_t flags = 0;
	std::uint64_t sequence = 0;
	double sensor_time_seconds = 0;
	Vector3d estimate_position{};
	Vector3d estimate_velocity{};
	Vector3d acceleration{};
	Vector3d gps_position{};
	Vector3d innovation{};
	Vector3d position_variance{};
	Vector3d adaptive_gps_variance{};
	double filter_duration_us = 0;
	std::uint64_t accepted_gps_count = 0;
	std::uint64_t rejected_gps_count = 0;
};

class TelemetryCodec {
public:
	static constexpr std::size_t WIRE_SIZE = 216;
	using Bytes = std::array<std::uint8_t, WIRE_SIZE>;

	static Bytes encode(const TelemetryPacket& packet);
	static bool decode(const std::uint8_t* bytes, std::size_t size, TelemetryPacket& packet);
};
