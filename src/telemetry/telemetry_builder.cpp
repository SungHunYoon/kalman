#include "telemetry/telemetry_builder.hpp"

namespace {
	Vector3d optional_vector(const std::optional<Vector<double>>& vector) {
		if (!vector) {
			return {};
		}
		return Vector3d{(*vector)[0], (*vector)[1], (*vector)[2]};
	}
}

TelemetryPacket TelemetryBuilder::build(const SensorUpdate& update,
	const FilterSnapshot& filter,
	const GpsUpdateResult& gps,
	const FilterTimingSnapshot& timing,
	std::uint64_t accepted_gps,
	std::uint64_t rejected_gps) {
	TelemetryPacket packet;
	packet.flags = TELEMETRY_INITIALIZED;
	if (update.gps) {
		packet.flags |= TELEMETRY_GPS_PRESENT;
		packet.flags |= gps.accepted ? TELEMETRY_GPS_ACCEPTED : TELEMETRY_GPS_REJECTED;
	}
	packet.sequence = sequence_++;
	packet.sensor_time_seconds = update.time;
	packet.estimate_position = filter.position;
	packet.estimate_velocity = filter.velocity;
	packet.acceleration = optional_vector(update.acceleration);
	packet.gps_position = optional_vector(update.gps);
	packet.innovation = update.gps ? gps.innovation : Vector3d{};
	packet.position_variance = filter.position_variance;
	packet.adaptive_gps_variance = filter.gps_variance;
	packet.filter_duration_us = timing.current;
	packet.accepted_gps_count = accepted_gps;
	packet.rejected_gps_count = rejected_gps;
	return packet;
}
