#include "telemetry/telemetry_packet.hpp"

#include <limits>
#include <stdexcept>

void run_telemetry_packet_tests() {
	TelemetryPacket packet;
	packet.flags = TELEMETRY_INITIALIZED | TELEMETRY_GPS_PRESENT | TELEMETRY_GPS_ACCEPTED;
	packet.sequence = 0x0102030405060708ULL;
	packet.sensor_time_seconds = 3.0;
	packet.estimate_position = {1, 2, 3};
	packet.estimate_velocity = {4, 5, 6};
	packet.acceleration = {-1, -2, -3};
	packet.gps_position = {7, 8, 9};
	packet.innovation = {6, 6, 6};
	packet.position_variance = {0.1, 0.2, 0.3};
	packet.adaptive_gps_variance = {1.1, 1.2, 1.3};
	packet.filter_duration_us = 12.5;
	packet.accepted_gps_count = 4;
	packet.rejected_gps_count = 2;
	const auto bytes = TelemetryCodec::encode(packet);
	if (bytes.size() != 216 || bytes[0] != 'M' || bytes[1] != 'L' ||
		bytes[2] != 'A' || bytes[3] != 'K' || bytes[4] != 1 || bytes[5] != 0) {
		throw std::runtime_error("wire header mismatch");
	}
	TelemetryPacket decoded;
	if (!TelemetryCodec::decode(bytes.data(), bytes.size(), decoded) ||
		decoded.sequence != packet.sequence || decoded.estimate_position != packet.estimate_position ||
		decoded.adaptive_gps_variance != packet.adaptive_gps_variance ||
		decoded.rejected_gps_count != packet.rejected_gps_count) {
		throw std::runtime_error("telemetry round trip mismatch");
	}
	if (TelemetryCodec::decode(bytes.data(), bytes.size() - 1, decoded)) {
		throw std::runtime_error("truncated packet accepted");
	}
	auto wrong_version = bytes;
	wrong_version[4] = 2;
	if (TelemetryCodec::decode(wrong_version.data(), wrong_version.size(), decoded)) {
		throw std::runtime_error("wrong version accepted");
	}
	auto wrong_magic = bytes;
	wrong_magic[0] = 0;
	if (TelemetryCodec::decode(wrong_magic.data(), wrong_magic.size(), decoded)) {
		throw std::runtime_error("wrong magic accepted");
	}
	packet.estimate_position[0] = std::numeric_limits<double>::infinity();
	const auto invalid = TelemetryCodec::encode(packet);
	if (TelemetryCodec::decode(invalid.data(), invalid.size(), decoded)) {
		throw std::runtime_error("nonfinite packet accepted");
	}
}
