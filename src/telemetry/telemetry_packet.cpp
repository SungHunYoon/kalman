#include "telemetry/telemetry_packet.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
	constexpr std::uint32_t MAGIC = 0x4B414C4D;
	constexpr std::uint16_t VERSION = 1;
	constexpr std::uint16_t KNOWN_FLAGS = TELEMETRY_INITIALIZED | TELEMETRY_GPS_PRESENT |
		TELEMETRY_GPS_ACCEPTED | TELEMETRY_GPS_REJECTED;

	void write_u16(TelemetryCodec::Bytes& bytes, std::size_t& offset, std::uint16_t value) {
		for (unsigned int shift = 0; shift < 16; shift += 8) {
			bytes[offset++] = static_cast<std::uint8_t>(value >> shift);
		}
	}

	void write_u32(TelemetryCodec::Bytes& bytes, std::size_t& offset, std::uint32_t value) {
		for (unsigned int shift = 0; shift < 32; shift += 8) {
			bytes[offset++] = static_cast<std::uint8_t>(value >> shift);
		}
	}

	void write_u64(TelemetryCodec::Bytes& bytes, std::size_t& offset, std::uint64_t value) {
		for (unsigned int shift = 0; shift < 64; shift += 8) {
			bytes[offset++] = static_cast<std::uint8_t>(value >> shift);
		}
	}

	void write_double(TelemetryCodec::Bytes& bytes, std::size_t& offset, double value) {
		std::uint64_t bits = 0;
		static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
		std::memcpy(&bits, &value, sizeof(bits));
		write_u64(bytes, offset, bits);
	}

	std::uint16_t read_u16(const std::uint8_t* bytes, std::size_t& offset) {
		std::uint16_t value = 0;
		for (unsigned int shift = 0; shift < 16; shift += 8) {
			value |= static_cast<std::uint16_t>(bytes[offset++]) << shift;
		}
		return value;
	}

	std::uint32_t read_u32(const std::uint8_t* bytes, std::size_t& offset) {
		std::uint32_t value = 0;
		for (unsigned int shift = 0; shift < 32; shift += 8) {
			value |= static_cast<std::uint32_t>(bytes[offset++]) << shift;
		}
		return value;
	}

	std::uint64_t read_u64(const std::uint8_t* bytes, std::size_t& offset) {
		std::uint64_t value = 0;
		for (unsigned int shift = 0; shift < 64; shift += 8) {
			value |= static_cast<std::uint64_t>(bytes[offset++]) << shift;
		}
		return value;
	}

	double read_double(const std::uint8_t* bytes, std::size_t& offset) {
		const std::uint64_t bits = read_u64(bytes, offset);
		double value = 0;
		std::memcpy(&value, &bits, sizeof(value));
		return value;
	}

	void write_vector(TelemetryCodec::Bytes& bytes, std::size_t& offset, const Vector3d& vector) {
		for (double value : vector) {
			write_double(bytes, offset, value);
		}
	}

	void read_vector(const std::uint8_t* bytes, std::size_t& offset, Vector3d& vector) {
		for (double& value : vector) {
			value = read_double(bytes, offset);
		}
	}

	bool vector_finite(const Vector3d& vector) {
		return std::all_of(vector.begin(), vector.end(), [](double value) {
			return std::isfinite(value);
		});
	}

	bool packet_valid(const TelemetryPacket& packet) {
		const bool known_flags = (packet.flags & ~KNOWN_FLAGS) == 0;
		const bool accepted = (packet.flags & TELEMETRY_GPS_ACCEPTED) != 0;
		const bool rejected = (packet.flags & TELEMETRY_GPS_REJECTED) != 0;
		const bool gps_present = (packet.flags & TELEMETRY_GPS_PRESENT) != 0;
		return known_flags && !(accepted && rejected) &&
			((!accepted && !rejected) || gps_present) &&
			std::isfinite(packet.sensor_time_seconds) &&
			vector_finite(packet.estimate_position) && vector_finite(packet.estimate_velocity) &&
			vector_finite(packet.acceleration) && vector_finite(packet.gps_position) &&
			vector_finite(packet.innovation) && vector_finite(packet.position_variance) &&
			vector_finite(packet.adaptive_gps_variance) &&
			std::isfinite(packet.filter_duration_us);
	}
}

TelemetryCodec::Bytes TelemetryCodec::encode(const TelemetryPacket& packet) {
	Bytes bytes{};
	std::size_t offset = 0;
	write_u32(bytes, offset, MAGIC);
	write_u16(bytes, offset, VERSION);
	write_u16(bytes, offset, packet.flags);
	write_u64(bytes, offset, packet.sequence);
	write_double(bytes, offset, packet.sensor_time_seconds);
	write_vector(bytes, offset, packet.estimate_position);
	write_vector(bytes, offset, packet.estimate_velocity);
	write_vector(bytes, offset, packet.acceleration);
	write_vector(bytes, offset, packet.gps_position);
	write_vector(bytes, offset, packet.innovation);
	write_vector(bytes, offset, packet.position_variance);
	write_vector(bytes, offset, packet.adaptive_gps_variance);
	write_double(bytes, offset, packet.filter_duration_us);
	write_u64(bytes, offset, packet.accepted_gps_count);
	write_u64(bytes, offset, packet.rejected_gps_count);
	return bytes;
}

bool TelemetryCodec::decode(const std::uint8_t* bytes, std::size_t size,
	TelemetryPacket& packet) {
	if (bytes == nullptr || size != WIRE_SIZE) {
		return false;
	}
	std::size_t offset = 0;
	if (read_u32(bytes, offset) != MAGIC || read_u16(bytes, offset) != VERSION) {
		return false;
	}
	TelemetryPacket candidate;
	candidate.flags = read_u16(bytes, offset);
	candidate.sequence = read_u64(bytes, offset);
	candidate.sensor_time_seconds = read_double(bytes, offset);
	read_vector(bytes, offset, candidate.estimate_position);
	read_vector(bytes, offset, candidate.estimate_velocity);
	read_vector(bytes, offset, candidate.acceleration);
	read_vector(bytes, offset, candidate.gps_position);
	read_vector(bytes, offset, candidate.innovation);
	read_vector(bytes, offset, candidate.position_variance);
	read_vector(bytes, offset, candidate.adaptive_gps_variance);
	candidate.filter_duration_us = read_double(bytes, offset);
	candidate.accepted_gps_count = read_u64(bytes, offset);
	candidate.rejected_gps_count = read_u64(bytes, offset);
	if (offset != WIRE_SIZE || !packet_valid(candidate)) {
		return false;
	}
	packet = candidate;
	return true;
}
