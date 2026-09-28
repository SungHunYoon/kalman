#include "visualizer/trajectory_buffer.hpp"

#include <stdexcept>

void run_trajectory_buffer_tests() {
	TrajectoryBuffer overview;
	if (overview.estimate_bounds() || overview.estimate_history_truncated()) {
		throw std::runtime_error("empty estimate history reported bounds or truncation");
	}
	TelemetryPacket first;
	first.sequence = 0;
	first.estimate_position = {-5.0, 2.0, 10.0};
	overview.push(first);
	TelemetryPacket second;
	second.sequence = 10;
	second.estimate_position = {8.0, -3.0, 12.0};
	overview.push(second);
	const auto bounds = overview.estimate_bounds();
	if (!bounds || bounds->min != Vector3d{-5.0, -3.0, 10.0} ||
		bounds->max != Vector3d{8.0, 2.0, 12.0}) {
		throw std::runtime_error("estimate bounds mismatch");
	}
	TelemetryPacket gps_only;
	gps_only.sequence = 11;
	gps_only.flags = TELEMETRY_GPS_PRESENT;
	gps_only.gps_position = {1e9, 1e9, 1e9};
	overview.push(gps_only);
	if (!overview.estimate_bounds() ||
		overview.estimate_bounds()->max != Vector3d{8.0, 2.0, 12.0}) {
		throw std::runtime_error("GPS changed estimate bounds");
	}

	TrajectoryBuffer buffer;
	for (std::uint64_t i = 0; i < TrajectoryBuffer::ESTIMATE_CAPACITY + 10; ++i) {
		TelemetryPacket packet;
		packet.sequence = i * 10;
		packet.estimate_position = {static_cast<double>(i), 0, 0};
		buffer.push(packet);
		if (i == TrajectoryBuffer::ESTIMATE_CAPACITY - 1 &&
			buffer.estimate_history_truncated()) {
			throw std::runtime_error("full estimate ring reported truncation before overwrite");
		}
	}
	if (buffer.estimate_count() != TrajectoryBuffer::ESTIMATE_CAPACITY ||
		buffer.estimate_at(0)[0] != 10.0 ||
		buffer.estimate_at(buffer.estimate_count() - 1)[0] != 54009.0) {
		throw std::runtime_error("estimate ring wraparound mismatch");
	}
	const auto retained_bounds = buffer.estimate_bounds();
	if (!buffer.estimate_history_truncated() || !retained_bounds ||
		retained_bounds->min != Vector3d{10.0, 0.0, 0.0} ||
		retained_bounds->max != Vector3d{54009.0, 0.0, 0.0}) {
		throw std::runtime_error("retained estimate bounds or truncation mismatch");
	}
	TelemetryPacket skipped;
	skipped.sequence = 540091;
	buffer.push(skipped);
	if (buffer.estimate_at(buffer.estimate_count() - 1)[0] != 54009.0) {
		throw std::runtime_error("estimate downsampling mismatch");
	}
	TelemetryPacket gps;
	gps.sequence = 540092;
	gps.flags = TELEMETRY_GPS_PRESENT;
	gps.gps_position = {1, 2, 3};
	buffer.push(gps);
	if (buffer.gps_count() != 1 || buffer.gps_at(0) != Vector3d{1, 2, 3}) {
		throw std::runtime_error("GPS storage mismatch");
	}
	bool threw = false;
	try { (void)buffer.gps_at(1); } catch (const std::out_of_range&) { threw = true; }
	if (!threw || sizeof(TrajectoryBuffer) >= 2 * 1024 * 1024) {
		throw std::runtime_error("trajectory bounds or fixed size mismatch");
	}
	buffer.clear();
	if (buffer.estimate_count() != 0 || buffer.gps_count() != 0 ||
		buffer.estimate_history_truncated() || buffer.estimate_bounds()) {
		throw std::runtime_error("clear failed");
	}
}
