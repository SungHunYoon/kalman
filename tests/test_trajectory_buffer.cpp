#include "visualizer/trajectory_buffer.hpp"

#include <stdexcept>

void run_trajectory_buffer_tests() {
	TrajectoryBuffer buffer;
	for (std::uint64_t i = 0; i < TrajectoryBuffer::ESTIMATE_CAPACITY + 10; ++i) {
		TelemetryPacket packet;
		packet.sequence = i * 10;
		packet.estimate_position = {static_cast<double>(i), 0, 0};
		buffer.push(packet);
	}
	if (buffer.estimate_count() != TrajectoryBuffer::ESTIMATE_CAPACITY ||
		buffer.estimate_at(0)[0] != 10.0 ||
		buffer.estimate_at(buffer.estimate_count() - 1)[0] != 54009.0) {
		throw std::runtime_error("estimate ring wraparound mismatch");
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
	if (buffer.estimate_count() != 0 || buffer.gps_count() != 0) {
		throw std::runtime_error("clear failed");
	}
}
