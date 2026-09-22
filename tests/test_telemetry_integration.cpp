#include "protocol/process_message.hpp"
#include "telemetry/telemetry_builder.hpp"

#include <stdexcept>
#include <string>
#include <vector>

void run_telemetry_integration_tests() {
	TelemetryBuilder builder;
	SensorUpdate update;
	update.time = 3.0;
	update.acceleration = Vector<double>{1, 2, 3};
	update.gps = Vector<double>{4, 5, 6};
	FilterSnapshot snapshot;
	snapshot.position = {7, 8, 9};
	snapshot.velocity = {10, 11, 12};
	snapshot.position_variance = {0.1, 0.2, 0.3};
	snapshot.gps_variance = {1, 2, 3};
	GpsUpdateResult gps;
	gps.accepted = true;
	gps.innovation = {-3, -3, -3};
	const TelemetryPacket first = builder.build(update, snapshot, gps, {}, 2, 1);
	const TelemetryPacket second = builder.build(update, snapshot, gps, {}, 3, 1);
	if (first.sequence != 0 || second.sequence != 1 ||
		(first.flags & TELEMETRY_GPS_PRESENT) == 0 ||
		(first.flags & TELEMETRY_GPS_ACCEPTED) == 0 ||
		first.acceleration != Vector3d{1, 2, 3} || first.filter_duration_us != 0) {
		throw std::runtime_error("telemetry builder mismatch");
	}

	SensorState state;
	SensorUpdate initial;
	initial.time = 0.0;
	initial.initial_position = Vector<double>{0, 0, 0};
	initial.initial_speed_kmh = 0.0;
	initial.direction = Vector<double>{0, 0, 0};
	std::vector<std::string> events;
	process_message(state, initial, builder,
		[&events](const Vector<double>&) { events.push_back("response"); },
		[&events](const TelemetryPacket&) { events.push_back("telemetry"); });
	if (events != std::vector<std::string>{"response", "telemetry"}) {
		throw std::runtime_error("response was not sent before telemetry");
	}
}
