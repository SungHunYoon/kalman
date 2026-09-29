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

	std::vector<TelemetryPacket> packets;
	SensorUpdate no_gps;
	no_gps.time = 1.0;
	no_gps.acceleration = Vector<double>{0, 0, 0};
	process_message(state, no_gps, builder,
		[](const Vector<double>&) {},
		[&packets](const TelemetryPacket& packet) { packets.push_back(packet); });
	SensorUpdate measured;
	measured.time = 2.0;
	measured.gps = Vector<double>{4, 0, 0};
	process_message(state, measured, builder,
		[](const Vector<double>&) {},
		[&packets](const TelemetryPacket& packet) { packets.push_back(packet); });
	if (packets.size() != 2 ||
		(packets[0].flags & (TELEMETRY_GPS_PRESENT | TELEMETRY_GPS_ACCEPTED)) != 0 ||
		packets[0].innovation != Vector3d{0, 0, 0} ||
		(packets[1].flags & (TELEMETRY_GPS_PRESENT | TELEMETRY_GPS_ACCEPTED)) !=
			(TELEMETRY_GPS_PRESENT | TELEMETRY_GPS_ACCEPTED) ||
		packets[1].innovation != Vector3d{4, 0, 0} ||
		packets[1].accepted_gps_count != 1 || packets[1].rejected_gps_count != 0 ||
		packets[1].adaptive_gps_variance != Vector3d{1, 1, 1}) {
		throw std::runtime_error("original filter telemetry mismatch");
	}
	const TelemetryCodec::Bytes wire = TelemetryCodec::encode(packets[1]);
	TelemetryPacket decoded;
	if (!TelemetryCodec::decode(wire.data(), wire.size(), decoded) ||
		decoded.flags != packets[1].flags ||
		decoded.innovation != packets[1].innovation ||
		decoded.adaptive_gps_variance != packets[1].adaptive_gps_variance) {
		throw std::runtime_error("original filter telemetry did not survive codec");
	}

	SensorUpdate stale;
	stale.time = 1.5;
	stale.acceleration = Vector<double>{100, 0, 0};
	stale.gps = Vector<double>{999, 0, 0};
	std::size_t stale_responses = 0;
	process_message(state, stale, builder,
		[&stale_responses](const Vector<double>&) { ++stale_responses; },
		[&packets](const TelemetryPacket& packet) { packets.push_back(packet); });
	if (stale_responses != 1 || packets.size() != 2 ||
		state.accepted_gps_count() != 1 || state.rejected_gps_count() != 0) {
		throw std::runtime_error("ignored stale GPS was published or applied");
	}

	GpsUpdateResult legacy_rejected;
	const TelemetryPacket applied = builder.build(measured, state.filter_snapshot(),
		legacy_rejected, state.timing_snapshot(), 1, 0);
	if ((applied.flags & TELEMETRY_GPS_ACCEPTED) == 0 ||
		(applied.flags & TELEMETRY_GPS_REJECTED) != 0) {
		throw std::runtime_error("original filter telemetry exposed a GPS rejection");
	}
}
