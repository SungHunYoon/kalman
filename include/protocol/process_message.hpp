#pragma once

#include "protocol/sensor_state.hpp"
#include "telemetry/telemetry_builder.hpp"

template <typename ResponseSender, typename TelemetrySender>
void process_message(SensorState& state, const SensorUpdate& update,
	TelemetryBuilder& builder, ResponseSender&& send_response,
	TelemetrySender&& send_telemetry, bool telemetry_enabled = true) {
	const bool applied = state.apply(update);
	if (!state.has_estimated_position()) {
		return;
	}
	send_response(state.estimated_position());
	if (!applied || !telemetry_enabled) {
		return;
	}
	const TelemetryPacket packet = builder.build(update, state.filter_snapshot(),
		state.last_gps_result(), state.timing_snapshot(false), state.accepted_gps_count(),
		state.rejected_gps_count());
	send_telemetry(packet);
}
