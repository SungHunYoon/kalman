#include "visualizer/view_model.hpp"

#include <chrono>
#include <limits>

void ViewerModel::accept(const TelemetryPacket& packet,
	std::chrono::steady_clock::time_point now) {
	// A new client starts its sequence at zero. Preserve the true uint64 wrap
	// from max to zero, but discard the previous client's session state.
	if (state_.has_packet && packet.sequence == 0 && state_.latest.sequence != 0 &&
		state_.latest.sequence != std::numeric_limits<std::uint64_t>::max()) {
		const std::uint64_t malformed = state_.malformed_packets;
		const std::uint64_t invalid = state_.invalid_packets;
		state_ = ViewerSnapshot{};
		state_.malformed_packets = malformed;
		state_.invalid_packets = invalid;
		timing_stats_ = FilterStats{0};
		trajectory_.clear();
	}
	if (state_.has_packet) {
		const std::uint64_t delta = packet.sequence - state_.latest.sequence;
		const bool newer = delta != 0 && delta <= std::numeric_limits<std::uint64_t>::max() / 2;
		if (!newer) {
			return;
		}
		state_.lost_packets += delta - 1;
	}
	last_packet_ = now;
	state_.latest = packet;
	state_.has_packet = true;
	if (packet.flags & TELEMETRY_GPS_PRESENT) {
		state_.has_last_gps_innovation = true;
		state_.last_gps_accepted = (packet.flags & TELEMETRY_GPS_ACCEPTED) != 0;
		state_.last_gps_innovation = packet.innovation;
	}
	timing_stats_.record(packet.filter_duration_us);
	if (!history_paused_) {
		trajectory_.push(packet);
	}
}

void ViewerModel::record_malformed() { ++state_.malformed_packets; }
void ViewerModel::record_invalid() { ++state_.invalid_packets; }
void ViewerModel::set_history_paused(bool paused) noexcept { history_paused_ = paused; }
bool ViewerModel::history_paused() const noexcept { return history_paused_; }

ViewerSnapshot ViewerModel::snapshot(std::chrono::steady_clock::time_point now) const {
	ViewerSnapshot result = state_;
	result.timing = timing_stats_.snapshot();
	result.connected = result.has_packet && now >= last_packet_ &&
		now - last_packet_ <= std::chrono::seconds(2);
	return result;
}

TrajectoryBuffer& ViewerModel::trajectory() { return trajectory_; }
const TrajectoryBuffer& ViewerModel::trajectory() const { return trajectory_; }
