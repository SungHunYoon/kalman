#include "visualizer/view_model.hpp"

#include <chrono>
#include <limits>

void ViewerModel::accept(const TelemetryPacket& packet,
	std::chrono::steady_clock::time_point now) {
	last_packet_ = now;
	if (state_.has_packet) {
		const std::uint64_t delta = packet.sequence - state_.latest.sequence;
		const bool newer = delta != 0 && delta <= std::numeric_limits<std::uint64_t>::max() / 2;
		if (!newer) {
			return;
		}
		state_.lost_packets += delta - 1;
	}
	state_.latest = packet;
	state_.has_packet = true;
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
	result.connected = result.has_packet && now >= last_packet_ &&
		now - last_packet_ <= std::chrono::seconds(2);
	return result;
}

TrajectoryBuffer& ViewerModel::trajectory() { return trajectory_; }
const TrajectoryBuffer& ViewerModel::trajectory() const { return trajectory_; }
