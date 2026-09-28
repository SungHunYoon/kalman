#include "visualizer/trajectory_buffer.hpp"

#include <algorithm>
#include <stdexcept>

namespace {
	std::size_t chronological_index(std::size_t head, std::size_t size,
		std::size_t capacity, std::size_t index) {
		if (index >= size) {
			throw std::out_of_range("trajectory index out of range");
		}
		return (head + capacity - size + index) % capacity;
	}
}

void TrajectoryBuffer::push(const TelemetryPacket& packet) {
	if (packet.sequence % 10 == 0) {
		if (estimate_size_ == ESTIMATE_CAPACITY) {
			estimate_history_truncated_ = true;
		}
		estimates_[estimate_head_] = packet.estimate_position;
		estimate_head_ = (estimate_head_ + 1) % ESTIMATE_CAPACITY;
		estimate_size_ = std::min(estimate_size_ + 1, ESTIMATE_CAPACITY);
	}
	if ((packet.flags & TELEMETRY_GPS_PRESENT) != 0) {
		gps_[gps_head_] = packet.gps_position;
		gps_head_ = (gps_head_ + 1) % GPS_CAPACITY;
		gps_size_ = std::min(gps_size_ + 1, GPS_CAPACITY);
	}
}

void TrajectoryBuffer::clear() {
	estimate_head_ = 0;
	estimate_size_ = 0;
	gps_head_ = 0;
	gps_size_ = 0;
	estimate_history_truncated_ = false;
}

std::size_t TrajectoryBuffer::estimate_count() const { return estimate_size_; }
std::size_t TrajectoryBuffer::gps_count() const { return gps_size_; }

const Vector3d& TrajectoryBuffer::estimate_at(std::size_t index) const {
	return estimates_[chronological_index(estimate_head_, estimate_size_, ESTIMATE_CAPACITY, index)];
}

const Vector3d& TrajectoryBuffer::gps_at(std::size_t index) const {
	return gps_[chronological_index(gps_head_, gps_size_, GPS_CAPACITY, index)];
}

std::optional<Bounds3d> TrajectoryBuffer::estimate_bounds() const {
	if (estimate_size_ == 0) {
		return std::nullopt;
	}
	Bounds3d bounds{estimate_at(0), estimate_at(0)};
	for (std::size_t index = 1; index < estimate_size_; ++index) {
		const Vector3d& point = estimate_at(index);
		for (std::size_t axis = 0; axis < point.size(); ++axis) {
			bounds.min[axis] = std::min(bounds.min[axis], point[axis]);
			bounds.max[axis] = std::max(bounds.max[axis], point[axis]);
		}
	}
	return bounds;
}

bool TrajectoryBuffer::estimate_history_truncated() const noexcept {
	return estimate_history_truncated_;
}
