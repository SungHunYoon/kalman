#pragma once

#include "filter/fixed_math.hpp"
#include "telemetry/telemetry_packet.hpp"

#include <array>
#include <cstddef>
#include <optional>

struct Bounds3d {
	Vector3d min;
	Vector3d max;
};

class TrajectoryBuffer {
public:
	static constexpr std::size_t ESTIMATE_CAPACITY = 54000;
	static constexpr std::size_t GPS_CAPACITY = 2048;

	void push(const TelemetryPacket& packet);
	void clear();
	std::size_t estimate_count() const;
	std::size_t gps_count() const;
	const Vector3d& estimate_at(std::size_t chronological_index) const;
	const Vector3d& gps_at(std::size_t chronological_index) const;
	std::optional<Bounds3d> estimate_bounds() const;
	bool estimate_history_truncated() const noexcept;

private:
	std::array<Vector3d, ESTIMATE_CAPACITY> estimates_{};
	std::array<Vector3d, GPS_CAPACITY> gps_{};
	std::size_t estimate_head_ = 0;
	std::size_t estimate_size_ = 0;
	std::size_t gps_head_ = 0;
	std::size_t gps_size_ = 0;
	bool estimate_history_truncated_ = false;
};
