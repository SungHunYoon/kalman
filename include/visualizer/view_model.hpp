#pragma once

#include "telemetry/telemetry_packet.hpp"
#include "visualizer/trajectory_buffer.hpp"

#include <chrono>
#include <cstdint>

struct ViewerSnapshot {
	TelemetryPacket latest{};
	bool has_packet = false;
	bool connected = false;
	std::uint64_t lost_packets = 0;
	std::uint64_t malformed_packets = 0;
	std::uint64_t invalid_packets = 0;
};

class ViewerModel {
public:
	void accept(const TelemetryPacket& packet, std::chrono::steady_clock::time_point now);
	void record_malformed();
	void record_invalid();
	void set_history_paused(bool paused) noexcept;
	bool history_paused() const noexcept;
	ViewerSnapshot snapshot(std::chrono::steady_clock::time_point now) const;
	TrajectoryBuffer& trajectory();
	const TrajectoryBuffer& trajectory() const;

private:
	ViewerSnapshot state_{};
	TrajectoryBuffer trajectory_{};
	std::chrono::steady_clock::time_point last_packet_{};
	bool history_paused_ = false;
};
