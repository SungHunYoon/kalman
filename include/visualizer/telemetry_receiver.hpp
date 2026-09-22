#pragma once

#include "visualizer/view_model.hpp"

#include <cstddef>
#include <cstdint>

class TelemetryReceiver {
public:
	explicit TelemetryReceiver(std::uint16_t port);
	~TelemetryReceiver();
	TelemetryReceiver(const TelemetryReceiver&) = delete;
	TelemetryReceiver& operator=(const TelemetryReceiver&) = delete;

	std::size_t drain(ViewerModel& model) noexcept;
	std::uint16_t bound_port() const noexcept;

private:
	int socket_ = -1;
	std::uint16_t bound_port_ = 0;
};
