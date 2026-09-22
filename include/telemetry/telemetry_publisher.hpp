#pragma once

#include "telemetry/telemetry_packet.hpp"

#include <cstdint>
#include <string>

class TelemetryPublisher {
public:
	TelemetryPublisher() noexcept;
	TelemetryPublisher(const std::string& host, std::uint16_t port);
	~TelemetryPublisher();
	TelemetryPublisher(const TelemetryPublisher&) = delete;
	TelemetryPublisher& operator=(const TelemetryPublisher&) = delete;

	bool publish(const TelemetryPacket& packet) noexcept;
	bool enabled() const noexcept;
	bool socket_is_blocking() const;
	std::uint64_t dropped_count() const noexcept;

private:
	int socket_ = -1;
	std::uint64_t dropped_ = 0;
};
