#include "telemetry/telemetry_publisher.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

TelemetryPublisher::TelemetryPublisher() noexcept = default;

TelemetryPublisher::TelemetryPublisher(const std::string& host, std::uint16_t port) {
	if (port == 0) {
		throw std::invalid_argument("telemetry port must be nonzero");
	}
	socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_ < 0) {
		throw std::runtime_error(std::string("telemetry socket: ") + std::strerror(errno));
	}
	const int flags = ::fcntl(socket_, F_GETFL, 0);
	if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
		const std::string message = std::string("telemetry nonblocking: ") + std::strerror(errno);
		::close(socket_);
		socket_ = -1;
		throw std::runtime_error(message);
	}
	sockaddr_in address{};
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
		::close(socket_);
		socket_ = -1;
		throw std::invalid_argument("invalid telemetry IPv4 address: " + host);
	}
	if (::connect(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
		const std::string message = std::string("telemetry connect: ") + std::strerror(errno);
		::close(socket_);
		socket_ = -1;
		throw std::runtime_error(message);
	}
}

TelemetryPublisher::~TelemetryPublisher() {
	if (socket_ >= 0) {
		::close(socket_);
	}
}

bool TelemetryPublisher::publish(const TelemetryPacket& packet) noexcept {
	if (socket_ < 0) {
		return false;
	}
	const TelemetryCodec::Bytes bytes = TelemetryCodec::encode(packet);
	const ssize_t sent = ::send(socket_, bytes.data(), bytes.size(), 0);
	if (sent != static_cast<ssize_t>(bytes.size())) {
		++dropped_;
		return false;
	}
	return true;
}

bool TelemetryPublisher::enabled() const noexcept {
	return socket_ >= 0;
}

bool TelemetryPublisher::socket_is_blocking() const {
	if (socket_ < 0) {
		return false;
	}
	const int flags = ::fcntl(socket_, F_GETFL, 0);
	if (flags < 0) {
		throw std::runtime_error(std::string("telemetry flags: ") + std::strerror(errno));
	}
	return (flags & O_NONBLOCK) == 0;
}

std::uint64_t TelemetryPublisher::dropped_count() const noexcept {
	return dropped_;
}
