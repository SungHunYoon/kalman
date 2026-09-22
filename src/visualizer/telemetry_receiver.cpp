#include "visualizer/telemetry_receiver.hpp"

#include "telemetry/telemetry_packet.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>

TelemetryReceiver::TelemetryReceiver(std::uint16_t port) {
	socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_ < 0) throw std::runtime_error(std::string("viewer socket: ") + std::strerror(errno));
	const int flags = ::fcntl(socket_, F_GETFL, 0);
	if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
		const std::string message = std::string("viewer nonblocking: ") + std::strerror(errno);
		::close(socket_);
		socket_ = -1;
		throw std::runtime_error(message);
	}
	sockaddr_in address{};
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	address.sin_port = htons(port);
	if (::bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
		const std::string message = std::string("viewer bind: ") + std::strerror(errno);
		::close(socket_);
		socket_ = -1;
		throw std::runtime_error(message);
	}
	socklen_t size = sizeof(address);
	if (::getsockname(socket_, reinterpret_cast<sockaddr*>(&address), &size) < 0) {
		const std::string message = std::string("viewer getsockname: ") + std::strerror(errno);
		::close(socket_);
		socket_ = -1;
		throw std::runtime_error(message);
	}
	bound_port_ = ntohs(address.sin_port);
}

TelemetryReceiver::~TelemetryReceiver() {
	if (socket_ >= 0) ::close(socket_);
}

std::size_t TelemetryReceiver::drain(ViewerModel& model) noexcept {
	std::size_t accepted = 0;
	std::array<std::uint8_t, TelemetryCodec::WIRE_SIZE + 1> bytes{};
	while (true) {
		const ssize_t received = ::recv(socket_, bytes.data(), bytes.size(), 0);
		if (received < 0) {
			if (errno == EINTR) continue;
			break;
		}
		if (received != static_cast<ssize_t>(TelemetryCodec::WIRE_SIZE)) {
			model.record_malformed();
			continue;
		}
		TelemetryPacket packet;
		if (!TelemetryCodec::decode(bytes.data(), static_cast<std::size_t>(received), packet)) {
			model.record_invalid();
			continue;
		}
		model.accept(packet, std::chrono::steady_clock::now());
		++accepted;
	}
	return accepted;
}

std::uint16_t TelemetryReceiver::bound_port() const noexcept { return bound_port_; }
