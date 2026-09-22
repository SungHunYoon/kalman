#include "telemetry/telemetry_publisher.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <stdexcept>

void run_telemetry_publisher_tests() {
	TelemetryPacket packet;
	packet.flags = TELEMETRY_INITIALIZED;
	packet.sequence = 42;
	packet.estimate_position = {1, 2, 3};

	TelemetryPublisher publisher("127.0.0.1", 65534);
	for (int i = 0; i < 10000; ++i) {
		(void)publisher.publish(packet);
	}
	if (publisher.socket_is_blocking()) {
		throw std::runtime_error("publisher socket is blocking");
	}
	TelemetryPublisher disabled;
	if (disabled.publish(packet) || disabled.enabled()) {
		throw std::runtime_error("disabled publisher sent data");
	}

	const int receiver = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (receiver < 0) {
		throw std::runtime_error("failed to create telemetry test receiver");
	}
	sockaddr_in address{};
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	address.sin_port = 0;
	if (::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
		::close(receiver);
		throw std::runtime_error("failed to bind telemetry test receiver");
	}
	socklen_t address_size = sizeof(address);
	if (::getsockname(receiver, reinterpret_cast<sockaddr*>(&address), &address_size) < 0) {
		::close(receiver);
		throw std::runtime_error("failed to read telemetry test port");
	}
	timeval timeout{0, 100000};
	(void)::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

	TelemetryPublisher loopback("127.0.0.1", ntohs(address.sin_port));
	if (!loopback.publish(packet)) {
		::close(receiver);
		throw std::runtime_error("loopback telemetry publish failed");
	}
	std::array<std::uint8_t, TelemetryCodec::WIRE_SIZE> bytes{};
	const ssize_t received = ::recv(receiver, bytes.data(), bytes.size(), 0);
	::close(receiver);
	TelemetryPacket decoded;
	if (received != static_cast<ssize_t>(bytes.size()) ||
		!TelemetryCodec::decode(bytes.data(), bytes.size(), decoded) ||
		decoded.sequence != 42 || decoded.estimate_position != Vector3d{1, 2, 3}) {
		throw std::runtime_error("loopback telemetry mismatch");
	}
}
