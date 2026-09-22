#include "visualizer/telemetry_receiver.hpp"
#include "visualizer/view_model.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace {
	void send_datagram(int socket, std::uint16_t port, const std::uint8_t* bytes,
		std::size_t size) {
		sockaddr_in address{};
		address.sin_family = AF_INET;
		address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		address.sin_port = htons(port);
		if (::sendto(socket, bytes, size, 0,
			reinterpret_cast<const sockaddr*>(&address), sizeof(address)) !=
			static_cast<ssize_t>(size)) {
			throw std::runtime_error("failed to send visualizer test datagram");
		}
	}
}

void run_visualizer_model_tests() {
	ViewerModel model;
	TelemetryPacket first;
	first.sequence = 10;
	TelemetryPacket second;
	second.sequence = 13;
	const auto now = std::chrono::steady_clock::time_point(std::chrono::seconds(10));
	model.accept(first, now);
	model.accept(second, now);
	if (model.snapshot(now).lost_packets != 2 || !model.snapshot(now).connected) {
		throw std::runtime_error("sequence gap or connection mismatch");
	}
	if (model.snapshot(now + std::chrono::seconds(3)).connected) {
		throw std::runtime_error("stale model remained connected");
	}
	TelemetryPacket reordered;
	reordered.sequence = 12;
	model.accept(reordered, now);
	if (model.snapshot(now).lost_packets != 2 || model.snapshot(now).latest.sequence != 13) {
		throw std::runtime_error("reordered packet changed latest state");
	}

	ViewerModel wrapped;
	TelemetryPacket near_wrap;
	near_wrap.sequence = std::numeric_limits<std::uint64_t>::max();
	TelemetryPacket after_wrap;
	after_wrap.sequence = 0;
	wrapped.accept(near_wrap, now);
	wrapped.accept(after_wrap, now);
	if (wrapped.snapshot(now).lost_packets != 0 || wrapped.snapshot(now).latest.sequence != 0) {
		throw std::runtime_error("sequence wraparound mismatch");
	}

	ViewerModel paused_model;
	paused_model.set_history_paused(true);
	TelemetryPacket paused;
	paused.sequence = 20;
	paused_model.accept(paused, now);
	if (!paused_model.history_paused() || paused_model.snapshot(now).latest.sequence != 20 ||
		paused_model.trajectory().estimate_count() != 0) {
		throw std::runtime_error("paused model stopped state updates or changed history");
	}

	TelemetryReceiver receiver(0);
	const int sender = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (sender < 0) throw std::runtime_error("failed to create receiver test sender");
	const std::array<std::uint8_t, 3> short_packet{1, 2, 3};
	send_datagram(sender, receiver.bound_port(), short_packet.data(), short_packet.size());
	TelemetryPacket valid;
	valid.flags = TELEMETRY_INITIALIZED;
	valid.sequence = 7;
	auto invalid = TelemetryCodec::encode(valid);
	invalid[0] = 0;
	send_datagram(sender, receiver.bound_port(), invalid.data(), invalid.size());
	const auto encoded = TelemetryCodec::encode(valid);
	send_datagram(sender, receiver.bound_port(), encoded.data(), encoded.size());
	::close(sender);
	ViewerModel received;
	if (receiver.drain(received) != 1 || received.snapshot(std::chrono::steady_clock::now()).malformed_packets != 1 ||
		received.snapshot(std::chrono::steady_clock::now()).invalid_packets != 1 ||
		received.snapshot(std::chrono::steady_clock::now()).latest.sequence != 7) {
		throw std::runtime_error("receiver classification mismatch");
	}
}
