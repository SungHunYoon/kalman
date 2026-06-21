#include "network/udp_client.hpp"
#include "protocol/message_assembler.hpp"
#include "protocol/parser.hpp"
#include "protocol/sensor_state.hpp"
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

namespace {
	std::string format_position(const Vector<double>& position) {
		std::ostringstream oss;
		oss << position[0] << " " << position[1] << " " << position[2] << "\n";
		return oss.str();
	}
}

int main() {
	try {
		std::string host = "127.0.0.1";
		std::uint16_t port = 4242;

		UDPClient client(host, port);
		MessageAssembler assembler;
		SensorState sensor_state;
		Parser parser;

		std::cout << "[send] READY\n";
		client.send_text("READY\n");

		while (1) {
			std::string chunk = client.recv_text();
			std::string message;
			while (assembler.append(chunk, message)) {
				chunk.clear();
				SensorUpdate update = parser.parse(message);
				sensor_state.apply(update);
				if (sensor_state.has_estimated_position()) {
					client.send_text(format_position(sensor_state.estimated_position()));
					std::cout << format_position(sensor_state.estimated_position()) << "\n";
				}
			}
		}
	} catch (const std::exception& e) {
		std::cerr << e.what() << "\n";
		return 1;
	}

	return 0;
}
