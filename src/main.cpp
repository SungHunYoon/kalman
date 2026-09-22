#include "cli/options.hpp"
#include "network/udp_client.hpp"
#include "protocol/message_assembler.hpp"
#include "protocol/parser.hpp"
#include "protocol/sensor_state.hpp"
#include "protocol/stream_control.hpp"
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
	std::string format_position(const Vector<double>& position) {
		for (std::size_t axis = 0; axis < 3; ++axis) {
			if (!std::isfinite(position[axis])) {
				throw std::runtime_error("[format_position] non-finite estimate");
			}
		}
		std::ostringstream oss;
		oss << std::setprecision(std::numeric_limits<double>::max_digits10);
		oss << position[0] << " " << position[1] << " " << position[2] << "\n";
		return oss.str();
	}
}

int main(int argc, char** argv) {
	try {
		const AppOptions options = parse_options(argc, argv);
		std::string host = "127.0.0.1";
		std::uint16_t port = 4242;

		UDPClient client(host, port);
		MessageAssembler assembler;
		SensorState sensor_state({options.accel_sigma, options.gps_sigma,
			options.gate_threshold});
		Parser parser;

		std::cout << "[send] READY\n";
		client.send_text("READY\n");

		while (1) {
			std::string chunk = client.recv_text(sensor_stream_receive_timeout_ms());
			if (is_sensor_stream_goodbye(chunk)) {
				std::cout << "[recv] GOODBYE.\n";
				break;
			}
			const std::vector<std::string> messages = assembler.append(chunk);
			for (const std::string& message : messages) {
				const SensorUpdate update = parser.parse(message);
				sensor_state.apply(update);
				if (sensor_state.has_estimated_position()) {
					client.send_text(format_position(sensor_state.estimated_position()));
					std::cout << format_position(sensor_state.estimated_position()) << "\n";
				}
			}
		}
	} catch (const HelpRequested&) {
		std::cout << usage(argv[0]);
		return 0;
	} catch (const std::exception& e) {
		std::cerr << e.what() << "\n";
		return 1;
	}

	return 0;
}
