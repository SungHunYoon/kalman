#include "visualizer/renderer.hpp"
#include "visualizer/telemetry_receiver.hpp"

#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
	std::uint16_t parse_port(const char* text) {
		errno = 0;
		char* end = nullptr;
		const unsigned long value = std::strtoul(text, &end, 10);
		if (errno != 0 || end == text || *end != '\0' || value == 0 || value > 65535) {
			throw std::invalid_argument("port must be in 1..65535");
		}
		return static_cast<std::uint16_t>(value);
	}
}

int main(int argc, char** argv) {
	try {
		std::uint16_t port = 4243;
		for (int index = 1; index < argc; ++index) {
			const std::string option(argv[index]);
			if (option == "--help") {
				std::cout << "Usage: " << argv[0] << " [--port 1..65535]\n";
				return 0;
			}
			if (option != "--port" || index + 1 >= argc) {
				throw std::invalid_argument("unknown or incomplete viewer option: " + option);
			}
			port = parse_port(argv[++index]);
		}
		TelemetryReceiver receiver(port);
		ViewerModel model;
		Renderer renderer;
		while (!renderer.should_close()) {
			(void)receiver.drain(model);
			renderer.update_controls(model);
			renderer.draw(model);
		}
	} catch (const std::exception& error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
	return 0;
}
