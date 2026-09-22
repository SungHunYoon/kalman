#include "cli/options.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace {
	const char* take_value(int argc, char** argv, int& index) {
		if (index + 1 >= argc) {
			throw std::invalid_argument(std::string("missing value for ") + argv[index]);
		}
		return argv[++index];
	}

	double parse_positive_double(const char* option, const char* text) {
		errno = 0;
		char* end = nullptr;
		const double value = std::strtod(text, &end);
		if (errno == ERANGE || end == text || *end != '\0' || !std::isfinite(value) || value <= 0.0) {
			throw std::invalid_argument(std::string("invalid value for ") + option + ": " + text);
		}
		return value;
	}

	std::uint16_t parse_port(const char* text) {
		errno = 0;
		char* end = nullptr;
		const unsigned long value = std::strtoul(text, &end, 10);
		if (errno == ERANGE || end == text || *end != '\0' || value == 0 || value > 65535) {
			throw std::invalid_argument(std::string("invalid telemetry port: ") + text);
		}
		return static_cast<std::uint16_t>(value);
	}
}

AppOptions parse_options(int argc, char** argv) {
	AppOptions options;
	for (int index = 1; index < argc; ++index) {
		const std::string option(argv[index]);
		if (option == "--help") {
			throw HelpRequested();
		}
		if (option == "--accel-sigma") {
			options.accel_sigma = parse_positive_double(argv[index], take_value(argc, argv, index));
		} else if (option == "--gps-sigma") {
			options.gps_sigma = parse_positive_double(argv[index], take_value(argc, argv, index));
		} else if (option == "--gate-threshold") {
			options.gate_threshold = parse_positive_double(argv[index], take_value(argc, argv, index));
		} else if (option == "--telemetry-host") {
			options.telemetry_host = take_value(argc, argv, index);
			if (options.telemetry_host.empty()) {
				throw std::invalid_argument("telemetry host must not be empty");
			}
		} else if (option == "--telemetry-port") {
			options.telemetry_port = parse_port(take_value(argc, argv, index));
		} else if (option == "--no-telemetry") {
			options.telemetry_enabled = false;
		} else {
			throw std::invalid_argument("unknown option: " + option);
		}
	}
	return options;
}

std::string usage(const char* program) {
	return std::string("Usage: ") + program + " [options]\n"
		"  --accel-sigma <positive number>\n"
		"  --gps-sigma <positive number>\n"
		"  --gate-threshold <positive number>\n"
		"  --telemetry-host <IPv4 address>\n"
		"  --telemetry-port <1..65535>\n"
		"  --no-telemetry\n"
		"  --help\n";
}
