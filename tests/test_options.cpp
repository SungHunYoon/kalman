#include "cli/options.hpp"

#include <stdexcept>

void run_option_tests() {
	{
		const char* argv[] = {"kalman"};
		const AppOptions options = parse_options(1, const_cast<char**>(argv));
		if (options.telemetry_host != "127.0.0.1" ||
			options.telemetry_port != 4243 || !options.telemetry_enabled) {
			throw std::runtime_error("default options mismatch");
		}
	}
	{
		const char* argv[] = {"kalman", "--telemetry-host", "127.0.0.2",
			"--telemetry-port", "5000", "--no-telemetry"};
		const AppOptions options = parse_options(6, const_cast<char**>(argv));
		if (options.telemetry_host != "127.0.0.2" ||
			options.telemetry_port != 5000 || options.telemetry_enabled) {
			throw std::runtime_error("explicit options mismatch");
		}
	}
	for (const char* obsolete : {"--accel-sigma", "--gps-sigma", "--gate-threshold"}) {
		const char* argv[] = {"kalman", obsolete, "1"};
		bool threw = false;
		try {
			(void)parse_options(3, const_cast<char**>(argv));
		} catch (const std::invalid_argument&) {
			threw = true;
		}
		if (!threw) {
			throw std::runtime_error("obsolete accuracy option accepted");
		}
	}
	for (const char* bad : {"0", "65536", "12x"}) {
		const char* argv[] = {"kalman", "--telemetry-port", bad};
		bool threw = false;
		try {
			(void)parse_options(3, const_cast<char**>(argv));
		} catch (const std::invalid_argument&) {
			threw = true;
		}
		if (!threw) {
			throw std::runtime_error("invalid telemetry port accepted");
		}
	}
}
