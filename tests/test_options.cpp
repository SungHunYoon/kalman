#include "cli/options.hpp"

#include <stdexcept>

void run_option_tests() {
	{
		const char* argv[] = {"kalman"};
		const AppOptions options = parse_options(1, const_cast<char**>(argv));
		if (options.accel_sigma != 1e-2 || options.gps_sigma != 1.0 ||
			options.gate_threshold != 11.345 || options.telemetry_host != "127.0.0.1" ||
			options.telemetry_port != 4243 || !options.telemetry_enabled) {
			throw std::runtime_error("default options mismatch");
		}
	}
	{
		const char* argv[] = {"kalman", "--accel-sigma", "0.1", "--gps-sigma", "10",
			"--gate-threshold", "15", "--telemetry-host", "127.0.0.2",
			"--telemetry-port", "5000", "--no-telemetry"};
		const AppOptions options = parse_options(12, const_cast<char**>(argv));
		if (options.accel_sigma != 0.1 || options.gps_sigma != 10.0 ||
			options.gate_threshold != 15.0 || options.telemetry_host != "127.0.0.2" ||
			options.telemetry_port != 5000 || options.telemetry_enabled) {
			throw std::runtime_error("explicit options mismatch");
		}
	}
	for (const char* bad : {"0", "-1", "nan", "inf", "text"}) {
		const char* argv[] = {"kalman", "--gps-sigma", bad};
		bool threw = false;
		try {
			(void)parse_options(3, const_cast<char**>(argv));
		} catch (const std::invalid_argument&) {
			threw = true;
		}
		if (!threw) {
			throw std::runtime_error("invalid sigma accepted");
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
