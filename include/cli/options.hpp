#pragma once

#include <cstdint>
#include <exception>
#include <string>

struct AppOptions {
	double accel_sigma = 1e-2;
	double gps_sigma = 1.0;
	double gate_threshold = 11.345;
	std::string telemetry_host = "127.0.0.1";
	std::uint16_t telemetry_port = 4243;
	bool telemetry_enabled = true;
};

class HelpRequested : public std::exception {
public:
	const char* what() const noexcept override { return "help requested"; }
};

AppOptions parse_options(int argc, char** argv);
std::string usage(const char* program);
