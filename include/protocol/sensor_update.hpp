#pragma once

#include "linear_algebra/vector.hpp"
#include <optional>

struct SensorUpdate {
	double time = 0.0;

	std::optional<Vector<double>> initial_position;
	std::optional<double> initial_speed_kmh;
	std::optional<Vector<double>> acceleration;
	std::optional<Vector<double>> direction;
	std::optional<Vector<double>> gps;
};
