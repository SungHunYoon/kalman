#pragma once

#include "filter/fixed_math.hpp"

struct FilterSnapshot {
	Vector3d position{};
	Vector3d velocity{};
	Vector3d position_variance{};
	Vector3d gps_variance{};
};

struct GpsUpdateResult {
	bool accepted = false;
	Vector3d innovation{};
	double mahalanobis_squared = 0.0;
};
