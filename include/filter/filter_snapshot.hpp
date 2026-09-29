#pragma once

#include "filter/fixed_math.hpp"

struct FilterSnapshot {
	Vector3d position{};
	Vector3d velocity{};
	Vector3d position_variance{};
	Vector3d gps_variance{};
};

struct GpsUpdateResult {
	Vector3d innovation{};
};
