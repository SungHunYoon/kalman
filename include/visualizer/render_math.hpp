#pragma once

#include "filter/fixed_math.hpp"

double covariance_radius(double variance) noexcept;
Vector3d smoothed_target(const Vector3d& current, const Vector3d& desired,
	double alpha) noexcept;
