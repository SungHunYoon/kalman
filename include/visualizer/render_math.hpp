#pragma once

#include "filter/fixed_math.hpp"

double covariance_radius(double variance) noexcept;
double vector_norm(const Vector3d& value) noexcept;
Vector3d smoothed_target(const Vector3d& current, const Vector3d& desired,
	double alpha) noexcept;
Vector3d camera_move_delta(double yaw, double forward, double right, double up,
	double elapsed_seconds, double speed) noexcept;
