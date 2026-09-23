#include "visualizer/render_math.hpp"

#include <algorithm>
#include <cmath>

double covariance_radius(double variance) noexcept {
	return std::isfinite(variance) && variance > 0.0 ? std::sqrt(variance) : 0.0;
}

double vector_norm(const Vector3d& value) noexcept {
	return std::hypot(value[0], value[1], value[2]);
}

Vector3d smoothed_target(const Vector3d& current, const Vector3d& desired,
	double alpha) noexcept {
	const double amount = std::isfinite(alpha) ? std::clamp(alpha, 0.0, 1.0) : 0.0;
	Vector3d result{};
	for (std::size_t axis = 0; axis < result.size(); ++axis) {
		result[axis] = current[axis] + amount * (desired[axis] - current[axis]);
	}
	return result;
}
