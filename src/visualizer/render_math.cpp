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

Vector3d camera_move_delta(double yaw, double forward, double right, double up,
	double elapsed_seconds, double speed) noexcept {
	const double distance = elapsed_seconds * speed /
		std::max(1.0, std::hypot(forward, right, up));
	return {distance * (-forward * std::cos(yaw) + right * std::sin(yaw)),
		distance * (-forward * std::sin(yaw) - right * std::cos(yaw)),
		distance * up};
}
