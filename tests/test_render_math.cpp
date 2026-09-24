#include "visualizer/render_math.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

void run_render_math_tests() {
	if (covariance_radius(4.0) != 2.0 || covariance_radius(0.0) != 0.0) {
		throw std::runtime_error("covariance radius mismatch");
	}
	for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::quiet_NaN()}) {
		if (covariance_radius(invalid) != 0.0) {
			throw std::runtime_error("invalid covariance produced a radius");
		}
	}
	if (smoothed_target({0, 0, 0}, {10, 20, 30}, 0.1) != Vector3d{1, 2, 3} ||
		smoothed_target({1, 2, 3}, {10, 20, 30}, -1.0) != Vector3d{1, 2, 3} ||
		smoothed_target({1, 2, 3}, {10, 20, 30}, 2.0) != Vector3d{10, 20, 30}) {
		throw std::runtime_error("camera smoothing mismatch");
	}
	if (vector_norm({3.0, 4.0, 0.0}) != 5.0 || vector_norm({0.0, 0.0, 0.0}) != 0.0) {
		throw std::runtime_error("innovation norm mismatch");
	}
	if (camera_move_delta(0.0, 1.0, 0.0, 0.0, 0.5, 4.0) != Vector3d{-2.0, 0.0, 0.0}) {
		throw std::runtime_error("forward camera movement mismatch");
	}
	if (camera_move_delta(0.0, 0.0, 1.0, 0.0, 0.5, 4.0) != Vector3d{0.0, -2.0, 0.0}) {
		throw std::runtime_error("right camera movement mismatch");
	}
	if (camera_move_delta(0.0, 0.0, 0.0, 1.0, 0.5, 4.0) != Vector3d{0.0, 0.0, 2.0}) {
		throw std::runtime_error("up camera movement mismatch");
	}
	const Vector3d rotated = camera_move_delta(1.5707963267948966, 1.0, 0.0, 0.0, 1.0, 2.0);
	if (std::abs(rotated[0]) > 1e-9 || std::abs(rotated[1] + 2.0) > 1e-9 ||
		std::abs(rotated[2]) > 1e-9) {
		throw std::runtime_error("rotated camera movement mismatch");
	}
	const Vector3d diagonal = camera_move_delta(0.0, 1.0, 1.0, 0.0, 1.0, 2.0);
	if (std::abs(vector_norm(diagonal) - 2.0) > 1e-9) {
		throw std::runtime_error("diagonal camera movement exceeds base speed");
	}
	if (camera_follow_enabled(true, false, true) ||
		!camera_follow_enabled(false, true, false) ||
		camera_follow_enabled(true, true, false) ||
		camera_follow_enabled(false, false, false)) {
		throw std::runtime_error("manual camera movement and follow toggle mismatch");
	}
}
