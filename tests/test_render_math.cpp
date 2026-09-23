#include "visualizer/render_math.hpp"

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
}
