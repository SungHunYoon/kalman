#include "visualizer/view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double TOLERANCE = 1e-8;

void require_finite_positive(const CameraFit& fit) {
	if (!std::isfinite(fit.vertical_size) || fit.vertical_size <= 0.0 ||
		!std::isfinite(fit.camera_distance) || fit.camera_distance <= 0.0 ||
		!std::isfinite(fit.far_clip) || fit.far_clip <= fit.camera_distance) {
		throw std::runtime_error("camera fit dimensions are not finite and positive");
	}
	for (double coordinate : fit.target) {
		if (!std::isfinite(coordinate)) {
			throw std::runtime_error("camera fit target is not finite");
		}
	}
}

void require_corners_visible(const Bounds3d& bounds, double yaw, double pitch,
	int viewport_width, int viewport_height, const CameraFit& fit) {
	const Vector3d right{std::sin(yaw), -std::cos(yaw), 0.0};
	const Vector3d up{-std::sin(pitch) * std::cos(yaw),
		-std::sin(pitch) * std::sin(yaw), std::cos(pitch)};
	const double aspect = static_cast<double>(std::max(1, viewport_width)) /
		std::max(1, viewport_height);
	for (int x = 0; x < 2; ++x) {
		for (int y = 0; y < 2; ++y) {
			for (int z = 0; z < 2; ++z) {
				const Vector3d offset{
					(x ? bounds.max[0] : bounds.min[0]) - fit.target[0],
					(y ? bounds.max[1] : bounds.min[1]) - fit.target[1],
					(z ? bounds.max[2] : bounds.min[2]) - fit.target[2]};
				const double horizontal = offset[0] * right[0] +
					offset[1] * right[1] + offset[2] * right[2];
				const double vertical = offset[0] * up[0] +
					offset[1] * up[1] + offset[2] * up[2];
				if (std::abs(horizontal) > 0.4 * fit.vertical_size * aspect + TOLERANCE ||
					std::abs(vertical) > 0.4 * fit.vertical_size + TOLERANCE) {
					throw std::runtime_error("trajectory corner outside 80% of orthographic view");
				}
			}
		}
	}
}

void require_fitted(const Bounds3d& bounds, double yaw, double pitch,
	int width, int height) {
	const CameraFit fit = fit_estimate_bounds(bounds, yaw, pitch, width, height);
	require_finite_positive(fit);
	require_corners_visible(bounds, yaw, pitch, width, height, fit);
}
}

void run_view_geometry_tests() {
	const Bounds3d point{{2.0, -3.0, 5.0}, {2.0, -3.0, 5.0}};
	const CameraFit point_fit = fit_estimate_bounds(point, 0.8, 0.5, 1200, 800);
	if (point_fit.target != point.min) {
		throw std::runtime_error("point route camera target mismatch");
	}
	require_fitted(point, 0.8, 0.5, 1200, 800);

	const Bounds3d far_route{{-1000.0, -10.0, -20.0}, {4000.0, 20.0, 30.0}};
	const CameraFit far_fit = fit_estimate_bounds(far_route, 0.0, 0.0, 1200, 800);
	if (far_fit.target != Vector3d{1500.0, 5.0, 5.0} ||
		far_fit.camera_distance <= 2500.0 ||
		far_fit.far_clip <= far_fit.camera_distance + std::hypot(2500.0, 15.0, 25.0)) {
		throw std::runtime_error("large trajectory camera fit mismatch");
	}
	require_fitted(far_route, 0.0, 0.0, 1200, 800);

	const Bounds3d flat_negative{{-12.0, -40.0, -7.0}, {-2.0, -10.0, -7.0}};
	if (fit_estimate_bounds(flat_negative, 0.7, 0.4, 1200, 800).target !=
		Vector3d{-7.0, -25.0, -7.0}) {
		throw std::runtime_error("negative flat route camera target mismatch");
	}
	require_fitted(flat_negative, 0.7, 0.4, 1200, 800);

	const Bounds3d wide{{-200.0, -2000.0, -5.0}, {200.0, 2000.0, -5.0}};
	const CameraFit landscape = fit_estimate_bounds(wide, 0.0, 0.0, 1200, 800);
	const CameraFit portrait = fit_estimate_bounds(wide, 0.0, 0.0, 600, 1000);
	if (portrait.vertical_size + TOLERANCE < landscape.vertical_size) {
		throw std::runtime_error("portrait fit clipped a wide flat route");
	}
	require_fitted(wide, 0.0, 0.0, 1200, 800);
	require_fitted(wide, 0.0, 0.0, 600, 1000);
	require_fitted(flat_negative, 0.7, 0.4, 0, -10);
}
