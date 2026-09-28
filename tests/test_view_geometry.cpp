#include "visualizer/view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
	if (nice_tick_step(4000.0) != 1000.0 ||
		axis_ticks(-120.0, 140.0, 100.0) != std::vector<double>{-100.0, 0.0, 100.0} ||
		format_axis_tick(-100.0, 100.0) != "-100" ||
		format_axis_tick(0.25, 0.05) != "0.25" ||
		format_axis_tick(1.0, 0.01) != "1" ||
		format_axis_tick(-0.0, 0.01) != "0") {
		throw std::runtime_error("axis tick formatting mismatch");
	}
	for (double span : {0.0, -10.0, std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::denorm_min()}) {
		if (!std::isfinite(nice_tick_step(span)) || nice_tick_step(span) <= 0.0) {
			throw std::runtime_error("invalid span produced an invalid tick step");
		}
	}
	if (!axis_ticks(0.0, 1.0, 0.0).empty() ||
		!axis_ticks(1.0, 0.0, 1.0).empty() ||
		!axis_ticks(0.0, std::numeric_limits<double>::infinity(), 1.0).empty() ||
		!axis_ticks(0.0, 1.0, std::numeric_limits<double>::quiet_NaN()).empty()) {
		throw std::runtime_error("invalid axis interval produced ticks");
	}
	const auto huge_ticks = axis_ticks(-1e200, 1e200, 1e199);
	const auto tiny_ticks = axis_ticks(-1e200, 1e200, 1e-200);
	if (huge_ticks.empty() || huge_ticks.size() > 12 || tiny_ticks.size() > 12) {
		throw std::runtime_error("huge axis range exceeded bounded tick count");
	}
	for (std::size_t i = 0; i < huge_ticks.size(); ++i) {
		if (!std::isfinite(huge_ticks[i]) || (i && huge_ticks[i] <= huge_ticks[i - 1])) {
			throw std::runtime_error("huge axis ticks are not finite and increasing");
		}
	}
	const Bounds3d flat{{0.0, 0.0, 0.0}, {4000.0, 0.0, 0.0}};
	const Bounds3d frame = padded_axis_bounds(flat);
	if (frame.min != Vector3d{-200.0, -80.0, -80.0} ||
		frame.max != Vector3d{4200.0, 80.0, 80.0}) {
		throw std::runtime_error("flat route has incorrect padded axis frame");
	}
	require_fitted(frame, 0.8, 0.5, 1200, 800);
	require_fitted(frame, 0.8, 0.5, 600, 1000);
	const Bounds3d single_frame = padded_axis_bounds(Bounds3d{{0, 0, 0}, {0, 0, 0}});
	if (single_frame.min != Vector3d{-0.02, -0.02, -0.02} ||
		single_frame.max != Vector3d{0.02, 0.02, 0.02}) {
		throw std::runtime_error("single point has collapsed padded axis frame");
	}
	if (next_view_mode(ViewMode::Overview, false, false, true) != ViewMode::Manual ||
		next_view_mode(ViewMode::Follow, false, false, true) != ViewMode::Manual ||
		next_view_mode(ViewMode::Manual, false, false, false) != ViewMode::Manual ||
		next_view_mode(ViewMode::Manual, true, true, true) != ViewMode::Overview ||
		next_view_mode(ViewMode::Overview, false, true, false) != ViewMode::Follow ||
		next_view_mode(ViewMode::Follow, false, true, false) != ViewMode::Manual) {
		throw std::runtime_error("camera mode transition mismatch");
	}
	const double zoom_in = zoom_vertical_size(1000.0, 1.0);
	const double zoom_out = zoom_vertical_size(1000.0, -1.0);
	const Vector3d pan = screen_pan_delta(0.0, 0.0, 100.0, 0.0, 1000.0, 1000);
	if (!(zoom_in < 1000.0) || !(zoom_out > 1000.0) ||
		!std::isfinite(zoom_in) || !std::isfinite(zoom_out) ||
		pan[0] != 0.0 || pan[1] <= 0.0 || pan[2] != 0.0 ||
		!(camera_move_speed(4000.0) > camera_move_speed(40.0))) {
		throw std::runtime_error("camera pan zoom or speed mismatch");
	}
	const ViewMode after_zoom = next_view_mode(ViewMode::Overview, false, false, true);
	if (next_view_mode(after_zoom, false, false, false) != ViewMode::Manual) {
		throw std::runtime_error("stream update reset manual zoom mode");
	}
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
