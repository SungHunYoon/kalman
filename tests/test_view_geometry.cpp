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

void require_close(double actual, double expected, const char* message) {
	if (!std::isfinite(actual) || std::abs(actual - expected) > TOLERANCE) {
		throw std::runtime_error(message);
	}
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
	const ViewOrientation top_orientation = preset_view_orientation(1);
	const Vector3d top_up = camera_up_direction(top_orientation.yaw, top_orientation.pitch);
	const Vector3d top_forward{std::cos(top_orientation.pitch) * std::cos(top_orientation.yaw),
		std::cos(top_orientation.pitch) * std::sin(top_orientation.yaw),
		std::sin(top_orientation.pitch)};
	require_close(top_orientation.pitch, std::acos(-1.0) / 2.0,
		"top preset does not look down the telemetry Z axis");
	require_close(top_up[0] * top_forward[0] + top_up[1] * top_forward[1] +
		top_up[2] * top_forward[2], 0.0,
		"top preset camera up is parallel to its viewing direction");
	const Bounds3d top_route_a{{-8.0, -4.0, -2.0}, {8.0, 4.0, 2.0}};
	const Bounds3d top_route_b{{-8.0, -4.0, -2000.0}, {8.0, 4.0, 2000.0}};
	require_close(fit_estimate_bounds(top_route_a, top_orientation.yaw,
		top_orientation.pitch, 1200, 800).vertical_size,
		fit_estimate_bounds(top_route_b, top_orientation.yaw,
			top_orientation.pitch, 1200, 800).vertical_size,
		"top view scale changed when only vertical extent changed");
	if (!overview_refit_for_viewport_change(ViewMode::Overview, true) ||
		overview_refit_for_viewport_change(ViewMode::Overview, false) ||
		overview_refit_for_viewport_change(ViewMode::Manual, true) ||
		overview_refit_for_viewport_change(ViewMode::Follow, true)) {
		throw std::runtime_error("viewport change camera mode behavior mismatch");
	}
	const Bounds3d expanded{{0.0, 0.0, 0.0}, {10000.0, 10000.0, 1000.0}};
	const CameraFit expansion_fit = fit_estimate_bounds(expanded, 0.7, 0.4, 1200, 800);
	Vector3d eased_target{};
	double eased_size = 1.0;
	for (std::size_t axis = 0; axis < eased_target.size(); ++axis) {
		eased_target[axis] += 0.12 * (expansion_fit.target[axis] - eased_target[axis]);
	}
	eased_size += 0.12 * (expansion_fit.vertical_size - eased_size);
	const double containing_size = overview_size_containing_bounds(expanded, eased_target,
		0.7, 0.4, 1200, 800, eased_size);
	if (!(containing_size > eased_size)) {
		throw std::runtime_error("overview expansion did not preserve trajectory containment");
	}
	require_corners_visible(expanded, 0.7, 0.4, 1200, 800,
		CameraFit{eased_target, containing_size, expansion_fit.camera_distance,
			expansion_fit.far_clip});
	const ScreenRect preferred_label{100.0, 100.0, 64.0, 24.0};
	const std::vector<ScreenRect> occupied_labels{preferred_label};
	const auto placed_label = place_screen_label(preferred_label, 400, 300, occupied_labels);
	if (!placed_label || screen_rects_overlap(*placed_label, preferred_label) ||
		placed_label->x < 0.0 || placed_label->y < 0.0 ||
		placed_label->x + placed_label->width > 400.0 ||
		placed_label->y + placed_label->height > 300.0) {
		throw std::runtime_error("colliding label did not receive a visible alternate position");
	}
	const double zoom_in = zoom_vertical_size(1000.0, 1.0);
	const double zoom_out = zoom_vertical_size(1000.0, -1.0);
	const Vector3d zoom_target{120.0, -35.0, 18.0};
	for (const double pitch : {0.0, 0.55, -0.8}) {
		const double yaw = 0.7;
		const Vector3d right{std::sin(yaw), -std::cos(yaw), 0.0};
		const Vector3d up = camera_up_direction(yaw, pitch);
		for (const double new_size : {zoom_in, zoom_out}) {
			const double cursor_x = 930.0;
			const double cursor_y = 170.0;
			Vector3d anchor = zoom_target;
			for (std::size_t axis = 0; axis < 3; ++axis) {
				anchor[axis] += (right[axis] * (cursor_x - 600.0) +
					up[axis] * (400.0 - cursor_y)) * 1000.0 / 800.0;
			}
			const Vector3d shifted = cursor_zoom_target(zoom_target, yaw, pitch,
				cursor_x, cursor_y, 1200, 800, 1000.0, new_size);
			double projected_x = 600.0;
			double projected_y = 400.0;
			for (std::size_t axis = 0; axis < 3; ++axis) {
				projected_x += (anchor[axis] - shifted[axis]) * right[axis] *
					800.0 / new_size;
				projected_y -= (anchor[axis] - shifted[axis]) * up[axis] *
					800.0 / new_size;
			}
			require_close(projected_x, cursor_x, "zoom moved cursor anchor horizontally");
			require_close(projected_y, cursor_y, "zoom moved cursor anchor vertically");
		}
	}
	if (cursor_zoom_target(zoom_target, 0.7, 0.55, 600.0, 400.0,
		1200, 800, 1000.0, zoom_in) != zoom_target) {
		throw std::runtime_error("centered zoom moved camera target");
	}
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
