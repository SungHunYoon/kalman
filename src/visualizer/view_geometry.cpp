#include "visualizer/view_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

Bounds3d padded_axis_bounds(const Bounds3d& raw) noexcept {
	Bounds3d result = raw;
	double longest = 1.0;
	for (std::size_t axis = 0; axis < 3; ++axis) {
		longest = std::max(longest, raw.max[axis] - raw.min[axis]);
	}
	for (std::size_t axis = 0; axis < 3; ++axis) {
		const double padding = std::max(0.05 * (raw.max[axis] - raw.min[axis]), 0.02 * longest);
		result.min[axis] -= padding;
		result.max[axis] += padding;
	}
	return result;
}

double nice_tick_step(double span) noexcept {
	if (!std::isfinite(span) || span <= 0.0) return 1.0;
	const double raw = span / 5.0;
	if (raw == 0.0) return 1.0;
	const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
	if (magnitude == 0.0) return raw;
	double best = magnitude;
	for (double multiplier : {2.0, 5.0, 10.0}) {
		const double candidate = multiplier * magnitude;
		if (std::abs(candidate - raw) < std::abs(best - raw)) best = candidate;
	}
	return best;
}

std::vector<double> axis_ticks(double minimum, double maximum, double step) {
	std::vector<double> ticks;
	if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
		!std::isfinite(step) || step <= 0.0 || minimum > maximum) return ticks;
	const double first = std::ceil(minimum / step);
	for (int index = 0; index < 12; ++index) {
		const double value = (first + index) * step;
		if (!std::isfinite(value) || value > maximum ||
			(!ticks.empty() && value <= ticks.back())) break;
		if (value >= minimum) ticks.push_back(value == 0.0 ? 0.0 : value);
	}
	return ticks;
}

std::string format_axis_tick(double value, double step) {
	if (!std::isfinite(value) || !std::isfinite(step) || step <= 0.0) return {};
	const int precision = std::max(0, static_cast<int>(-std::floor(std::log10(step))));
	std::ostringstream output;
	output.imbue(std::locale::classic());
	output << std::fixed << std::setprecision(precision) << value;
	std::string label = output.str();
	if (label.find('.') != std::string::npos) {
		while (label.back() == '0') label.pop_back();
		if (label.back() == '.') label.pop_back();
	}
	return label == "-0" ? "0" : label;
}

ViewMode next_view_mode(ViewMode current, bool whole_view_pressed,
	bool follow_pressed, bool manual_input) noexcept {
	if (whole_view_pressed) return ViewMode::Overview;
	if (manual_input) return ViewMode::Manual;
	if (follow_pressed) return current == ViewMode::Follow ? ViewMode::Manual : ViewMode::Follow;
	return current;
}

double zoom_vertical_size(double current_size, double wheel_steps) noexcept {
	constexpr double MIN_SIZE = 1e-6;
	constexpr double MAX_SIZE = 1e12;
	const double size = std::isfinite(current_size) ?
		std::clamp(current_size, MIN_SIZE, MAX_SIZE) : 1.0;
	if (!std::isfinite(wheel_steps)) return size;
	const double exponent = std::clamp(wheel_steps, -1000.0, 1000.0);
	return std::clamp(size * std::pow(0.85, exponent), MIN_SIZE, MAX_SIZE);
}

Vector3d screen_pan_delta(double yaw, double pitch, double dx_pixels,
	double dy_pixels, double vertical_size, int viewport_height) noexcept {
	const double scale = vertical_size / std::max(1, viewport_height);
	const double sin_yaw = std::sin(yaw);
	const double cos_yaw = std::cos(yaw);
	const double sin_pitch = std::sin(pitch);
	const double cos_pitch = std::cos(pitch);
	return {scale * (-dx_pixels * sin_yaw - dy_pixels * sin_pitch * cos_yaw),
		scale * (dx_pixels * cos_yaw - dy_pixels * sin_pitch * sin_yaw),
		scale * dy_pixels * cos_pitch};
}

double camera_move_speed(double vertical_size) noexcept {
	return std::max(1.0, vertical_size * 0.5);
}

ViewOrientation preset_view_orientation(int preset) noexcept {
	constexpr double HALF_PI = 1.57079632679489661923;
	if (preset == 1) return {0.0, HALF_PI};
	if (preset == 2) return {HALF_PI, 0.0};
	return {0.0, 0.0};
}

Vector3d camera_up_direction(double yaw, double pitch) noexcept {
	return {-std::sin(pitch) * std::cos(yaw),
		-std::sin(pitch) * std::sin(yaw), std::cos(pitch)};
}

bool overview_refit_for_viewport_change(ViewMode mode, bool dimensions_changed) noexcept {
	return dimensions_changed && mode == ViewMode::Overview;
}

double overview_size_containing_bounds(const Bounds3d& bounds, const Vector3d& target,
	double yaw, double pitch, int viewport_width, int viewport_height,
	double requested_size) noexcept {
	const double sin_yaw = std::sin(yaw);
	const double cos_yaw = std::cos(yaw);
	const Vector3d right{sin_yaw, -cos_yaw, 0.0};
	const Vector3d up = camera_up_direction(yaw, pitch);
	double horizontal_extent = 0.0;
	double vertical_extent = 0.0;
	for (int x = 0; x < 2; ++x) {
		for (int y = 0; y < 2; ++y) {
			for (int z = 0; z < 2; ++z) {
				const Vector3d offset{
					(x ? bounds.max[0] : bounds.min[0]) - target[0],
					(y ? bounds.max[1] : bounds.min[1]) - target[1],
					(z ? bounds.max[2] : bounds.min[2]) - target[2]};
				const double horizontal = offset[0] * right[0] +
					offset[1] * right[1] + offset[2] * right[2];
				const double vertical = offset[0] * up[0] +
					offset[1] * up[1] + offset[2] * up[2];
				horizontal_extent = std::max(horizontal_extent, std::abs(horizontal));
				vertical_extent = std::max(vertical_extent, std::abs(vertical));
			}
		}
	}
	const double aspect = static_cast<double>(std::max(1, viewport_width)) /
		std::max(1, viewport_height);
	const double required = std::max({1.0, 2.5 * vertical_extent,
		2.5 * horizontal_extent / aspect});
	return std::max(requested_size, required);
}

bool screen_rects_overlap(const ScreenRect& first, const ScreenRect& second) noexcept {
	return first.x < second.x + second.width && first.x + first.width > second.x &&
		first.y < second.y + second.height && first.y + first.height > second.y;
}

std::optional<ScreenRect> place_screen_label(const ScreenRect& preferred,
	int viewport_width, int viewport_height,
	const std::vector<ScreenRect>& occupied) noexcept {
	constexpr double SPACING = 3.0;
	const std::array<std::pair<double, double>, 8> directions{{
		{0.0, -1.0}, {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0},
		{1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};
	const double max_x = std::max(0, viewport_width) - preferred.width;
	const double max_y = std::max(0, viewport_height) - preferred.height;
	for (int ring = 0; ring <= 16; ++ring) {
		const double distance = ring * (std::max(preferred.width, preferred.height) + SPACING);
		for (const auto& direction : directions) {
			const ScreenRect candidate{preferred.x + direction.first * distance,
				preferred.y + direction.second * distance, preferred.width, preferred.height};
			if (candidate.x < 0.0 || candidate.y < 0.0 ||
				candidate.x > max_x || candidate.y > max_y) continue;
			bool overlaps = false;
			for (const ScreenRect& previous : occupied) {
				overlaps = overlaps || screen_rects_overlap(candidate, previous);
			}
			if (!overlaps) return candidate;
		}
	}
	return std::nullopt;
}

CameraFit fit_estimate_bounds(const Bounds3d& bounds, double yaw, double pitch,
	int viewport_width, int viewport_height) noexcept {
	Vector3d target{};
	for (std::size_t axis = 0; axis < target.size(); ++axis) {
		target[axis] = bounds.min[axis] + 0.5 * (bounds.max[axis] - bounds.min[axis]);
	}

	const double sin_yaw = std::sin(yaw);
	const double cos_yaw = std::cos(yaw);
	const double sin_pitch = std::sin(pitch);
	const double cos_pitch = std::cos(pitch);
	const Vector3d right{sin_yaw, -cos_yaw, 0.0};
	const Vector3d up = camera_up_direction(yaw, pitch);
	const Vector3d outward{cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch};
	double horizontal_extent = 0.0;
	double vertical_extent = 0.0;
	double depth_extent = 0.0;
	for (int x = 0; x < 2; ++x) {
		for (int y = 0; y < 2; ++y) {
			for (int z = 0; z < 2; ++z) {
				const Vector3d offset{
					(x ? bounds.max[0] : bounds.min[0]) - target[0],
					(y ? bounds.max[1] : bounds.min[1]) - target[1],
					(z ? bounds.max[2] : bounds.min[2]) - target[2]};
				const auto projection = [&offset](const Vector3d& direction) {
					return offset[0] * direction[0] + offset[1] * direction[1] +
						offset[2] * direction[2];
				};
				horizontal_extent = std::max(horizontal_extent, std::abs(projection(right)));
				vertical_extent = std::max(vertical_extent, std::abs(projection(up)));
				depth_extent = std::max(depth_extent, std::abs(projection(outward)));
			}
		}
	}

	const double aspect = static_cast<double>(std::max(1, viewport_width)) /
		std::max(1, viewport_height);
	const double vertical_size = std::max({1.0, 2.5 * vertical_extent,
		2.5 * horizontal_extent / aspect});
	const double camera_distance = std::max(10.0, 2.0 * depth_extent + vertical_size);
	const double far_clip = camera_distance + depth_extent + vertical_size;
	return {target, vertical_size, camera_distance, far_clip};
}
