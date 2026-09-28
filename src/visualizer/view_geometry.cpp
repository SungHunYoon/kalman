#include "visualizer/view_geometry.hpp"

#include <algorithm>
#include <cmath>

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
	const Vector3d up{-sin_pitch * cos_yaw, -sin_pitch * sin_yaw, cos_pitch};
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
