#include "visualizer/view_geometry.hpp"

#include <algorithm>
#include <cmath>

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
