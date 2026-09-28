#pragma once

#include "visualizer/trajectory_buffer.hpp"

struct CameraFit {
	Vector3d target;
	double vertical_size;
	double camera_distance;
	double far_clip;
};

CameraFit fit_estimate_bounds(const Bounds3d& bounds, double yaw, double pitch,
	int viewport_width, int viewport_height) noexcept;
