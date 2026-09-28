#pragma once

#include "visualizer/trajectory_buffer.hpp"
#include <string>
#include <vector>

Bounds3d padded_axis_bounds(const Bounds3d& raw) noexcept;
double nice_tick_step(double span) noexcept;
std::vector<double> axis_ticks(double minimum, double maximum, double step);
std::string format_axis_tick(double value, double step);

struct CameraFit {
	Vector3d target;
	double vertical_size;
	double camera_distance;
	double far_clip;
};

enum class ViewMode { Overview, Manual, Follow };

ViewMode next_view_mode(ViewMode current, bool whole_view_pressed,
	bool follow_pressed, bool manual_input) noexcept;
double zoom_vertical_size(double current_size, double wheel_steps) noexcept;
Vector3d screen_pan_delta(double yaw, double pitch, double dx_pixels,
	double dy_pixels, double vertical_size, int viewport_height) noexcept;
double camera_move_speed(double vertical_size) noexcept;

CameraFit fit_estimate_bounds(const Bounds3d& bounds, double yaw, double pitch,
	int viewport_width, int viewport_height) noexcept;
