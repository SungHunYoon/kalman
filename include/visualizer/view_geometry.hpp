#pragma once

#include "visualizer/trajectory_buffer.hpp"
#include <string>
#include <optional>
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

struct ViewOrientation {
	double yaw;
	double pitch;
};

struct ScreenRect {
	double x;
	double y;
	double width;
	double height;
};

enum class ViewMode { Overview, Manual, Follow };

ViewMode next_view_mode(ViewMode current, bool whole_view_pressed,
	bool follow_pressed, bool manual_input) noexcept;
double zoom_vertical_size(double current_size, double wheel_steps) noexcept;
Vector3d cursor_zoom_target(const Vector3d& target, double yaw, double pitch,
	double cursor_x, double cursor_y, int viewport_width, int viewport_height,
	double old_size, double new_size) noexcept;
Vector3d screen_pan_delta(double yaw, double pitch, double dx_pixels,
	double dy_pixels, double vertical_size, int viewport_height) noexcept;
double camera_move_speed(double vertical_size) noexcept;
ViewOrientation preset_view_orientation(int preset) noexcept;
Vector3d camera_up_direction(double yaw, double pitch) noexcept;
bool overview_refit_for_viewport_change(ViewMode mode, bool dimensions_changed) noexcept;
double overview_size_containing_bounds(const Bounds3d& bounds, const Vector3d& target,
	double yaw, double pitch, int viewport_width, int viewport_height,
	double requested_size) noexcept;
bool screen_rects_overlap(const ScreenRect& first, const ScreenRect& second) noexcept;
std::optional<ScreenRect> place_screen_label(const ScreenRect& preferred,
	int viewport_width, int viewport_height,
	const std::vector<ScreenRect>& occupied) noexcept;

CameraFit fit_estimate_bounds(const Bounds3d& bounds, double yaw, double pitch,
	int viewport_width, int viewport_height) noexcept;
