#pragma once

#include "visualizer/view_geometry.hpp"
#include <raylib.h>

// Telemetry X/Y are horizontal; telemetry Z is raylib's up axis.
inline Vector3 to_view(const Vector3d& point) {
	return Vector3{static_cast<float>(point[0]), static_cast<float>(point[2]),
		static_cast<float>(point[1])};
}

void draw_axis_frame_3d(const Bounds3d& bounds);
void draw_axis_labels_2d(const Bounds3d& bounds, const Camera3D& camera);
void draw_world_label(const char* text, const Vector3d& point, const Camera3D& camera,
	int font_size, Color color, int offset_y = 6);
