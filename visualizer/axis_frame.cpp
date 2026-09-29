#include "visualizer/axis_frame.hpp"

#include <cmath>

bool draw_world_label(const char* text, const Vector3d& point, const Camera3D& camera,
	int font_size, Color color, std::vector<ScreenRect>* occupied, int offset_y) {
	const Vector3 world = to_view(point);
	const Vector3 forward{camera.target.x - camera.position.x,
		camera.target.y - camera.position.y, camera.target.z - camera.position.z};
	const double depth = (world.x - camera.position.x) * forward.x +
		(world.y - camera.position.y) * forward.y + (world.z - camera.position.z) * forward.z;
	if (depth <= 0.0) return false;
	const Vector2 screen = GetWorldToScreen(world, camera);
	const ScreenRect preferred{screen.x + 6.0, screen.y + offset_y,
		static_cast<double>(MeasureText(text, font_size) + 6),
		static_cast<double>(font_size + 4)};
	if (!std::isfinite(preferred.x) || !std::isfinite(preferred.y)) return false;
	const std::vector<ScreenRect> empty;
	const auto placement = place_screen_label(preferred, GetScreenWidth(),
		GetScreenHeight(), occupied ? *occupied : empty);
	if (!placement) return false;
	if (occupied) occupied->push_back(*placement);
	const int x = static_cast<int>(placement->x);
	const int y = static_cast<int>(placement->y);
	DrawText(text, x + 1, y + 1, font_size, BLACK);
	DrawText(text, x, y, font_size, color);
	return true;
}

void draw_axis_frame_3d(const Bounds3d& bounds) {
	const Color edge{95, 105, 120, 255};
	const Color grid{46, 55, 68, 255};
	for (std::size_t axis = 0; axis < 3; ++axis) {
		const std::size_t other1 = (axis + 1) % 3;
		const std::size_t other2 = (axis + 2) % 3;
		for (int side1 = 0; side1 < 2; ++side1) {
			for (int side2 = 0; side2 < 2; ++side2) {
				Vector3d a = bounds.min;
				a[other1] = side1 ? bounds.max[other1] : bounds.min[other1];
				a[other2] = side2 ? bounds.max[other2] : bounds.min[other2];
				Vector3d b = a;
				b[axis] = bounds.max[axis];
				DrawLine3D(to_view(a), to_view(b), edge);
			}
		}
		const double step = nice_tick_step(bounds.max[axis] - bounds.min[axis]);
		for (double tick : axis_ticks(bounds.min[axis], bounds.max[axis], step)) {
			Vector3d a = bounds.min;
			a[axis] = tick;
			for (std::size_t other : {other1, other2}) {
				Vector3d b = a;
				b[other] = bounds.max[other];
				DrawLine3D(to_view(a), to_view(b), grid);
			}
		}
	}
}

void draw_axis_labels_2d(const Bounds3d& bounds, const Camera3D& camera,
	std::vector<ScreenRect>& occupied) {
	const char* names[3] = {"X", "Y", "Z"};
	for (std::size_t axis = 0; axis < 3; ++axis) {
		Vector3d endpoint = bounds.min;
		endpoint[axis] = bounds.max[axis];
		draw_world_label(names[axis], endpoint, camera, 20, RAYWHITE, &occupied, -24);
	}
	for (std::size_t axis = 0; axis < 3; ++axis) {
		const double step = nice_tick_step(bounds.max[axis] - bounds.min[axis]);
		for (double tick : axis_ticks(bounds.min[axis], bounds.max[axis], step)) {
			Vector3d point = bounds.min;
			point[axis] = tick;
			const std::string text = format_axis_tick(tick, step);
			draw_world_label(text.c_str(), point, camera, 14,
				Color{165, 175, 190, 255}, &occupied, 3);
		}
	}
}
