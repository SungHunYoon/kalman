#include "visualizer/renderer.hpp"

#include "visualizer/render_math.hpp"
#include "visualizer/axis_frame.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace {
	constexpr int SCENE_TOP = 206;
	int scene_height() { return std::max(1, GetScreenHeight() - SCENE_TOP); }
}

struct Renderer::Impl {
	Camera3D camera{};
	Vector3d target{};
	float yaw = 0.8f;
	float pitch = 0.55f;
	double vertical_size = 10.0;
	std::optional<Bounds3d> previous_bounds;
	bool force_fit = true;
	bool smoothing_fit = false;
	bool entering_follow = false;
	bool show_details = false;
	int viewport_width = 0;
	int viewport_height = 0;
};

Renderer::Renderer(int width, int height) : impl_(new Impl) {
	InitWindow(width, height, "Kalman 3D Visualizer");
	SetWindowState(FLAG_WINDOW_RESIZABLE);
	SetTargetFPS(60);
	impl_->camera.up = Vector3{0, 1, 0};
	impl_->camera.fovy = static_cast<float>(impl_->vertical_size);
	impl_->camera.projection = CAMERA_ORTHOGRAPHIC;
}

Renderer::~Renderer() {
	CloseWindow();
	delete impl_;
}

bool Renderer::should_close() const { return WindowShouldClose(); }

void Renderer::update_controls(ViewerModel& model) {
	const double forward = static_cast<double>(IsKeyDown(KEY_W)) - IsKeyDown(KEY_S);
	const double right = static_cast<double>(IsKeyDown(KEY_D)) - IsKeyDown(KEY_A);
	const double up = static_cast<double>(IsKeyDown(KEY_E)) - IsKeyDown(KEY_Q);
	const bool moving = forward != 0.0 || right != 0.0 || up != 0.0;
	const Vector2 mouse_delta = GetMouseDelta();
	const bool orbiting = IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
		(mouse_delta.x != 0.0f || mouse_delta.y != 0.0f);
	const bool panning = IsMouseButtonDown(MOUSE_BUTTON_RIGHT) &&
		(mouse_delta.x != 0.0f || mouse_delta.y != 0.0f);
	const double wheel_steps = GetMouseWheelMove();
	const bool whole_view_pressed = IsKeyPressed(KEY_H);
	const ViewMode old_mode = mode_;
	mode_ = next_view_mode(mode_, whole_view_pressed, IsKeyPressed(KEY_F),
		moving || panning || wheel_steps != 0.0);
	if (whole_view_pressed) impl_->force_fit = true;
	if (mode_ == ViewMode::Follow && old_mode != ViewMode::Follow) {
		impl_->entering_follow = true;
	}
	if (orbiting) {
		impl_->yaw -= mouse_delta.x * 0.006f;
		impl_->pitch = std::clamp(impl_->pitch + mouse_delta.y * 0.006f, -1.45f, 1.45f);
		impl_->force_fit = true;
	}
	if (IsKeyPressed(KEY_ONE)) {
		const ViewOrientation orientation = preset_view_orientation(1);
		impl_->yaw = static_cast<float>(orientation.yaw);
		impl_->pitch = static_cast<float>(orientation.pitch);
		impl_->force_fit = true;
	} else if (IsKeyPressed(KEY_TWO)) {
		const ViewOrientation orientation = preset_view_orientation(2);
		impl_->yaw = static_cast<float>(orientation.yaw);
		impl_->pitch = static_cast<float>(orientation.pitch);
		impl_->force_fit = true;
	} else if (IsKeyPressed(KEY_THREE)) {
		const ViewOrientation orientation = preset_view_orientation(3);
		impl_->yaw = static_cast<float>(orientation.yaw);
		impl_->pitch = static_cast<float>(orientation.pitch);
		impl_->force_fit = true;
	}
	if (!whole_view_pressed) {
		if (moving) {
			const double speed = camera_move_speed(impl_->vertical_size) *
				((IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 4.0 : 1.0);
			const Vector3d delta = camera_move_delta(impl_->yaw, forward, right, up,
				std::clamp(static_cast<double>(GetFrameTime()), 0.0, 0.1), speed);
			for (std::size_t axis = 0; axis < impl_->target.size(); ++axis) {
				impl_->target[axis] += delta[axis];
			}
		}
		if (panning) {
			const Vector3d delta = screen_pan_delta(impl_->yaw, impl_->pitch,
				mouse_delta.x, mouse_delta.y, impl_->vertical_size, scene_height());
			for (std::size_t axis = 0; axis < impl_->target.size(); ++axis) {
				impl_->target[axis] += delta[axis];
			}
		}
		if (wheel_steps != 0.0) {
			impl_->vertical_size = zoom_vertical_size(impl_->vertical_size, wheel_steps);
		}
	}
	if (IsKeyPressed(KEY_SPACE)) {
		paused_ = !paused_;
		model.set_history_paused(paused_);
	}
	if (IsKeyPressed(KEY_R)) {
		model.trajectory().clear();
		impl_->previous_bounds.reset();
		impl_->force_fit = true;
	}
	if (IsKeyPressed(KEY_G)) show_gps_ = !show_gps_;
	if (IsKeyPressed(KEY_C)) show_covariance_ = !show_covariance_;
	if (IsKeyPressed(KEY_TAB)) impl_->show_details = !impl_->show_details;
}

void Renderer::draw(const ViewerModel& model) {
	const ViewerSnapshot snapshot = model.snapshot(std::chrono::steady_clock::now());
	const int viewport_width = GetScreenWidth();
	const int viewport_height = scene_height();
	const bool viewport_changed = viewport_width != impl_->viewport_width ||
		viewport_height != impl_->viewport_height;
	if (overview_refit_for_viewport_change(mode_, viewport_changed)) {
		impl_->force_fit = true;
	}
	impl_->viewport_width = viewport_width;
	impl_->viewport_height = viewport_height;
	const auto bounds = model.trajectory().estimate_bounds();
	const Bounds3d reference_bounds{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};
	const Bounds3d fitted_bounds = bounds ? padded_axis_bounds(*bounds) : reference_bounds;
	const CameraFit fit = fit_estimate_bounds(fitted_bounds, impl_->yaw, impl_->pitch,
		viewport_width, viewport_height);
	if (mode_ == ViewMode::Overview) {
		bool expanding = false;
		bool contracting = false;
		if (bounds && impl_->previous_bounds) {
			for (std::size_t axis = 0; axis < 3; ++axis) {
				expanding = expanding || bounds->min[axis] < impl_->previous_bounds->min[axis] ||
					bounds->max[axis] > impl_->previous_bounds->max[axis];
				contracting = contracting || bounds->min[axis] > impl_->previous_bounds->min[axis] ||
					bounds->max[axis] < impl_->previous_bounds->max[axis];
			}
		}
		if (impl_->force_fit || !bounds || !impl_->previous_bounds || contracting) {
			impl_->target = fit.target;
			impl_->vertical_size = fit.vertical_size;
			impl_->smoothing_fit = false;
		} else {
			if (expanding) impl_->smoothing_fit = true;
			if (impl_->smoothing_fit) {
				impl_->target = smoothed_target(impl_->target, fit.target, 0.12);
				impl_->vertical_size += 0.12 * (fit.vertical_size - impl_->vertical_size);
				if (std::abs(impl_->vertical_size - fit.vertical_size) < 1e-6 &&
					vector_norm(Vector3d{impl_->target[0] - fit.target[0],
						impl_->target[1] - fit.target[1],
						impl_->target[2] - fit.target[2]}) < 1e-6) {
					impl_->target = fit.target;
					impl_->vertical_size = fit.vertical_size;
					impl_->smoothing_fit = false;
				}
			}
		}
		impl_->vertical_size = overview_size_containing_bounds(fitted_bounds,
			impl_->target, impl_->yaw, impl_->pitch, viewport_width, viewport_height,
			impl_->vertical_size);
	} else if (mode_ == ViewMode::Follow) {
		impl_->smoothing_fit = false;
		if (impl_->entering_follow) {
			impl_->vertical_size = std::max(20.0, fit.vertical_size * 0.05);
			impl_->entering_follow = false;
		}
	} else {
		impl_->smoothing_fit = false;
	}
	if (mode_ == ViewMode::Follow && snapshot.has_packet) {
		impl_->target = smoothed_target(impl_->target, snapshot.latest.estimate_position, 0.12);
	}
	impl_->previous_bounds = bounds;
	impl_->force_fit = false;
	// Compose the scene below the compact panel while keeping raylib's full-window
	// projection so GetWorldToScreen uses the same coordinates as the 3D drawing.
	const double screen_shift = 0.5 * SCENE_TOP * impl_->vertical_size / scene_height();
	Vector3d composed_target = impl_->target;
	composed_target[0] -= screen_shift * std::sin(impl_->pitch) * std::cos(impl_->yaw);
	composed_target[1] -= screen_shift * std::sin(impl_->pitch) * std::sin(impl_->yaw);
	composed_target[2] += screen_shift * std::cos(impl_->pitch);
	const Vector3 target = to_view(composed_target);
	const Vector3d extent{std::max(std::abs(fitted_bounds.min[0] - impl_->target[0]),
			std::abs(fitted_bounds.max[0] - impl_->target[0])),
		std::max(std::abs(fitted_bounds.min[1] - impl_->target[1]),
			std::abs(fitted_bounds.max[1] - impl_->target[1])),
		std::max(std::abs(fitted_bounds.min[2] - impl_->target[2]),
			std::abs(fitted_bounds.max[2] - impl_->target[2]))};
	const double radius = std::hypot(extent[0], extent[1], extent[2]) + screen_shift;
	const double distance = std::max({10.0, fit.camera_distance,
		2.0 * radius + impl_->vertical_size});
	const double far_clip = distance + radius + impl_->vertical_size;
	const float horizontal = static_cast<float>(distance * std::cos(impl_->pitch));
	impl_->camera.target = target;
	const Vector3d camera_up = camera_up_direction(impl_->yaw, impl_->pitch);
	impl_->camera.up = to_view(camera_up);
	impl_->camera.position = Vector3{
		target.x + horizontal * std::cos(impl_->yaw),
		target.y + static_cast<float>(distance * std::sin(impl_->pitch)),
		target.z + horizontal * std::sin(impl_->yaw)};
	impl_->camera.fovy = static_cast<float>(impl_->vertical_size * GetScreenHeight() / scene_height());

	BeginDrawing();
	ClearBackground(Color{12, 16, 24, 255});
	rlSetClipPlanes(0.1, far_clip);
	BeginMode3D(impl_->camera);
	draw_axis_frame_3d(fitted_bounds);
	const TrajectoryBuffer& trajectory = model.trajectory();
	const std::size_t count = trajectory.estimate_count();
	const float marker_radius = static_cast<float>(impl_->vertical_size * 0.007);
	for (std::size_t index = 1; index < count; ++index) {
		const float progress = static_cast<float>(index) / static_cast<float>(count - 1);
		DrawLine3D(to_view(trajectory.estimate_at(index - 1)),
			to_view(trajectory.estimate_at(index)), ColorLerp(SKYBLUE, ORANGE, progress));
	}
	if (count != 0) {
		DrawSphere(to_view(trajectory.estimate_at(0)), marker_radius, GREEN);
		DrawSphere(to_view(trajectory.estimate_at(count - 1)), marker_radius, ORANGE);
	}
	if (show_gps_) {
		for (std::size_t index = 0; index < trajectory.gps_count(); ++index) {
			DrawSphere(to_view(trajectory.gps_at(index)), marker_radius * 0.35f,
				Color{120, 130, 160, 130});
		}
	}
	if (snapshot.has_packet) {
		const Vector3 position = to_view(snapshot.latest.estimate_position);
		if (snapshot.connected) {
			DrawSphereWires(position, marker_radius * 1.7f, 8, 12, YELLOW);
		}
		if (show_covariance_) {
			const float rx = static_cast<float>(covariance_radius(snapshot.latest.position_variance[0]));
			const float ry = static_cast<float>(covariance_radius(snapshot.latest.position_variance[2]));
			const float rz = static_cast<float>(covariance_radius(snapshot.latest.position_variance[1]));
			rlPushMatrix();
			rlTranslatef(position.x, position.y, position.z);
			rlScalef(rx, ry, rz);
			DrawSphereWires(Vector3{0, 0, 0}, 1.0f, 12, 12, PURPLE);
			rlPopMatrix();
		}
	}
	EndMode3D();
	std::vector<ScreenRect> occupied_labels;
	draw_axis_labels_2d(fitted_bounds, impl_->camera, occupied_labels);
	if (count != 0) {
		draw_world_label(trajectory.estimate_history_truncated() ? "Oldest retained" : "Start",
			trajectory.estimate_at(0), impl_->camera, 18, GREEN, &occupied_labels, -24);
		draw_world_label("End", trajectory.estimate_at(count - 1), impl_->camera,
			18, ORANGE, &occupied_labels);
	} else {
		const char* waiting = "Waiting for estimated trajectory";
		DrawText(waiting, (GetScreenWidth() - MeasureText(waiting, 22)) / 2,
			GetScreenHeight() / 2 + 50, 22, RAYWHITE);
	}

	DrawRectangle(12, 12, 442, 182, Fade(BLACK, 0.82f));
	DrawText(TextFormat("%s  |  %s  %s",
		snapshot.connected ? "CONNECTED" : "DISCONNECTED",
		mode_ == ViewMode::Overview ? "OVERVIEW" :
		mode_ == ViewMode::Follow ? "FOLLOW" : "MANUAL", paused_ ? "[PAUSED]" : ""),
		24, 22, 18, snapshot.connected ? GREEN : LIGHTGRAY);
	DrawText(TextFormat("Retained %llu  |  Lost packets %llu",
		static_cast<unsigned long long>(count), static_cast<unsigned long long>(snapshot.lost_packets)),
		24, 47, 16, snapshot.lost_packets ? YELLOW : RAYWHITE);
	DrawText(trajectory.estimate_history_truncated() ? "History truncated - oldest samples overwritten" :
		"Path: received estimates retained in memory", 24, 69, 16,
		trajectory.estimate_history_truncated() ? YELLOW : LIGHTGRAY);
	DrawText("H whole view   F follow   1/2/3 top/front/side", 24, 94, 16, RAYWHITE);
	DrawText("Left drag orbit   Right drag pan   Wheel zoom", 24, 114, 16, RAYWHITE);
	DrawText("WASD move   Q/E height   Shift fast", 24, 134, 16, RAYWHITE);
	DrawText(TextFormat("Space pause  R clear  G GPS:%s  C cov:%s",
		show_gps_ ? "on" : "off", show_covariance_ ? "on" : "off"), 24, 154, 15, RAYWHITE);
	DrawText("Tab details   Esc close", 24, 174, 15, LIGHTGRAY);
	if (impl_->show_details && snapshot.has_packet) {
		DrawRectangle(12, 202, 670, 222, Fade(BLACK, 0.82f));
		DrawText(TextFormat("seq %llu  lost %llu  malformed %llu  invalid %llu",
			static_cast<unsigned long long>(snapshot.latest.sequence),
			static_cast<unsigned long long>(snapshot.lost_packets),
			static_cast<unsigned long long>(snapshot.malformed_packets),
			static_cast<unsigned long long>(snapshot.invalid_packets)), 24, 212, 18, RAYWHITE);
		DrawText(TextFormat("pos %.3f %.3f %.3f", snapshot.latest.estimate_position[0],
			snapshot.latest.estimate_position[1], snapshot.latest.estimate_position[2]), 24, 238, 18, RAYWHITE);
		DrawText(TextFormat("vel %.3f %.3f %.3f", snapshot.latest.estimate_velocity[0],
			snapshot.latest.estimate_velocity[1], snapshot.latest.estimate_velocity[2]), 24, 262, 18, RAYWHITE);
		DrawText(TextFormat("acc %.3f %.3f %.3f", snapshot.latest.acceleration[0],
			snapshot.latest.acceleration[1], snapshot.latest.acceleration[2]), 24, 286, 18, RAYWHITE);
		DrawText(TextFormat("filter us now %.3f avg %.3f p95 %.3f p99 %.3f max %.3f",
			snapshot.timing.current, snapshot.timing.average, snapshot.timing.p95,
			snapshot.timing.p99, snapshot.timing.maximum), 24, 310, 18, RAYWHITE);
		DrawText(TextFormat("GPS model variance %.3f %.3f %.3f",
			snapshot.latest.adaptive_gps_variance[0],
			snapshot.latest.adaptive_gps_variance[1],
			snapshot.latest.adaptive_gps_variance[2]), 24, 334, 18, RAYWHITE);
		if (snapshot.has_last_gps_innovation) {
			DrawText(TextFormat("last innovation norm %.3f",
				vector_norm(snapshot.last_gps_innovation)),
				24, 358, 18, RAYWHITE);
		} else {
			DrawText("innovation: waiting for GPS", 24, 358, 18, RAYWHITE);
		}
		DrawText(TextFormat("GPS applied %llu  [%s]",
			static_cast<unsigned long long>(snapshot.latest.accepted_gps_count),
			paused_ ? "PAUSED" : "LIVE"), 24, 382, 18, RAYWHITE);
	}
	DrawFPS(GetScreenWidth() - 100, 20);
	EndDrawing();
}
