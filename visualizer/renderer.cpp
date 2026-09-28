#include "visualizer/renderer.hpp"

#include "visualizer/render_math.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace {
	Vector3 to_view(const Vector3d& value) {
		return Vector3{static_cast<float>(value[0]), static_cast<float>(value[2]),
			static_cast<float>(value[1])};
	}
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
};

Renderer::Renderer(int width, int height) : impl_(new Impl) {
	InitWindow(width, height, "Kalman 3D Visualizer");
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
		impl_->yaw = 0.0f;
		impl_->pitch = 1.45f;
		impl_->force_fit = true;
	} else if (IsKeyPressed(KEY_TWO)) {
		impl_->yaw = 1.5707963f;
		impl_->pitch = 0.0f;
		impl_->force_fit = true;
	} else if (IsKeyPressed(KEY_THREE)) {
		impl_->yaw = 0.0f;
		impl_->pitch = 0.0f;
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
				mouse_delta.x, mouse_delta.y, impl_->vertical_size, GetScreenHeight());
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
}

void Renderer::draw(const ViewerModel& model) {
	const ViewerSnapshot snapshot = model.snapshot(std::chrono::steady_clock::now());
	const auto bounds = model.trajectory().estimate_bounds();
	const Bounds3d reference_bounds{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};
	const Bounds3d& fitted_bounds = bounds ? *bounds : reference_bounds;
	const CameraFit fit = fit_estimate_bounds(fitted_bounds, impl_->yaw, impl_->pitch,
		GetScreenWidth(), GetScreenHeight());
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
	const Vector3 target = to_view(impl_->target);
	const Vector3d extent{std::max(std::abs(fitted_bounds.min[0] - impl_->target[0]),
			std::abs(fitted_bounds.max[0] - impl_->target[0])),
		std::max(std::abs(fitted_bounds.min[1] - impl_->target[1]),
			std::abs(fitted_bounds.max[1] - impl_->target[1])),
		std::max(std::abs(fitted_bounds.min[2] - impl_->target[2]),
			std::abs(fitted_bounds.max[2] - impl_->target[2]))};
	const double radius = std::hypot(extent[0], extent[1], extent[2]);
	const double distance = std::max({10.0, fit.camera_distance,
		2.0 * radius + impl_->vertical_size});
	const double far_clip = distance + radius + impl_->vertical_size;
	const float horizontal = static_cast<float>(distance * std::cos(impl_->pitch));
	impl_->camera.target = target;
	impl_->camera.position = Vector3{
		target.x + horizontal * std::cos(impl_->yaw),
		target.y + static_cast<float>(distance * std::sin(impl_->pitch)),
		target.z + horizontal * std::sin(impl_->yaw)};
	impl_->camera.fovy = static_cast<float>(impl_->vertical_size);

	BeginDrawing();
	ClearBackground(Color{12, 16, 24, 255});
	rlSetClipPlanes(0.1, far_clip);
	BeginMode3D(impl_->camera);
	DrawGrid(40, 1.0f);
	DrawLine3D(Vector3{0, 0, 0}, Vector3{5, 0, 0}, RED);
	DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 5, 0}, GREEN);
	DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 0, 5}, BLUE);
	const TrajectoryBuffer& trajectory = model.trajectory();
	for (std::size_t index = 1; index < trajectory.estimate_count(); ++index) {
		DrawLine3D(to_view(trajectory.estimate_at(index - 1)),
			to_view(trajectory.estimate_at(index)), SKYBLUE);
	}
	if (show_gps_) {
		for (std::size_t index = 0; index < trajectory.gps_count(); ++index) {
			DrawSphere(to_view(trajectory.gps_at(index)), 0.08f, ORANGE);
		}
	}
	if (snapshot.has_packet) {
		const Vector3 position = to_view(snapshot.latest.estimate_position);
		DrawSphere(position, 0.18f, YELLOW);
		const Vector3 velocity = to_view(snapshot.latest.estimate_velocity);
		const float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
		if (speed > 1e-6f) {
			DrawLine3D(position, Vector3{position.x + velocity.x / speed * 2.0f,
				position.y + velocity.y / speed * 2.0f, position.z + velocity.z / speed * 2.0f}, LIME);
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

	DrawRectangle(12, 12, 720, 290, Fade(BLACK, 0.72f));
	DrawText(snapshot.connected ? "CONNECTED" : "DISCONNECTED", 24, 22, 20,
		snapshot.connected ? GREEN : RED);
	if (snapshot.has_packet) {
		DrawText(TextFormat("seq %llu  lost %llu  malformed %llu  invalid %llu",
			static_cast<unsigned long long>(snapshot.latest.sequence),
			static_cast<unsigned long long>(snapshot.lost_packets),
			static_cast<unsigned long long>(snapshot.malformed_packets),
			static_cast<unsigned long long>(snapshot.invalid_packets)), 24, 50, 18, RAYWHITE);
		DrawText(TextFormat("pos %.3f %.3f %.3f", snapshot.latest.estimate_position[0],
			snapshot.latest.estimate_position[1], snapshot.latest.estimate_position[2]), 24, 76, 18, RAYWHITE);
		DrawText(TextFormat("vel %.3f %.3f %.3f", snapshot.latest.estimate_velocity[0],
			snapshot.latest.estimate_velocity[1], snapshot.latest.estimate_velocity[2]), 24, 100, 18, RAYWHITE);
		DrawText(TextFormat("acc %.3f %.3f %.3f", snapshot.latest.acceleration[0],
			snapshot.latest.acceleration[1], snapshot.latest.acceleration[2]), 24, 124, 18, RAYWHITE);
		DrawText(TextFormat("filter us now %.3f avg %.3f p95 %.3f p99 %.3f max %.3f",
			snapshot.timing.current, snapshot.timing.average, snapshot.timing.p95,
			snapshot.timing.p99, snapshot.timing.maximum), 24, 148, 18, RAYWHITE);
		DrawText(TextFormat("GPS model variance %.3f %.3f %.3f",
			snapshot.latest.adaptive_gps_variance[0],
			snapshot.latest.adaptive_gps_variance[1],
			snapshot.latest.adaptive_gps_variance[2]), 24, 172, 18, RAYWHITE);
		if (snapshot.has_last_gps_innovation) {
			DrawText(TextFormat("last innovation norm %.3f",
				vector_norm(snapshot.last_gps_innovation)),
				24, 196, 18, RAYWHITE);
		} else {
			DrawText("innovation: waiting for GPS", 24, 196, 18, RAYWHITE);
		}
		DrawText(TextFormat("GPS applied %llu  [%s]",
			static_cast<unsigned long long>(snapshot.latest.accepted_gps_count),
			paused_ ? "PAUSED" : "LIVE"), 24, 220, 18, RAYWHITE);
	}
	DrawText(TextFormat("WASD move  Q/E height  Shift fast  view [%s]",
		mode_ == ViewMode::Overview ? "OVERVIEW" :
		mode_ == ViewMode::Follow ? "FOLLOW" : "MANUAL"), 24, 246, 16, RAYWHITE);
	DrawText("H whole view  F follow  1/2/3 views  Left orbit  Right pan  Wheel zoom", 24, 270, 16, RAYWHITE);
	DrawFPS(GetScreenWidth() - 100, 20);
	EndDrawing();
}
