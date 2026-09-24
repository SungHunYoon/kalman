#include "visualizer/renderer.hpp"

#include "visualizer/render_math.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>

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
	float distance = 25.0f;
};

Renderer::Renderer(int width, int height) : impl_(new Impl) {
	InitWindow(width, height, "Kalman 3D Visualizer");
	SetTargetFPS(60);
	impl_->camera.up = Vector3{0, 1, 0};
	impl_->camera.fovy = 45.0f;
	impl_->camera.projection = CAMERA_PERSPECTIVE;
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
	follow_ = camera_follow_enabled(follow_, IsKeyPressed(KEY_F), moving);
	if (moving) {
		const double speed = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? 40.0 : 10.0;
		const Vector3d delta = camera_move_delta(impl_->yaw, forward, right, up,
			std::min(static_cast<double>(GetFrameTime()), 0.1), speed);
		for (std::size_t axis = 0; axis < impl_->target.size(); ++axis) {
			impl_->target[axis] += delta[axis];
		}
	}
	if (IsKeyPressed(KEY_SPACE)) {
		paused_ = !paused_;
		model.set_history_paused(paused_);
	}
	if (IsKeyPressed(KEY_R)) model.trajectory().clear();
	if (IsKeyPressed(KEY_G)) show_gps_ = !show_gps_;
	if (IsKeyPressed(KEY_C)) show_covariance_ = !show_covariance_;
	if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
		const Vector2 delta = GetMouseDelta();
		impl_->yaw -= delta.x * 0.006f;
		impl_->pitch = std::clamp(impl_->pitch + delta.y * 0.006f, -1.45f, 1.45f);
	}
	impl_->distance = std::clamp(impl_->distance - GetMouseWheelMove() * 2.0f, 2.0f, 500.0f);
}

void Renderer::draw(const ViewerModel& model) {
	const ViewerSnapshot snapshot = model.snapshot(std::chrono::steady_clock::now());
	if (snapshot.has_packet && follow_) {
		impl_->target = smoothed_target(impl_->target, snapshot.latest.estimate_position, 0.12);
	}
	const Vector3 target = to_view(impl_->target);
	const float horizontal = impl_->distance * std::cos(impl_->pitch);
	impl_->camera.target = target;
	impl_->camera.position = Vector3{
		target.x + horizontal * std::cos(impl_->yaw),
		target.y + impl_->distance * std::sin(impl_->pitch),
		target.z + horizontal * std::sin(impl_->yaw)};

	BeginDrawing();
	ClearBackground(Color{12, 16, 24, 255});
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
	DrawText(TextFormat("WASD move  Q/E height  Shift fast  F follow [%s]",
		follow_ ? "ON" : "OFF"), 24, 246, 16, RAYWHITE);
	DrawText("Drag orbit  Wheel zoom  Space pause  R clear", 24, 270, 16, RAYWHITE);
	DrawFPS(GetScreenWidth() - 100, 20);
	EndDrawing();
}
