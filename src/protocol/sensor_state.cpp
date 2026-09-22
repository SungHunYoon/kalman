#include "protocol/sensor_state.hpp"

#include <cmath>
#include <chrono>

namespace {
	Vector<double> initial_velocity(double speed_kmh, const Vector<double>& direction) {
		const double speed_mps = speed_kmh / 3.6;
		const double pitch = direction[1];
		const double yaw = direction[2];
		return Vector<double>{
			speed_mps * std::cos(yaw) * std::cos(pitch),
			speed_mps * std::sin(yaw) * std::cos(pitch),
			-speed_mps * std::sin(pitch)
		};
	}

	double elapsed_seconds(double previous, double current) {
		double elapsed = current - previous;
		if (elapsed < -43200.0) {
			elapsed += 86400.0;
		}
		return elapsed;
	}

	Vector3d to_vector3d(const Vector<double>& vector) {
		return Vector3d{vector[0], vector[1], vector[2]};
	}

	Vector<double> to_vector(const Vector3d& vector) {
		return Vector<double>{vector[0], vector[1], vector[2]};
	}
}

SensorState::SensorState(const FilterConfig& filter_config)
	: filter_time(0.0),
	  speed_kmh(0.0),
	  acceleration(3, 0.0),
	  direction(3, 0.0),
	  initial_position(3, 0.0),
	  estimate(3, 0.0),
	  config(filter_config),
	  last_gps_result_(),
	  timing_stats_(),
	  has_initial_position(false),
	  has_speed(false),
	  has_direction(false),
	  filter() {}

void SensorState::apply(const SensorUpdate& update) {
	double dt = 0.0;
	if (filter) {
		dt = elapsed_seconds(filter_time, update.time);
		if (dt <= 0.0) {
			return;
		}
	}

	if (update.initial_position) {
		initial_position = *update.initial_position;
		has_initial_position = true;
	}
	if (update.initial_speed_kmh) {
		speed_kmh = *update.initial_speed_kmh;
		has_speed = true;
	}
	if (update.acceleration) {
		acceleration = *update.acceleration;
	}
	if (update.direction) {
		direction = *update.direction;
		has_direction = true;
	}

	if (!filter && has_initial_position && has_speed && has_direction) {
		filter.emplace(to_vector3d(initial_position),
			to_vector3d(initial_velocity(speed_kmh, direction)), config);
		filter_time = update.time;
		estimate = to_vector(filter->snapshot().position);
		return;
	}
	if (!filter) {
		return;
	}

	const auto started = std::chrono::steady_clock::now();
	last_gps_result_ = GpsUpdateResult{};
	filter->predict(to_vector3d(acceleration), dt);
	filter_time = update.time;
	if (update.gps) {
		last_gps_result_ = filter->update_gps(to_vector3d(*update.gps));
	}
	const auto finished = std::chrono::steady_clock::now();
	timing_stats_.record(std::chrono::duration<double, std::micro>(finished - started).count());
	estimate = to_vector(filter->snapshot().position);
}

bool SensorState::has_estimated_position() const {
	return filter.has_value();
}

const Vector<double>& SensorState::estimated_position() const {
	return estimate;
}

FilterSnapshot SensorState::filter_snapshot() const {
	return filter ? filter->snapshot() : FilterSnapshot{};
}

GpsUpdateResult SensorState::last_gps_result() const {
	return last_gps_result_;
}

std::uint64_t SensorState::accepted_gps_count() const {
	return filter ? filter->accepted_gps_count() : 0;
}

std::uint64_t SensorState::rejected_gps_count() const {
	return filter ? filter->rejected_gps_count() : 0;
}

FilterTimingSnapshot SensorState::timing_snapshot() const {
	return timing_stats_.snapshot();
}
