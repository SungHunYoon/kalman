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

}

SensorState::SensorState()
	: filter_time(0.0),
	  speed_kmh(0.0),
	  acceleration(3, 0.0),
	  direction(3, 0.0),
	  initial_position(3, 0.0),
	  estimate(3, 0.0),
	  last_gps_result_(),
	  timing_stats_(),
	  has_initial_position(false),
	  has_speed(false),
	  has_direction(false),
	  filter() {}

bool SensorState::apply(const SensorUpdate& update) {
	double dt = 0.0;
	if (filter) {
		dt = elapsed_seconds(filter_time, update.time);
		if (dt <= 0.0) {
			return false;
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
		filter.emplace(initial_position, initial_velocity(speed_kmh, direction));
		filter_time = update.time;
		estimate = filter->position();
		return true;
	}
	if (!filter) {
		return false;
	}

	const auto started = std::chrono::steady_clock::now();
	last_gps_result_ = GpsUpdateResult{};
	filter->predict(acceleration, dt);
	filter_time = update.time;
	if (update.gps) {
		const Vector<double> predicted_position = filter->position();
		for (std::size_t axis = 0; axis < 3; ++axis) {
			last_gps_result_.innovation[axis] = (*update.gps)[axis] - predicted_position[axis];
		}
		filter->update_gps(*update.gps);
		++applied_gps_count_;
	}
	const auto finished = std::chrono::steady_clock::now();
	timing_stats_.record(std::chrono::duration<double, std::micro>(finished - started).count());
	estimate = filter->position();
	return true;
}

bool SensorState::has_estimated_position() const {
	return filter.has_value();
}

const Vector<double>& SensorState::estimated_position() const {
	return estimate;
}

FilterSnapshot SensorState::filter_snapshot() const {
	FilterSnapshot result;
	if (!filter) {
		return result;
	}
	const Vector<double> position = filter->position();
	const Vector<double> velocity = filter->velocity();
	const Matrix<double>& covariance = filter->state_covariance();
	for (std::size_t axis = 0; axis < 3; ++axis) {
		result.position[axis] = position[axis];
		result.velocity[axis] = velocity[axis];
		result.position_variance[axis] = covariance(axis, axis);
		result.gps_variance[axis] = 1.0;
	}
	return result;
}

GpsUpdateResult SensorState::last_gps_result() const {
	return last_gps_result_;
}

std::uint64_t SensorState::accepted_gps_count() const {
	return applied_gps_count_;
}

std::uint64_t SensorState::rejected_gps_count() const {
	return 0;
}

FilterTimingSnapshot SensorState::timing_snapshot() const {
	return timing_stats_.snapshot();
}
