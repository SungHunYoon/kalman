#include "protocol/sensor_state.hpp"

#include <cmath>

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
		filter.emplace(initial_position, initial_velocity(speed_kmh, direction));
		filter_time = update.time;
		estimate = filter->position();
		return;
	}
	if (!filter) {
		return;
	}

	filter->predict(acceleration, dt);
	filter_time = update.time;
	if (update.gps) {
		filter->update_gps(*update.gps);
	}
	estimate = filter->position();
}

bool SensorState::has_estimated_position() const {
	return filter.has_value();
}

const Vector<double>& SensorState::estimated_position() const {
	return estimate;
}
