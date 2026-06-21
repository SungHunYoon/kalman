#include "protocol/sensor_state.hpp"

SensorState::SensorState()
	: current_time(0.0),
	  previous_time(0.0),
	  position(3, 0.0),
	  speed_kmh(0.0),
	  acceleration(3, 0.0),
	  direction(3, 0.0),
	  gps(3, 0.0),
	  has_position(false),
	  has_speed(false),
	  has_acceleration(false),
	  has_direction(false),
	  has_gps(false) {}

void SensorState::apply(const SensorUpdate& update) {
	previous_time = current_time;
	current_time = update.time;

	if (update.initial_position) {
		position = *update.initial_position;
		has_position = true;
	}
	if (update.initial_speed_kmh) {
		speed_kmh = *update.initial_speed_kmh;
		has_speed = true;
	}
	if (update.acceleration) {
		acceleration = *update.acceleration;
		has_acceleration = true;
	}
	if (update.direction) {
		direction = *update.direction;
		has_direction = true;
	}
	if (update.gps) {
		gps = *update.gps;
		has_gps = true;
	}
}

bool SensorState::has_estimated_position() const {
	return has_position;
}

const Vector<double>& SensorState::estimated_position() const {
	return position;
}
