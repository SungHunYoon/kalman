#pragma once

#include "linear_algebra/vector.hpp"
#include "protocol/sensor_update.hpp"

class SensorState {
	private:
		double current_time;
		double previous_time;

		Vector<double> position;
		double speed_kmh;
		Vector<double> acceleration;
		Vector<double> direction;
		Vector<double> gps;

		bool has_position;
		bool has_speed;
		bool has_acceleration;
		bool has_direction;
		bool has_gps;

	public:
		SensorState();

		void apply(const SensorUpdate& update);
		bool has_estimated_position() const;
		const Vector<double>& estimated_position() const;
};
