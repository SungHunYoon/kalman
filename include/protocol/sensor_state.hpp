#pragma once

#include "filter/kalman_filter.hpp"
#include "linear_algebra/vector.hpp"
#include "protocol/sensor_update.hpp"

#include <optional>

class SensorState {
	private:
		double filter_time;
		double speed_kmh;
		Vector<double> acceleration;
		Vector<double> direction;
		Vector<double> initial_position;
		Vector<double> estimate;

		bool has_initial_position;
		bool has_speed;
		bool has_direction;
		std::optional<KalmanFilter> filter;

	public:
		SensorState();

		void apply(const SensorUpdate& update);
		bool has_estimated_position() const;
		const Vector<double>& estimated_position() const;
};
