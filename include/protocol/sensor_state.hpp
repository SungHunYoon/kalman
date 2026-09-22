#pragma once

#include "filter/fixed_kalman_filter.hpp"
#include "linear_algebra/vector.hpp"
#include "performance/filter_stats.hpp"
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
		FilterConfig config;
		GpsUpdateResult last_gps_result_;
		FilterStats timing_stats_;

		bool has_initial_position;
		bool has_speed;
		bool has_direction;
		std::optional<FixedKalmanFilter> filter;

	public:
		explicit SensorState(const FilterConfig& config = {1e-2, 1.0, 11.345});

		void apply(const SensorUpdate& update);
		bool has_estimated_position() const;
		const Vector<double>& estimated_position() const;
		FilterSnapshot filter_snapshot() const;
		GpsUpdateResult last_gps_result() const;
		std::uint64_t accepted_gps_count() const;
		std::uint64_t rejected_gps_count() const;
		FilterTimingSnapshot timing_snapshot() const;
};
