#pragma once

#include "filter/filter_snapshot.hpp"
#include "filter/kalman_filter.hpp"
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
		GpsUpdateResult last_gps_result_;
		FilterStats timing_stats_;
		std::uint64_t applied_gps_count_ = 0;

		bool has_initial_position;
		bool has_speed;
		bool has_direction;
		std::optional<KalmanFilter> filter;

	public:
		SensorState();

		bool apply(const SensorUpdate& update);
		bool has_estimated_position() const;
		const Vector<double>& estimated_position() const;
		FilterSnapshot filter_snapshot() const;
		GpsUpdateResult last_gps_result() const;
		std::uint64_t accepted_gps_count() const;
		std::uint64_t rejected_gps_count() const;
		FilterTimingSnapshot timing_snapshot(bool include_percentiles = true) const;
};
