#pragma once

#include "filter/fixed_math.hpp"

struct FilterConfig {
	double accel_sigma;
	double gps_sigma;
	double gate_threshold;
};

struct FilterSnapshot {
	Vector3d position{};
	Vector3d velocity{};
	Vector3d position_variance{};
	Vector3d gps_variance{};
};

struct GpsUpdateResult {
	bool accepted = false;
	Vector3d innovation{};
	double mahalanobis_squared = 0.0;
};

class FixedKalmanFilter {
public:
	FixedKalmanFilter(const Vector3d& position, const Vector3d& velocity,
		const FilterConfig& config);
	void predict(const Vector3d& acceleration, double dt);
	GpsUpdateResult update_gps(const Vector3d& gps);
	FilterSnapshot snapshot() const;
	bool invariants_hold() const;

private:
	State6d state_{};
	Matrix6d covariance_{};
	FilterConfig config_;
	Vector3d gps_variance_{};
};
