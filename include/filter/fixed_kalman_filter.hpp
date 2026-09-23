#pragma once

#include "filter/fixed_math.hpp"
#include "filter/filter_snapshot.hpp"
#include "filter/noise_adaptation.hpp"

#include <cstddef>

struct FilterConfig {
	double accel_sigma;
	double gps_sigma;
	double gate_threshold;
	bool adaptive_noise = true;
	bool innovation_gating = true;
	double direction_sigma = 0.01;
};

class FixedKalmanFilter {
public:
	FixedKalmanFilter(const Vector3d& position, const Vector3d& velocity,
		const FilterConfig& config);
	void predict(const Vector3d& acceleration, double dt);
	GpsUpdateResult update_gps(const Vector3d& gps);
	bool update_direction(const Vector3d& direction);
	FilterSnapshot snapshot() const;
	bool invariants_hold() const;
	std::uint64_t accepted_gps_count() const;
	std::uint64_t rejected_gps_count() const;

private:
	State6d state_{};
	Matrix6d covariance_{};
	FilterConfig config_;
	NoiseAdaptation adaptation_;
	static constexpr std::size_t DIRECTION_WINDOW = 16;
	std::array<Vector3d, DIRECTION_WINDOW> direction_samples_{};
	std::size_t direction_sample_count_ = 0;
	std::size_t next_direction_sample_ = 0;
};
