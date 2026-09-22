#pragma once

#include "filter/fixed_math.hpp"

#include <cstdint>

class NoiseAdaptation {
public:
	NoiseAdaptation(double gps_sigma, double gate_threshold);
	double current_gate_threshold() const;
	const Vector3d& gps_variance() const;
	void record_rejection();
	void record_acceptance(const Vector3d& innovation,
		const Vector3d& predicted_position_variance);
	std::uint64_t accepted_count() const;
	std::uint64_t rejected_count() const;

private:
	double base_variance_;
	double base_gate_threshold_;
	double current_gate_threshold_;
	Vector3d innovation_ema_{};
	Vector3d gps_variance_{};
	std::uint32_t consecutive_rejections_ = 0;
	std::uint64_t accepted_count_ = 0;
	std::uint64_t rejected_count_ = 0;
};
