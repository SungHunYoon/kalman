#include "filter/noise_adaptation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

NoiseAdaptation::NoiseAdaptation(double gps_sigma, double gate_threshold)
	: base_variance_(gps_sigma * gps_sigma),
	  base_gate_threshold_(gate_threshold),
	  current_gate_threshold_(gate_threshold) {
	if (!std::isfinite(gps_sigma) || gps_sigma <= 0.0 || !std::isfinite(base_variance_) ||
		!std::isfinite(gate_threshold) || gate_threshold <= 0.0) {
		throw std::invalid_argument("noise adaptation inputs must be positive and finite");
	}
	innovation_ema_.fill(base_variance_);
	gps_variance_.fill(base_variance_);
}

double NoiseAdaptation::current_gate_threshold() const {
	return current_gate_threshold_;
}

const Vector3d& NoiseAdaptation::gps_variance() const {
	return gps_variance_;
}

void NoiseAdaptation::record_rejection() {
	++rejected_count_;
	++consecutive_rejections_;
	if (consecutive_rejections_ % 5 == 0) {
		current_gate_threshold_ = std::min(current_gate_threshold_ * 2.0,
			base_gate_threshold_ * 16.0);
	}
}

void NoiseAdaptation::record_acceptance(const Vector3d& innovation,
	const Vector3d& predicted_position_variance) {
	for (std::size_t axis = 0; axis < innovation.size(); ++axis) {
		if (!std::isfinite(innovation[axis]) || !std::isfinite(predicted_position_variance[axis])) {
			throw std::invalid_argument("adaptation inputs must be finite");
		}
		innovation_ema_[axis] = 0.95 * innovation_ema_[axis] +
			0.05 * innovation[axis] * innovation[axis];
		const double estimated = std::clamp(innovation_ema_[axis] - predicted_position_variance[axis],
			base_variance_ * 0.25, base_variance_ * 100.0);
		gps_variance_[axis] += 0.1 * (estimated - gps_variance_[axis]);
	}
	++accepted_count_;
	consecutive_rejections_ = 0;
	current_gate_threshold_ = base_gate_threshold_;
}

std::uint64_t NoiseAdaptation::accepted_count() const {
	return accepted_count_;
}

std::uint64_t NoiseAdaptation::rejected_count() const {
	return rejected_count_;
}
