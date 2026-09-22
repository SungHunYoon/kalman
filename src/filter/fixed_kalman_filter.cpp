#include "filter/fixed_kalman_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace {
	constexpr std::size_t DIMENSIONS = 3;
	constexpr std::size_t STATE_SIZE = 6;

	constexpr std::size_t index6(std::size_t row, std::size_t column) {
		return row * STATE_SIZE + column;
	}

	constexpr std::size_t index3(std::size_t row, std::size_t column) {
		return row * DIMENSIONS + column;
	}

	bool vector_finite(const Vector3d& vector) {
		return std::all_of(vector.begin(), vector.end(), [](double value) {
			return std::isfinite(value);
		});
	}

	void validate_config(const FilterConfig& config) {
		if (!std::isfinite(config.accel_sigma) || config.accel_sigma <= 0.0 ||
			!std::isfinite(config.gps_sigma) || config.gps_sigma <= 0.0 ||
			!std::isfinite(config.gate_threshold) || config.gate_threshold <= 0.0) {
			throw std::invalid_argument("filter configuration must be positive and finite");
		}
	}
}

bool invert_symmetric_3x3(const Matrix3d& input, Matrix3d& output) {
	if (!std::all_of(input.begin(), input.end(), [](double value) { return std::isfinite(value); })) {
		return false;
	}
	const double a = input[0];
	const double b = input[1];
	const double c = input[2];
	const double d = input[4];
	const double e = input[5];
	const double f = input[8];
	const double cofactor00 = d * f - e * e;
	const double cofactor01 = c * e - b * f;
	const double cofactor02 = b * e - c * d;
	const double cofactor11 = a * f - c * c;
	const double cofactor12 = b * c - a * e;
	const double cofactor22 = a * d - b * b;
	const double determinant = a * cofactor00 + b * cofactor01 + c * cofactor02;
	if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-15) {
		return false;
	}
	const double scale = 1.0 / determinant;
	const Matrix3d candidate{
		cofactor00 * scale, cofactor01 * scale, cofactor02 * scale,
		cofactor01 * scale, cofactor11 * scale, cofactor12 * scale,
		cofactor02 * scale, cofactor12 * scale, cofactor22 * scale
	};
	if (!std::all_of(candidate.begin(), candidate.end(), [](double value) {
		return std::isfinite(value);
	})) {
		return false;
	}
	output = candidate;
	return true;
}

bool all_finite(const State6d& state, const Matrix6d& covariance) {
	return std::all_of(state.begin(), state.end(), [](double value) { return std::isfinite(value); }) &&
		std::all_of(covariance.begin(), covariance.end(), [](double value) {
			return std::isfinite(value);
		});
}

bool covariance_valid(const Matrix6d& covariance, double symmetry_tolerance) {
	if (!std::isfinite(symmetry_tolerance) || symmetry_tolerance < 0.0) {
		return false;
	}
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		if (!std::isfinite(covariance[index6(row, row)]) || covariance[index6(row, row)] < 0.0) {
			return false;
		}
		for (std::size_t column = 0; column < STATE_SIZE; ++column) {
			if (!std::isfinite(covariance[index6(row, column)]) ||
				std::abs(covariance[index6(row, column)] - covariance[index6(column, row)]) >
					symmetry_tolerance) {
				return false;
			}
		}
	}
	return true;
}

FixedKalmanFilter::FixedKalmanFilter(const Vector3d& position, const Vector3d& velocity,
	const FilterConfig& config)
	: config_(config), adaptation_(config.gps_sigma, config.gate_threshold) {
	validate_config(config_);
	if (!vector_finite(position) || !vector_finite(velocity)) {
		throw std::invalid_argument("initial filter state must be finite");
	}
	const double gps_variance = config_.gps_sigma * config_.gps_sigma;
	if (!std::isfinite(gps_variance)) {
		throw std::invalid_argument("GPS variance must be finite");
	}
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		state_[axis] = position[axis];
		state_[axis + DIMENSIONS] = velocity[axis];
		covariance_[index6(axis, axis)] = 1e-6;
		covariance_[index6(axis + DIMENSIONS, axis + DIMENSIONS)] = 1e-2;
	}
}

void FixedKalmanFilter::predict(const Vector3d& acceleration, double dt) {
	if (!vector_finite(acceleration) || !std::isfinite(dt) || dt <= 0.0) {
		throw std::invalid_argument("predict inputs must be finite and dt must be positive");
	}
	State6d candidate_state = state_;
	Matrix6d candidate_covariance{};
	const double half_dt_squared = 0.5 * dt * dt;
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		candidate_state[axis] += dt * state_[axis + DIMENSIONS] +
			half_dt_squared * acceleration[axis];
		candidate_state[axis + DIMENSIONS] += dt * acceleration[axis];
	}
	const double acceleration_variance = config_.accel_sigma * config_.accel_sigma;
	for (std::size_t row = 0; row < DIMENSIONS; ++row) {
		for (std::size_t column = 0; column < DIMENSIONS; ++column) {
			const double diagonal = row == column ? acceleration_variance : 0.0;
			candidate_covariance[index6(row, column)] = covariance_[index6(row, column)] +
				dt * (covariance_[index6(row, column + DIMENSIONS)] +
					covariance_[index6(row + DIMENSIONS, column)]) +
				dt * dt * covariance_[index6(row + DIMENSIONS, column + DIMENSIONS)] +
				half_dt_squared * half_dt_squared * diagonal;
			candidate_covariance[index6(row, column + DIMENSIONS)] =
				covariance_[index6(row, column + DIMENSIONS)] +
				dt * covariance_[index6(row + DIMENSIONS, column + DIMENSIONS)] +
				half_dt_squared * dt * diagonal;
			candidate_covariance[index6(row + DIMENSIONS, column)] =
				covariance_[index6(row + DIMENSIONS, column)] +
				dt * covariance_[index6(row + DIMENSIONS, column + DIMENSIONS)] +
				half_dt_squared * dt * diagonal;
			candidate_covariance[index6(row + DIMENSIONS, column + DIMENSIONS)] =
				covariance_[index6(row + DIMENSIONS, column + DIMENSIONS)] + dt * dt * diagonal;
		}
	}
	if (!all_finite(candidate_state, candidate_covariance) || !covariance_valid(candidate_covariance)) {
		throw std::runtime_error("predict produced an invalid filter state");
	}
	state_ = candidate_state;
	covariance_ = candidate_covariance;
}

GpsUpdateResult FixedKalmanFilter::update_gps(const Vector3d& gps) {
	if (!vector_finite(gps)) {
		throw std::invalid_argument("GPS position must be finite");
	}
	Matrix3d innovation_covariance{};
	const Vector3d measurement_variance = config_.adaptive_noise
		? adaptation_.gps_variance()
		: Vector3d{config_.gps_sigma * config_.gps_sigma,
			config_.gps_sigma * config_.gps_sigma, config_.gps_sigma * config_.gps_sigma};
	Vector3d predicted_position_variance{};
	for (std::size_t row = 0; row < DIMENSIONS; ++row) {
		predicted_position_variance[row] = covariance_[index6(row, row)];
		for (std::size_t column = 0; column < DIMENSIONS; ++column) {
			innovation_covariance[index3(row, column)] = covariance_[index6(row, column)];
		}
		innovation_covariance[index3(row, row)] += measurement_variance[row];
	}
	Matrix3d inverse_innovation{};
	if (!invert_symmetric_3x3(innovation_covariance, inverse_innovation)) {
		throw std::runtime_error("GPS innovation covariance is singular");
	}
	std::array<double, 18> gain{};
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t column = 0; column < DIMENSIONS; ++column) {
			for (std::size_t inner = 0; inner < DIMENSIONS; ++inner) {
				gain[row * DIMENSIONS + column] += covariance_[index6(row, inner)] *
					inverse_innovation[index3(inner, column)];
			}
		}
	}
	GpsUpdateResult result;
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		result.innovation[axis] = gps[axis] - state_[axis];
	}
	for (std::size_t row = 0; row < DIMENSIONS; ++row) {
		for (std::size_t column = 0; column < DIMENSIONS; ++column) {
			result.mahalanobis_squared += result.innovation[row] *
				inverse_innovation[index3(row, column)] * result.innovation[column];
		}
	}
	if (config_.innovation_gating &&
		result.mahalanobis_squared > adaptation_.current_gate_threshold()) {
		adaptation_.record_rejection();
		return result;
	}
	result.accepted = true;
	State6d candidate_state = state_;
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
			candidate_state[row] += gain[row * DIMENSIONS + axis] * result.innovation[axis];
		}
	}
	Matrix6d correction{};
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t column = 0; column < STATE_SIZE; ++column) {
			correction[index6(row, column)] = (row == column ? 1.0 : 0.0) -
				(column < DIMENSIONS ? gain[row * DIMENSIONS + column] : 0.0);
		}
	}
	Matrix6d left_product{};
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t column = 0; column < STATE_SIZE; ++column) {
			for (std::size_t inner = 0; inner < STATE_SIZE; ++inner) {
				left_product[index6(row, column)] += correction[index6(row, inner)] *
					covariance_[index6(inner, column)];
			}
		}
	}
	Matrix6d candidate_covariance{};
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t column = 0; column < STATE_SIZE; ++column) {
			for (std::size_t inner = 0; inner < STATE_SIZE; ++inner) {
				candidate_covariance[index6(row, column)] += left_product[index6(row, inner)] *
					correction[index6(column, inner)];
			}
			for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
				candidate_covariance[index6(row, column)] += gain[row * DIMENSIONS + axis] *
					measurement_variance[axis] * gain[column * DIMENSIONS + axis];
			}
		}
	}
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t column = row + 1; column < STATE_SIZE; ++column) {
			const double symmetric = 0.5 * (candidate_covariance[index6(row, column)] +
				candidate_covariance[index6(column, row)]);
			candidate_covariance[index6(row, column)] = symmetric;
			candidate_covariance[index6(column, row)] = symmetric;
		}
	}
	if (!all_finite(candidate_state, candidate_covariance) || !covariance_valid(candidate_covariance)) {
		throw std::runtime_error("GPS update produced an invalid filter state");
	}
	state_ = candidate_state;
	covariance_ = candidate_covariance;
	adaptation_.record_acceptance(result.innovation, predicted_position_variance);
	return result;
}

FilterSnapshot FixedKalmanFilter::snapshot() const {
	FilterSnapshot snapshot;
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		snapshot.position[axis] = state_[axis];
		snapshot.velocity[axis] = state_[axis + DIMENSIONS];
		snapshot.position_variance[axis] = covariance_[index6(axis, axis)];
		snapshot.gps_variance[axis] = config_.adaptive_noise
			? adaptation_.gps_variance()[axis]
			: config_.gps_sigma * config_.gps_sigma;
	}
	return snapshot;
}

bool FixedKalmanFilter::invariants_hold() const {
	return all_finite(state_, covariance_) && covariance_valid(covariance_);
}

std::uint64_t FixedKalmanFilter::accepted_gps_count() const {
	return adaptation_.accepted_count();
}

std::uint64_t FixedKalmanFilter::rejected_gps_count() const {
	return adaptation_.rejected_count();
}
