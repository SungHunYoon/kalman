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
			!std::isfinite(config.gate_threshold) || config.gate_threshold <= 0.0 ||
			!std::isfinite(config.direction_sigma) || config.direction_sigma <= 0.0) {
			throw std::invalid_argument("filter configuration must be positive and finite");
		}
	}

	double project_velocity(const Vector3d& normal, const State6d& state) {
		double result = 0.0;
		for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
			result += normal[axis] * state[axis + DIMENSIONS];
		}
		return result;
	}

	double projected_covariance(const Vector3d& left, const Matrix6d& covariance,
		const Vector3d& right) {
		double result = 0.0;
		for (std::size_t row = 0; row < DIMENSIONS; ++row) {
			for (std::size_t column = 0; column < DIMENSIONS; ++column) {
				result += left[row] * covariance[index6(row + DIMENSIONS,
					column + DIMENSIONS)] * right[column];
			}
		}
		return result;
	}

	bool scalar_direction_update(const Vector3d& normal, double variance,
		State6d& state, Matrix6d& covariance) {
		const double innovation = -project_velocity(normal, state);
		const double innovation_variance =
			projected_covariance(normal, covariance, normal) + variance;
		if (!std::isfinite(innovation) || !std::isfinite(innovation_variance) ||
			innovation_variance <= 0.0) {
			return false;
		}
		std::array<double, STATE_SIZE> gain{};
		for (std::size_t row = 0; row < STATE_SIZE; ++row) {
			for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
				gain[row] += covariance[index6(row, axis + DIMENSIONS)] * normal[axis];
			}
			gain[row] /= innovation_variance;
		}
		State6d candidate_state = state;
		for (std::size_t row = 0; row < STATE_SIZE; ++row) {
			candidate_state[row] += gain[row] * innovation;
		}
		Matrix6d correction{};
		for (std::size_t row = 0; row < STATE_SIZE; ++row) {
			for (std::size_t column = 0; column < STATE_SIZE; ++column) {
				correction[index6(row, column)] = (row == column ? 1.0 : 0.0) -
					(column >= DIMENSIONS ? gain[row] * normal[column - DIMENSIONS] : 0.0);
			}
		}
		Matrix6d left_product{};
		for (std::size_t row = 0; row < STATE_SIZE; ++row) {
			for (std::size_t column = 0; column < STATE_SIZE; ++column) {
				for (std::size_t inner = 0; inner < STATE_SIZE; ++inner) {
					left_product[index6(row, column)] += correction[index6(row, inner)] *
						covariance[index6(inner, column)];
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
				candidate_covariance[index6(row, column)] += gain[row] * variance * gain[column];
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
		if (!all_finite(candidate_state, candidate_covariance) ||
			!covariance_valid(candidate_covariance)) {
			return false;
		}
		state = candidate_state;
		covariance = candidate_covariance;
		return true;
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

bool FixedKalmanFilter::update_direction(const Vector3d& direction) {
	if (!vector_finite(direction)) {
		throw std::invalid_argument("direction must be finite");
	}
	const double speed = std::hypot(state_[3], state_[4], state_[5]);
	if (!std::isfinite(speed) || speed < 1.0) {
		return false;
	}
	const double scaled_sigma = speed * config_.direction_sigma;
	const double variance = scaled_sigma * scaled_sigma;
	if (!std::isfinite(variance) || variance <= 0.0) {
		return false;
	}
	const double pitch = direction[1];
	const double yaw = direction[2];
	const Vector3d raw_yaw_normal{-std::sin(yaw), std::cos(yaw), 0.0};
	const Vector3d raw_pitch_normal{std::sin(pitch) * std::cos(yaw),
		std::sin(pitch) * std::sin(yaw), std::cos(pitch)};
	if (!vector_finite(raw_yaw_normal) || !vector_finite(raw_pitch_normal)) {
		return false;
	}
	const double s00 = projected_covariance(raw_yaw_normal, covariance_, raw_yaw_normal) + variance;
	const double s01 = projected_covariance(raw_yaw_normal, covariance_, raw_pitch_normal);
	const double s11 = projected_covariance(raw_pitch_normal, covariance_, raw_pitch_normal) + variance;
	const double determinant = s00 * s11 - s01 * s01;
	if (!std::isfinite(s00) || !std::isfinite(s01) || !std::isfinite(s11) ||
		s00 <= 0.0 || s11 <= 0.0 || !std::isfinite(determinant) || determinant <= 0.0) {
		return false;
	}
	const double r0 = -project_velocity(raw_yaw_normal, state_);
	const double r1 = -project_velocity(raw_pitch_normal, state_);
	const double distance_squared =
		(s11 * r0 * r0 - 2.0 * s01 * r0 * r1 + s00 * r1 * r1) / determinant;
	if (!std::isfinite(distance_squared) || distance_squared < 0.0 ||
		distance_squared > 9.21) {
		return false;
	}
	// Average unit directions so independent attitude noise does not repeatedly
	// turn into a longitudinal zero-velocity constraint.
	auto candidate_samples = direction_samples_;
	candidate_samples[next_direction_sample_] = Vector3d{
		std::cos(yaw) * std::cos(pitch),
		std::sin(yaw) * std::cos(pitch), -std::sin(pitch)};
	const std::size_t candidate_count = std::min(direction_sample_count_ + 1,
		DIRECTION_WINDOW);
	Vector3d sum{};
	for (std::size_t index = 0; index < candidate_count; ++index) {
		for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
			sum[axis] += candidate_samples[index][axis];
		}
	}
	const double horizontal = std::hypot(sum[0], sum[1]);
	const double length = std::hypot(horizontal, sum[2]);
	if (!vector_finite(sum) || !std::isfinite(length) || length <= 1e-12) {
		return false;
	}
	const double mean_yaw = std::atan2(sum[1], sum[0]);
	const double mean_pitch = std::atan2(-sum[2], horizontal);
	const Vector3d yaw_normal{-std::sin(mean_yaw), std::cos(mean_yaw), 0.0};
	const Vector3d pitch_normal{std::sin(mean_pitch) * std::cos(mean_yaw),
		std::sin(mean_pitch) * std::sin(mean_yaw), std::cos(mean_pitch)};
	State6d candidate_state = state_;
	Matrix6d candidate_covariance = covariance_;
	if (!scalar_direction_update(yaw_normal, variance, candidate_state, candidate_covariance) ||
		!scalar_direction_update(pitch_normal, variance, candidate_state, candidate_covariance)) {
		return false;
	}
	// Heading is not a speed measurement. The linearized transverse constraints
	// otherwise shrink forward speed when noisy headings are applied repeatedly.
	const double updated_speed = std::hypot(candidate_state[3], candidate_state[4],
		candidate_state[5]);
	if (!std::isfinite(updated_speed) || updated_speed <= 0.0) {
		return false;
	}
	const double speed_scale = speed / updated_speed;
	for (std::size_t axis = DIMENSIONS; axis < STATE_SIZE; ++axis) {
		candidate_state[axis] *= speed_scale;
	}
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t column = 0; column < STATE_SIZE; ++column) {
			candidate_covariance[index6(row, column)] *=
				(row >= DIMENSIONS ? speed_scale : 1.0) *
				(column >= DIMENSIONS ? speed_scale : 1.0);
		}
	}
	if (!all_finite(candidate_state, candidate_covariance) ||
		!covariance_valid(candidate_covariance)) {
		return false;
	}
	state_ = candidate_state;
	covariance_ = candidate_covariance;
	direction_samples_ = candidate_samples;
	direction_sample_count_ = candidate_count;
	next_direction_sample_ = (next_direction_sample_ + 1) % DIRECTION_WINDOW;
	return true;
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
