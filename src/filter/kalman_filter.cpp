#include "filter/kalman_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace {
	const std::size_t DIMENSIONS = 3;
	const std::size_t STATE_SIZE = 6;
	const double ACCELEROMETER_VARIANCE = 1e-4;
	const double GPS_VARIANCE = 1.0;

	Matrix<double> identity(std::size_t size) {
		Matrix<double> result(size, size, 0.0);
		for (std::size_t i = 0; i < size; ++i) {
			result(i, i) = 1.0;
		}
		return result;
	}
}

KalmanFilter::KalmanFilter(const Vector<double>& initial_position,
						   const Vector<double>& initial_velocity)
	: state(STATE_SIZE, 0.0), covariance(STATE_SIZE, STATE_SIZE, 0.0) {
	if (initial_position.size() != DIMENSIONS || initial_velocity.size() != DIMENSIONS) {
		throw std::invalid_argument("[KalmanFilter] position and velocity must have 3 elements");
	}
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		state[axis] = initial_position[axis];
		state[axis + DIMENSIONS] = initial_velocity[axis];
		covariance(axis, axis) = 1e-6;
		covariance(axis + DIMENSIONS, axis + DIMENSIONS) = 1e-2;
	}
}

void KalmanFilter::predict(const Vector<double>& acceleration, double dt) {
	if (acceleration.size() != DIMENSIONS) {
		throw std::invalid_argument("[KalmanFilter::predict] acceleration must have 3 elements");
	}
	if (!std::isfinite(dt) || dt <= 0.0) {
		throw std::invalid_argument("[KalmanFilter::predict] dt must be positive and finite");
	}

	Matrix<double> transition = identity(STATE_SIZE);
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		transition(axis, axis + DIMENSIONS) = dt;
	}

	state = transition.mul_vec(state);
	const double half_dt_squared = 0.5 * dt * dt;
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		state[axis] += half_dt_squared * acceleration[axis];
		state[axis + DIMENSIONS] += dt * acceleration[axis];
	}

	Matrix<double> process_noise(STATE_SIZE, STATE_SIZE, 0.0);
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		process_noise(axis, axis) = half_dt_squared * half_dt_squared * ACCELEROMETER_VARIANCE;
		process_noise(axis, axis + DIMENSIONS) = half_dt_squared * dt * ACCELEROMETER_VARIANCE;
		process_noise(axis + DIMENSIONS, axis) = process_noise(axis, axis + DIMENSIONS);
		process_noise(axis + DIMENSIONS, axis + DIMENSIONS) = dt * dt * ACCELEROMETER_VARIANCE;
	}
	covariance = transition.mul_mat(covariance).mul_mat(transition.transpose()) + process_noise;
}

void KalmanFilter::update_gps(const Vector<double>& gps_position) {
	if (gps_position.size() != DIMENSIONS) {
		throw std::invalid_argument("[KalmanFilter::update_gps] position must have 3 elements");
	}

	Matrix<double> innovation_covariance(DIMENSIONS, DIMENSIONS, 0.0);
	for (std::size_t row = 0; row < DIMENSIONS; ++row) {
		for (std::size_t col = 0; col < DIMENSIONS; ++col) {
			innovation_covariance(row, col) = covariance(row, col);
		}
		innovation_covariance(row, row) += GPS_VARIANCE;
	}

	const Matrix<double> inverse_innovation = innovation_covariance.inverse();
	Matrix<double> gain(STATE_SIZE, DIMENSIONS, 0.0);
	for (std::size_t row = 0; row < STATE_SIZE; ++row) {
		for (std::size_t col = 0; col < DIMENSIONS; ++col) {
			for (std::size_t inner = 0; inner < DIMENSIONS; ++inner) {
				gain(row, col) += covariance(row, inner) * inverse_innovation(inner, col);
			}
		}
	}

	Vector<double> innovation(DIMENSIONS, 0.0);
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		innovation[axis] = gps_position[axis] - state[axis];
	}
	state = state + gain.mul_vec(innovation);

	Matrix<double> observation(DIMENSIONS, STATE_SIZE, 0.0);
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		observation(axis, axis) = 1.0;
	}
	const Matrix<double> correction = identity(STATE_SIZE) - gain.mul_mat(observation);
	Matrix<double> measurement_noise(DIMENSIONS, DIMENSIONS, 0.0);
	for (std::size_t axis = 0; axis < DIMENSIONS; ++axis) {
		measurement_noise(axis, axis) = GPS_VARIANCE;
	}
	covariance = correction.mul_mat(covariance).mul_mat(correction.transpose()) +
		gain.mul_mat(measurement_noise).mul_mat(gain.transpose());
}

Vector<double> KalmanFilter::position() const {
	return Vector<double>{state[0], state[1], state[2]};
}

Vector<double> KalmanFilter::velocity() const {
	return Vector<double>{state[3], state[4], state[5]};
}

const Matrix<double>& KalmanFilter::state_covariance() const {
	return covariance;
}
