#pragma once

#include "linear_algebra/matrix.hpp"
#include "linear_algebra/vector.hpp"

class KalmanFilter {
	private:
		Vector<double> state;
		Matrix<double> covariance;

	public:
		KalmanFilter(const Vector<double>& initial_position,
					 const Vector<double>& initial_velocity);

		void predict(const Vector<double>& acceleration, double dt);
		void update_gps(const Vector<double>& gps_position);

		Vector<double> position() const;
		Vector<double> velocity() const;
		const Matrix<double>& state_covariance() const;
};
