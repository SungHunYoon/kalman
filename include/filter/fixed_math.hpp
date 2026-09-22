#pragma once

#include <array>

using Vector3d = std::array<double, 3>;
using State6d = std::array<double, 6>;
using Matrix3d = std::array<double, 9>;
using Matrix6d = std::array<double, 36>;

bool invert_symmetric_3x3(const Matrix3d& input, Matrix3d& output);
bool all_finite(const State6d& state, const Matrix6d& covariance);
bool covariance_valid(const Matrix6d& covariance, double symmetry_tolerance = 1e-9);
