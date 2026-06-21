#pragma once

#include <cmath>

template <typename K>
K mul_add(const K &a, const K &b, const K &c) {
	return a * b + c;
}

inline float mul_add(const float &a, const float &b, const float &c) {
	return std::fma(a, b, c);
}

inline double mul_add(const double &a, const double &b, const double &c) {
	return std::fma(a, b, c);
}
