#pragma once

#include <vector>
#include <stdexcept>
#include "vector.hpp"

template <typename K>
Vector<K> linear_combination(const std::vector<Vector<K>> &vec, const std::vector<K> &scalar) {
	if (vec.empty()) {
		return Vector<K>(0, K{});
	}
	if (vec.size() != scalar.size()) throw std::runtime_error("[linear_combination] Different vector and scalar size\n");
	std::size_t n = vec[0].size();
	for (std::size_t i = 0; i < vec.size(); i++) {
		if (vec[i].size() != n) throw std::runtime_error("[linear_combination] Different vector size\n");
	}
	Vector<K> ret(n);
	for (std::size_t i = 0; i < vec.size(); i++) {
		for (std::size_t j = 0; j < n; j++) {
			ret[j] = mul_add(vec[i][j], scalar[i], ret[j]);
		}
	}
	return ret;
}

template <typename K>
K lerp(const K &u, const K &v, const float &t) {
	return u + (v - u) * t;
}

template <typename K>
K angle_cos(const Vector<K> u, const Vector<K> v) {
	K norm_u = u.norm();
	K norm_v = v.norm();

	if (norm_u == K{} || norm_v == K{}) throw std::runtime_error("[angle_cos] One of the vector norm is zero\n");

	return u.dot(v) / (u.norm() * v.norm());
}

template <typename K>
Vector<K> cross_product(const Vector<K> &u, const Vector<K> &v) {
	if (u.size() != 3 || v.size() != 3) throw std::runtime_error("[cross_product] One of the vector is not 3 dimension\n");
	Vector<K> ret(u.size());
	for (std::size_t i = 0; i < ret.size(); i++) {
		int idx1 = (i + 1) % ret.size();
		int idx2 = (i + 2) % ret.size();
		ret[i] = u[idx1] * v[idx2];
		ret[i] -= u[idx2] * v[idx1];
	}
	return ret;
}

template <typename K>
Matrix<K> projection(K fov, K ratio, K near, K far) {
	K f = 1.0f / std::tan(fov / 2.0f);

	return Matrix<K>{
		{f / ratio, 0.0f, 0.0f, 0.0f},
		{0.0f, f, 0.0f, 0.0f},
		{0.0f, 0.0f, far / (near - far), -1.0f},
		{0.0f, 0.0f, (far * near) / (near - far), 0.0f}
	};
}
