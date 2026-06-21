#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include "math_util.hpp"

template <typename K>
class Vector {
	private:
		std::vector<K> vec;

	public:
		Vector(const std::size_t size) : vec(size) {}
		Vector(const std::size_t size, const K &value) : vec(size, value) {}
		Vector(const std::vector<K> values) : vec(values) {}
		Vector(std::initializer_list<K> values) : vec(values) {}
		Vector& operator=(const Vector<K> &rhs) {
			if (this == &rhs) return *this;
			vec = rhs.vec;
			return *this;
		}
		Vector(const Vector<K> &rhs) : vec(rhs.vec) {}
		~Vector() {}

		std::size_t size(void) const {
			return vec.size();
		}

		void add(const Vector<K> &rhs) {
			if (this->size() != rhs.vec.size()) throw std::runtime_error("[add] Different vector size\n");
			for (std::size_t i = 0; i < vec.size(); i++) {
				vec[i] += rhs.vec[i];
			}
		}

		void sub(const Vector<K> &rhs) {
			if (this->size() != rhs.vec.size()) throw std::runtime_error("[sub] Different vector size\n");
			for (std::size_t i = 0; i < vec.size(); i++) {
				vec[i] -= rhs.vec[i];
			}
		}

		void scl(const K &scalar) {
			for (std::size_t i = 0; i < vec.size(); i++) {
				vec[i] *= scalar;
			}
		}

		K dot(const Vector<K> &rhs) const {
			if (this->size() != rhs.size()) throw std::runtime_error("[dot] Different vector size\n");
			K ret{};
			for (std::size_t i = 0; i < vec.size(); i++) {
				ret = mul_add(vec[i], rhs[i], ret);
			}
			return ret;
		}

		K norm_1(void) const {
			K ret{};
			for (std::size_t i = 0; i < vec.size(); i++) {
				ret += std::abs(vec[i]);
			}
			return ret;
		}

		K norm(void) const {
			K ret{};
			for (std::size_t i = 0; i < vec.size(); i++) {
				ret += vec[i] * vec[i];
			}
			return std::sqrt(ret);
		}

		K norm_inf(void) const {
			K ret{};
			for (std::size_t i = 0; i < vec.size(); i++) {
				ret = std::max(ret, static_cast<K>(std::abs(vec[i])));
			}
			return ret;
		}


		K& operator[](std::size_t idx) {
			if (idx >= vec.size()) throw std::out_of_range("[operator[]] Invalid vector index\n");
			return vec[idx];
		}

		const K& operator[](std::size_t idx) const {
			if (idx >= vec.size()) throw std::out_of_range("[operator[]] Invalid vector index\n");
			return vec[idx];
		}

		Vector<K> operator+(const Vector<K> &rhs) const {
			if (this->size() != rhs.size()) throw std::runtime_error("[operator+] Different vector size\n");
			Vector<K> ret(this->size());
			for (std::size_t i = 0; i < ret.size(); i++) {
				ret[i] = vec[i] + rhs[i];
			}
			return ret;
		}

		Vector<K> operator-(const Vector<K> &rhs) const {
			if (this->size() != rhs.size()) throw std::runtime_error("[operator-] Different vector size\n");
			Vector<K> ret(this->size());
			for (std::size_t i = 0; i < ret.size(); i++) {
				ret[i] = vec[i] - rhs[i];
			}
			return ret;
		}

		Vector<K> operator*(const K &rhs) const {
			Vector<K> ret(this->size());
			for (std::size_t i = 0; i < ret.size(); i++) {
				ret[i] = vec[i] * rhs;
			}
			return ret;
		}
};

template <typename K>
std::ostream& operator<<(std::ostream &out, const Vector<K> &rhs) {
	out << "[";
	for (std::size_t i = 0; i < rhs.size(); i++) {
		out << rhs[i];
		if (i + 1 < rhs.size()) {
			out << ", ";
		}
	}
	out << "]";
	return out;
}
