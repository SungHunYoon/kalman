#pragma once

#include <vector>
#include <utility>
#include <complex>
#include <stdexcept>
#include <iostream>
#include "vector.hpp"

template <typename K>
class Matrix {
	private:
		std::size_t rows;
		std::size_t cols;
		std::vector<K> mat;

	public:
		Matrix(const std::size_t rows, const std::size_t cols) : rows(rows), cols(cols), mat(rows * cols) {}
		Matrix(const std::size_t rows, const std::size_t cols, const K &value) : rows(rows), cols(cols), mat(rows * cols, value) {}
		Matrix(std::initializer_list<std::initializer_list<K>> values) {
			rows = values.size();
			cols = values.begin()->size();
			mat.reserve(rows * cols);
			for (const auto &row : values) {
				if (cols != row.size()) throw std::runtime_error("[Constructor] Invalid matrix\n");
				for (const auto &col : row) {
					mat.push_back(col);
				}
			}
		}
		Matrix& operator=(const Matrix<K> &rhs) {
			if (this == &rhs) return *this;
			rows = rhs.rows;
			cols = rhs.cols;
			mat = rhs.mat;
			return *this;
		}
		Matrix(const Matrix<K> &rhs) : rows(rhs.rows), cols(rhs.cols), mat(rhs.mat) {}
		~Matrix() {}

		std::pair<std::size_t, std::size_t> size(void) const {
			return {rows, cols};
		}

		void add(const Matrix &rhs) {
			auto [rhsRows, rhsCols] = rhs.size();
			if (rows != rhsRows || cols != rhsCols) throw std::runtime_error("[add] Different matrix size\n");
			for (std::size_t i = 0; i < mat.size(); i++) {
				mat[i] += rhs.mat[i];
			}
		}

		void sub(const Matrix &rhs) {
			auto [rhsRows, rhsCols] = rhs.size();
			if (rows != rhsRows || cols != rhsCols) throw std::runtime_error("[sub] Different matrix size\n");
			for (std::size_t i = 0; i < mat.size(); i++) {
				mat[i] -= rhs.mat[i];
			}
		}

		void scl(const K &scalar) {
			for (std::size_t i = 0; i < mat.size(); i++) {
				mat[i] *= scalar;
			}
		}

		Vector<K> mul_vec(const Vector<K> &rhs) const {
			if (cols != rhs.size()) throw std::runtime_error("[mul_vec] Different matrix cols and vector size\n");
			Vector<K> ret(rows);
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < rhs.size(); j++) {
					ret[i] = mul_add(mat[i * cols + j], rhs[j], ret[i]);
				}
			}
			return ret;
		}

		Matrix<K> mul_mat(const Matrix<K> &rhs) const {
			auto [rhsRows, rhsCols] = rhs.size();
			if (cols != rhsRows) throw std::runtime_error("[mul_mat] Different matrix cols and matrix rows\n");
			Matrix<K> ret(rows, rhsCols);
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < rhsCols; j++) {
					for (std::size_t k = 0; k < cols; k++) {
						ret(i, j) = mul_add(mat[i * cols + k], rhs(k, j), ret(i, j));
					}
				}
			}
			return ret;
		}

		K trace(void) const {
			if (rows != cols) throw std::runtime_error("[trace] Matrix not sqarue\n");
			K ret{};
			for (std::size_t i = 0; i < rows; i++) {
				ret += mat[i * cols + i];
			}
			return ret;
		}

		Matrix<K> transpose(void) const {
			Matrix<K> ret(cols, rows);
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < cols; j++) {
					ret(j, i) = mat[i * cols + j];
				}
			}
			return ret;
		}

		Matrix<K> row_echelon(void) const {
			Matrix<K> ret(*this);

			const double epsilon = 1e-6;
			std::size_t pivot_row = 0;
			for (std::size_t col = 0; col < cols && pivot_row < rows; col++) {
				std::size_t pivot = pivot_row;
				while (pivot < rows && std::abs(ret(pivot, col)) < epsilon) {
					pivot++;
				}
				if (pivot == rows) {
					continue;
				}
				if (pivot != pivot_row) {
					for (std::size_t i = 0; i < cols; i++) {
						std::swap(ret(pivot, i), ret(pivot_row, i));
					}
				}
				K pivot_value = ret(pivot_row, col);
				for (std::size_t i = 0; i < cols; i++) {
					ret(pivot_row, i) /= pivot_value;
				}
				for (std::size_t i = 0; i < rows; i++) {
					if (i == pivot_row) continue;
					K factor = ret(i, col);
					if (std::abs(factor) < epsilon) continue;
					for (std::size_t j = 0; j < cols; j++) {
						ret(i, j) -= factor * ret(pivot_row, j);
					}
				}
				pivot_row++;
			}
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < cols; j++) {
					if (std::abs(ret(i, j)) < epsilon) {
						ret(i, j) = K{};
					}
				}
			}
			return ret;
		}

		K determinant(void) const {
			if (rows != cols) throw std::runtime_error("[determinant] Matrix is not square\n");

			Matrix<K> tmp(*this);
			const double epsilon = 1e-6;
			K det = static_cast<K>(1);
			for (std::size_t col = 0; col < rows; col++) {
				std::size_t pivot = col;
				for (std::size_t i = col + 1; i < rows; i++) {
					if (std::abs(tmp(i, col)) > std::abs(tmp(pivot, col))) {
						pivot = i;
					}
				}
				if (std::abs(tmp(pivot, col)) < epsilon) {
					return K{};
				}
				if (pivot != col) {
					for (std::size_t i = 0; i < cols; i++) {
						std::swap(tmp(col, i), tmp(pivot, i));
					}
					det = -det;
				}
				K pivot_value = tmp(col, col);
				det *= pivot_value;
				for (std::size_t i = col + 1; i < rows; i++) {
					K factor = tmp(i, col) / pivot_value;
					for (std::size_t j = col; j < cols; j++) {
						tmp(i, j) -= factor * tmp(col, j);
					}
				}
			}
			return det;
		}

		Matrix<K> inverse(void) const {
			if (rows != cols) throw std::runtime_error("[inverse] Matrix is not sqaure\n");
			const double epsilon = 1e-6;
			Matrix<K> left(*this), right(rows, cols);
			for (std::size_t i = 0; i < rows; i++) {
				right(i, i) = static_cast<K>(1);
			}
			std::size_t pivot_row = 0;
			for (std::size_t col = 0; col < cols && pivot_row < rows; col++) {
				std::size_t pivot = col;
				for (std::size_t i = pivot_row + 1; i < rows; i++) {
					if (std::abs(left(i, col)) > std::abs(left(pivot, col))) {
						pivot = i;
					}
				}
				if (std::abs(left(pivot, col)) < epsilon) continue;
				if (pivot != pivot_row) {
					for (std::size_t i = 0; i < cols; i++) {
						std::swap(left(pivot, i), left(pivot_row, i));
						std::swap(right(pivot, i), right(pivot_row, i));
					}
				}
				K pivot_value = left(pivot_row, col);
				for (std::size_t i = 0; i < cols; i++) {
					left(pivot_row, i) /= pivot_value;
					right(pivot_row, i) /= pivot_value;
				}
				for (std::size_t i = 0; i < rows; i++) {
					if (i == pivot_row) continue;
					K factor = left(i, col);
					if (std::abs(factor) < epsilon) continue;
					for (std::size_t j = 0; j < cols; j++) {
						left(i, j) -= factor * left(pivot_row, j);
						right(i, j) -= factor * right(pivot_row, j);
					}
				}
				pivot_row++;
			}
			if (pivot_row != rows) throw std::runtime_error("[inverse] Determinant is zero\n");
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < cols; j++) {
					if (std::abs(right(i, j)) < epsilon) {
						right(i, j) = K{};
					}
				}
			}
			return right;
		}

		std::size_t rank(void) {
			Matrix<K> rref = this->row_echelon();
			const double epsilon = 1e-6;
			std::size_t rank = 0;
			for (std::size_t i = 0; i < rows; i++) {
				bool isZeroRow = true;
				for (std::size_t j = 0; j < cols; j++) {
					if (std::abs(rref(i, j)) >= epsilon) {
						isZeroRow = false;
					}
				}
				if (!isZeroRow) rank++;
			}
			return rank;
		}


		K& operator()(std::size_t row, std::size_t col) {
			if (row >= rows || col >= cols) throw std::out_of_range("[operator()] Invalid matrix index\n");
			return mat[row * cols + col];
		}

		const K& operator()(std::size_t row, std::size_t col) const {
			if (row >= rows || col >= cols) throw std::out_of_range("[operator()] Invalid matrix index\n");
			return mat[row * cols + col];
		}

		Matrix<K> operator+(const Matrix<K> &rhs) const {
			if (this->size() != rhs.size()) throw std::runtime_error("[operator+] Different matrix size\n");
			auto [rhsRows, rhsCols] = rhs.size();
			Matrix<K> ret(rhsRows, rhsCols);
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < cols; j++) {
					ret(i, j) = mat[i * cols + j] + rhs(i, j);
				}
			}
			return ret;
		}

		Matrix<K> operator-(const Matrix<K> &rhs) const {
			if (this->size() != rhs.size()) throw std::runtime_error("[operator-] Different matrix size\n");
			auto [rhsRows, rhsCols] = rhs.size();
			Matrix<K> ret(rhsRows, rhsCols);
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < cols; j++) {
					ret(i, j) = mat[i * cols + j] - rhs(i, j);
				}
			}
			return ret;
		}

		Matrix<K> operator*(const K &rhs) const {
			Matrix<K> ret(rows, cols);
			for (std::size_t i = 0; i < rows; i++) {
				for (std::size_t j = 0; j < cols; j++) {
					ret(i, j) = mat[i * cols + j] * rhs;
				}
			}
			return ret;
		}
};

template <typename K>
std::ostream& operator<<(std::ostream &out, const Matrix<K> &rhs) {
	auto [rows, cols] = rhs.size();
	out << "[";
	for (std::size_t i = 0; i < rows; i++) {
		out << "[";
		for (std::size_t j = 0; j < cols; j++) {
			out << rhs(i, j);
			if (j + 1 < cols) {
				out << ", ";
			}
		}
		out << "]";
		if (i + 1 < rows) {
			out << "\n";
		}
	}
	out << "]";
	return out;
}
