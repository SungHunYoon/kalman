#include "filter/fixed_kalman_filter.hpp"
#include "filter/kalman_filter.hpp"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <new>
#include <stdexcept>

namespace {
	std::atomic<std::size_t> allocation_count{0};
	std::atomic<bool> count_allocations{false};

	void require_close(double actual, double expected, double tolerance, const char* message) {
		if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
			throw std::runtime_error(message);
		}
	}
}

void* operator new(std::size_t size) {
	if (count_allocations.load(std::memory_order_relaxed)) {
		allocation_count.fetch_add(1, std::memory_order_relaxed);
	}
	if (void* pointer = std::malloc(size)) {
		return pointer;
	}
	throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
	return ::operator new(size);
}

void operator delete(void* pointer) noexcept {
	std::free(pointer);
}

void operator delete[](void* pointer) noexcept {
	std::free(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept {
	std::free(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept {
	std::free(pointer);
}

void run_fixed_kalman_filter_tests() {
	const FilterConfig config{1e-2, 1.0, 1e300};
	FixedKalmanFilter filter({0, 0, 0}, {1, 0, 0}, config);
	filter.predict({2, 0, 0}, 0.5);
	require_close(filter.snapshot().position[0], 0.75, 1e-12, "fixed predict position mismatch");
	require_close(filter.snapshot().velocity[0], 2.0, 1e-12, "fixed predict velocity mismatch");

	const FilterSnapshot before = filter.snapshot();
	bool threw = false;
	try {
		filter.predict({0, 0, 0}, std::numeric_limits<double>::infinity());
	} catch (const std::invalid_argument&) {
		threw = true;
	}
	if (!threw || filter.snapshot().position != before.position) {
		throw std::runtime_error("invalid predict mutated state");
	}

	Matrix3d singular{1, 2, 3, 2, 4, 6, 3, 6, 9};
	Matrix3d inverse{11, 12, 13, 14, 15, 16, 17, 18, 19};
	const Matrix3d unchanged = inverse;
	if (invert_symmetric_3x3(singular, inverse) || inverse != unchanged) {
		throw std::runtime_error("singular inverse changed output");
	}

	KalmanFilter legacy(Vector<double>{0, 0, 0}, Vector<double>{1, 2, 3});
	FixedKalmanFilter fixed({0, 0, 0}, {1, 2, 3}, config);
	for (int step = 1; step <= 1000; ++step) {
		const Vector3d acceleration{0.01, -0.02, 0.03};
		legacy.predict(Vector<double>{0.01, -0.02, 0.03}, 0.01);
		fixed.predict(acceleration, 0.01);
		if (step % 300 == 0) {
			const Vector3d gps{step * 0.011, step * 0.019, step * 0.031};
			legacy.update_gps(Vector<double>{gps[0], gps[1], gps[2]});
			fixed.update_gps(gps);
		}
	}
	const FilterSnapshot snapshot = fixed.snapshot();
	for (std::size_t axis = 0; axis < 3; ++axis) {
		require_close(snapshot.position[axis], legacy.position()[axis], 1e-9,
			"fixed position differs from legacy filter");
		require_close(snapshot.velocity[axis], legacy.velocity()[axis], 1e-9,
			"fixed velocity differs from legacy filter");
	}
	if (!fixed.invariants_hold()) {
		throw std::runtime_error("fixed filter invariants failed");
	}

	FixedKalmanFilter allocation_filter({0, 0, 0}, {0, 0, 0}, config);
	const std::size_t allocations_before = allocation_count.load(std::memory_order_relaxed);
	count_allocations.store(true, std::memory_order_relaxed);
	for (int step = 0; step < 100000; ++step) {
		allocation_filter.predict({0.01, 0.02, 0.03}, 0.01);
	}
	count_allocations.store(false, std::memory_order_relaxed);
	if (allocation_count.load(std::memory_order_relaxed) != allocations_before) {
		throw std::runtime_error("fixed predict allocated memory");
	}
}
