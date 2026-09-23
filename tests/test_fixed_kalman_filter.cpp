#include "filter/fixed_kalman_filter.hpp"

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
	const FilterConfig config{1e-2, 1.0, 1e300, false, false};
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

	FixedKalmanFilter fixed({0, 0, 0}, {1, 2, 3}, config);
	for (int step = 1; step <= 1000; ++step) {
		const Vector3d acceleration{0.01, -0.02, 0.03};
		fixed.predict(acceleration, 0.01);
		if (step % 300 == 0) {
			const Vector3d gps{step * 0.011, step * 0.019, step * 0.031};
			fixed.update_gps(gps);
		}
	}
	const FilterSnapshot snapshot = fixed.snapshot();
	const Vector3d expected_position{10.842523994563845, 18.87255498469673,
		31.412366036043331};
	const Vector3d expected_velocity{1.1342554536025147, 1.7872551636160854,
		3.2912342191654882};
	for (std::size_t axis = 0; axis < 3; ++axis) {
		require_close(snapshot.position[axis], expected_position[axis], 1e-9,
			"fixed position differs from legacy golden value");
		require_close(snapshot.velocity[axis], expected_velocity[axis], 1e-9,
			"fixed velocity differs from legacy golden value");
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

	FixedKalmanFilter aligned({0, 0, 0}, {16.0, -0.16, 0.0}, {0.1, 10.0, 11.345});
	for (int step = 0; step < 100; ++step) {
		aligned.predict({0, 0, 0}, 0.01);
		if (!aligned.update_direction({0, 0, 0})) {
			throw std::runtime_error("valid direction was rejected");
		}
	}
	if (std::abs(aligned.snapshot().velocity[1]) >= 0.03 ||
		std::abs(aligned.snapshot().position[1]) >= 0.10 || !aligned.invariants_hold()) {
		throw std::runtime_error("direction observation did not remove lateral drift");
	}
	const FilterSnapshot before_outlier = aligned.snapshot();
	if (aligned.update_direction({0, 0, 1.5}) ||
		aligned.snapshot().velocity != before_outlier.velocity ||
		aligned.snapshot().position != before_outlier.position ||
		aligned.snapshot().position_variance != before_outlier.position_variance) {
		throw std::runtime_error("direction outlier changed filter state");
	}
	FixedKalmanFilter stopped({0, 0, 0}, {0.5, 0, 0}, {0.1, 10.0, 11.345});
	if (stopped.update_direction({0, 0, 0}) || stopped.snapshot().velocity[0] != 0.5) {
		throw std::runtime_error("low-speed direction observation was applied");
	}
	const FilterSnapshot before_invalid = aligned.snapshot();
	bool invalid_threw = false;
	try {
		(void)aligned.update_direction({0, 0, std::numeric_limits<double>::infinity()});
	} catch (const std::invalid_argument&) {
		invalid_threw = true;
	}
	if (!invalid_threw || aligned.snapshot().position != before_invalid.position ||
		aligned.snapshot().velocity != before_invalid.velocity ||
		aligned.snapshot().position_variance != before_invalid.position_variance) {
		throw std::runtime_error("invalid direction mutated filter");
	}
	invalid_threw = false;
	try {
		FixedKalmanFilter bad({0, 0, 0}, {16, 0, 0},
			{0.1, 10.0, 11.345, true, true, 0.0});
		(void)bad;
	} catch (const std::invalid_argument&) {
		invalid_threw = true;
	}
	if (!invalid_threw) {
		throw std::runtime_error("zero direction sigma accepted");
	}

	FixedKalmanFilter noisy_angles({0, 0, 0}, {16, 0, 0}, {0.1, 10.0, 11.345});
	for (int step = 0; step < 200; ++step) {
		noisy_angles.predict({0, 0, 0}, 0.01);
		const double angle = step % 2 == 0 ? 0.01 : -0.01;
		(void)noisy_angles.update_direction({0, angle, angle});
	}
	if (std::abs(noisy_angles.snapshot().velocity[1]) >= 0.10 ||
		std::abs(noisy_angles.snapshot().velocity[2]) >= 0.10 ||
		!noisy_angles.invariants_hold()) {
		throw std::runtime_error("alternating direction noise biased velocity");
	}
	FixedKalmanFilter turning({0, 0, 0}, {16, 0, 0}, {0.1, 10.0, 11.345});
	for (int step = 1; step <= 100; ++step) {
		turning.predict({0, 0, 0}, 0.01);
		(void)turning.update_direction({0, 0, step * 0.001});
	}
	if (turning.snapshot().velocity[1] <= 0.20 || !turning.invariants_hold()) {
		throw std::runtime_error("direction update did not follow gradual turn");
	}
	const auto accepted_before = aligned.accepted_gps_count();
	const auto rejected_before = aligned.rejected_gps_count();
	(void)aligned.update_direction({0, 0, 0});
	if (aligned.accepted_gps_count() != accepted_before ||
		aligned.rejected_gps_count() != rejected_before) {
		throw std::runtime_error("direction changed GPS counters");
	}
	FixedKalmanFilter control({0, 0, 0}, {16, 0, 0}, {0.1, 10.0, 11.345});
	FixedKalmanFilter rejected({0, 0, 0}, {16, 0, 0}, {0.1, 10.0, 11.345});
	(void)rejected.update_direction({0, 0, 1.5});
	(void)control.update_gps({2, 0, 0});
	(void)rejected.update_gps({2, 0, 0});
	if (control.snapshot().position_variance != rejected.snapshot().position_variance ||
		control.snapshot().velocity != rejected.snapshot().velocity) {
		throw std::runtime_error("rejected direction changed covariance");
	}
	FixedKalmanFilter combined({0, 0, 0}, {16, -0.16, 0}, {0.1, 10, 11.345});
	(void)combined.update_direction({0, 0, 0});
	(void)combined.update_gps({0.16, 0, 0});
	if (combined.accepted_gps_count() != 1 || !combined.invariants_hold()) {
		throw std::runtime_error("direction then GPS update invalid");
	}
	FixedKalmanFilter direction_allocations({0, 0, 0}, {16, 0, 0},
		{0.1, 10, 11.345});
	const std::size_t allocations_before_direction =
		allocation_count.load(std::memory_order_relaxed);
	count_allocations.store(true, std::memory_order_relaxed);
	for (int step = 0; step < 100000; ++step) {
		direction_allocations.predict({0, 0, 0}, 0.01);
		(void)direction_allocations.update_direction({0, 0, 0});
	}
	count_allocations.store(false, std::memory_order_relaxed);
	if (allocation_count.load(std::memory_order_relaxed) != allocations_before_direction) {
		throw std::runtime_error("direction update allocated memory");
	}
}
