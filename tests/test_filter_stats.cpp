#include "performance/filter_stats.hpp"

#include <stdexcept>

void run_filter_stats_tests() {
	FilterStats stats(2);
	stats.record(100.0);
	stats.record(200.0);
	if (stats.snapshot().count != 0) {
		throw std::runtime_error("warmup was counted");
	}
	for (int value = 1; value <= 5000; ++value) {
		stats.record(static_cast<double>(value));
	}
	const FilterTimingSnapshot result = stats.snapshot();
	if (result.count != 5000 || result.current != 5000.0 || result.minimum != 1.0 ||
		result.maximum != 5000.0 || result.average != 2500.5) {
		throw std::runtime_error("timing aggregate mismatch");
	}
	if (result.p95 < 4700.0 || result.p95 > 4800.0 ||
		result.p99 < 4900.0 || result.p99 > 5000.0) {
		throw std::runtime_error("timing percentile mismatch");
	}
}
