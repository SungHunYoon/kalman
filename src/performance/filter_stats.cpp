#include "performance/filter_stats.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

FilterStats::FilterStats(std::uint64_t warmup_samples)
	: warmup_remaining_(warmup_samples) {}

void FilterStats::record(double microseconds) {
	if (!std::isfinite(microseconds) || microseconds < 0.0) {
		throw std::invalid_argument("filter duration must be finite and nonnegative");
	}
	if (warmup_remaining_ > 0) {
		--warmup_remaining_;
		return;
	}
	current_ = microseconds;
	if (count_ == 0) {
		minimum_ = microseconds;
		maximum_ = microseconds;
	} else {
		minimum_ = std::min(minimum_, microseconds);
		maximum_ = std::max(maximum_, microseconds);
	}
	samples_[count_ % CAPACITY] = microseconds;
	++count_;
	sum_ += microseconds;
}

FilterTimingSnapshot FilterStats::snapshot() const {
	FilterTimingSnapshot result;
	result.count = count_;
	if (count_ == 0) {
		return result;
	}
	result.current = current_;
	result.average = sum_ / static_cast<double>(count_);
	result.minimum = minimum_;
	result.maximum = maximum_;

	std::array<double, CAPACITY> ordered{};
	const std::size_t used = static_cast<std::size_t>(std::min<std::uint64_t>(count_, CAPACITY));
	std::copy_n(samples_.begin(), used, ordered.begin());
	std::sort(ordered.begin(), ordered.begin() + used);
	const auto percentile_index = [used](double percentile) {
		return static_cast<std::size_t>(std::ceil(percentile * static_cast<double>(used))) - 1;
	};
	result.p95 = ordered[percentile_index(0.95)];
	result.p99 = ordered[percentile_index(0.99)];
	return result;
}
