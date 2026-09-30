#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

struct FilterTimingSnapshot {
	std::uint64_t count = 0;
	double current = 0;
	double average = 0;
	double minimum = 0;
	double maximum = 0;
	double p95 = 0;
	double p99 = 0;
};

class FilterStats {
public:
	explicit FilterStats(std::uint64_t warmup_samples = 1000);
	void record(double microseconds);
	FilterTimingSnapshot snapshot(bool include_percentiles = true) const;

private:
	static constexpr std::size_t CAPACITY = 4096;
	std::array<double, CAPACITY> samples_{};
	std::uint64_t warmup_remaining_;
	std::uint64_t count_ = 0;
	double current_ = 0;
	double sum_ = 0;
	double minimum_ = 0;
	double maximum_ = 0;
};
