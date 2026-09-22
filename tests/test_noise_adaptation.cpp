#include "filter/noise_adaptation.hpp"
#include "filter/fixed_kalman_filter.hpp"
#include "protocol/sensor_state.hpp"

#include <stdexcept>

void run_noise_adaptation_tests() {
	NoiseAdaptation adaptation(1.0, 11.345);
	for (int i = 0; i < 5; ++i) {
		adaptation.record_rejection();
	}
	if (adaptation.current_gate_threshold() != 22.69) {
		throw std::runtime_error("gate did not expand after five rejects");
	}
	for (int i = 0; i < 20; ++i) {
		adaptation.record_rejection();
	}
	if (adaptation.current_gate_threshold() > 11.345 * 16.0) {
		throw std::runtime_error("gate exceeded cap");
	}
	adaptation.record_acceptance({20, 0, 0}, {1, 1, 1});
	if (adaptation.current_gate_threshold() != 11.345) {
		throw std::runtime_error("acceptance did not reset gate");
	}
	const Vector3d high = adaptation.gps_variance();
	if (high[0] <= high[1] || high[0] > 100.0 || high[1] < 0.25) {
		throw std::runtime_error("adaptive variance clamp mismatch");
	}

	FixedKalmanFilter filter({0, 0, 0}, {0, 0, 0}, {1e-2, 1.0, 11.345});
	const FilterSnapshot before = filter.snapshot();
	const GpsUpdateResult rejected = filter.update_gps({1000, 1000, 1000});
	if (rejected.accepted || filter.snapshot().position != before.position ||
		filter.accepted_gps_count() != 0 || filter.rejected_gps_count() != 1) {
		throw std::runtime_error("outlier rejection changed state or counters");
	}
	const GpsUpdateResult accepted = filter.update_gps({0, 0, 0});
	if (!accepted.accepted || filter.accepted_gps_count() != 1 ||
		filter.rejected_gps_count() != 1) {
		throw std::runtime_error("accepted GPS counters mismatch");
	}

	SensorState state({1e-2, 1.0, 11.345});
	SensorUpdate initial;
	initial.time = 0.0;
	initial.initial_position = Vector<double>{0, 0, 0};
	initial.initial_speed_kmh = 0.0;
	initial.direction = Vector<double>{0, 0, 0};
	state.apply(initial);
	SensorUpdate outlier;
	outlier.time = 1.0;
	outlier.acceleration = Vector<double>{0, 0, 0};
	outlier.gps = Vector<double>{1000, 1000, 1000};
	state.apply(outlier);
	if (state.last_gps_result().accepted || state.rejected_gps_count() != 1 ||
		state.filter_snapshot().position != Vector3d{0, 0, 0}) {
		throw std::runtime_error("SensorState did not expose rejected GPS snapshot");
	}
}
