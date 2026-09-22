#include "filter/kalman_filter.hpp"
#include "protocol/message_assembler.hpp"
#include "protocol/parser.hpp"
#include "protocol/sensor_state.hpp"
#include "protocol/stream_control.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

void run_option_tests();
void run_fixed_kalman_filter_tests();

namespace {
	void require(bool condition, const std::string& message) {
		if (!condition) {
			throw std::runtime_error(message);
		}
	}

	void require_close(double actual, double expected, const std::string& message) {
		if (!std::isfinite(actual) || !std::isfinite(expected) ||
			std::abs(actual - expected) > 1e-9) {
			throw std::runtime_error(message);
		}
	}

	void test_assembles_split_markers() {
		MessageAssembler assembler;
		require(assembler.append("noise MSG_STA").empty(), "partial start marker produced a message");
		require(assembler.append("RT\nA\nMSG_").empty(), "partial end marker produced a message");
		const std::vector<std::string> messages = assembler.append("END trailing");
		require(messages.size() == 1, "split message was not assembled");
		require(messages[0] == "MSG_START\nA\nMSG_END", "assembled payload differs from input");
	}

	void test_extracts_multiple_messages_from_one_chunk() {
		MessageAssembler assembler;
		const std::vector<std::string> messages = assembler.append(
			"MSG_START\nA\nMSG_ENDMSG_START\nB\nMSG_END");
		require(messages.size() == 2, "expected two messages from one chunk");
		require(messages[0] == "MSG_START\nA\nMSG_END", "first message mismatch");
		require(messages[1] == "MSG_START\nB\nMSG_END", "second message mismatch");
	}

	void test_assembles_line_per_datagram_protocol() {
		MessageAssembler assembler;
		const std::vector<std::string> chunks = {
			"MSG_START", "[00:00:00.000]TRUE POSITION", "-6.5", "-3.9", "0.5", "",
			"[00:00:00.000]SPEED", "36", "", "[00:00:00.000]DIRECTION", "0", "0", "0",
			"MSG_END"
		};
		std::vector<std::string> messages;
		for (const std::string& chunk : chunks) {
			const std::vector<std::string> completed = assembler.append(chunk);
			messages.insert(messages.end(), completed.begin(), completed.end());
		}
		require(messages.size() == 1, "datagram sequence did not produce one message");
		const SensorUpdate update = Parser().parse(messages[0]);
		require(update.initial_position.has_value(), "datagram fields were concatenated");
		require(update.initial_speed_kmh.has_value(), "datagram speed was not parsed");
		require_close((*update.initial_position)[0], -6.5, "datagram X mismatch");
	}

	void test_rejects_oversized_incomplete_message() {
		MessageAssembler assembler;
		assembler.append("MSG_START");
		bool threw = false;
		try {
			assembler.append(std::string(1024 * 1024, 'x'));
		} catch (const std::runtime_error&) {
			threw = true;
		}
		require(threw, "oversized incomplete message was retained");
	}

	void test_parses_position_as_gps_without_confusing_true_position() {
		Parser parser;
		const SensorUpdate update = parser.parse(
			"MSG_START\r\n"
			"[00:00:01.250] TRUE POSITION\r\n1\r\n2\r\n3\r\n"
			"[00:00:01.250] POSITION\r\n4\r\n5\r\n6\r\n"
			"MSG_END\r\n");
		require(update.initial_position.has_value(), "TRUE POSITION was not parsed");
		require(update.gps.has_value(), "POSITION was not parsed as GPS");
		require_close((*update.initial_position)[0], 1.0, "initial X mismatch");
		require_close((*update.gps)[0], 4.0, "GPS X mismatch");
		require_close(update.time, 1.25, "timestamp mismatch");
	}

	void test_rejects_number_with_trailing_data() {
		Parser parser;
		bool threw = false;
		try {
			parser.parse("MSG_START\n[00:00:01.000] POSITION\n1oops\n2\n3\nMSG_END\n");
		} catch (const std::runtime_error&) {
			threw = true;
		}
		require(threw, "numeric value with trailing data was accepted");
	}

	void test_rejects_malformed_value_even_when_more_numbers_follow() {
		Parser parser;
		bool threw = false;
		try {
			parser.parse("MSG_START\n[00:00:01.000]POSITION\n1oops\n2\n3\n4\nMSG_END\n");
		} catch (const std::runtime_error&) {
			threw = true;
		}
		require(threw, "malformed value was skipped in favor of later numbers");
	}

	void test_rejects_extra_vector_component() {
		Parser parser;
		bool threw = false;
		try {
			parser.parse("MSG_START\n[00:00:01.000]POSITION\n1\n2\n3\n4\nMSG_END\n");
		} catch (const std::runtime_error&) {
			threw = true;
		}
		require(threw, "extra vector component was accepted");
	}

	void test_rejects_invalid_timestamp() {
		Parser parser;
		bool threw = false;
		try {
			parser.parse("MSG_START\n[not-a-time]POSITION\n1\n2\n3\nMSG_END\n");
		} catch (const std::runtime_error&) {
			threw = true;
		}
		require(threw, "invalid timestamp was accepted");
	}

	void test_kalman_predicts_constant_acceleration() {
		KalmanFilter filter(Vector<double>{0.0, 0.0, 0.0}, Vector<double>{1.0, 0.0, 0.0});
		filter.predict(Vector<double>{2.0, 0.0, 0.0}, 0.5);
		require_close(filter.position()[0], 0.75, "predicted position mismatch");
		require_close(filter.velocity()[0], 2.0, "predicted velocity mismatch");
	}

	void test_kalman_gps_update_moves_estimate_toward_measurement() {
		KalmanFilter filter(Vector<double>{0.0, 0.0, 0.0}, Vector<double>{0.0, 0.0, 0.0});
		filter.predict(Vector<double>{0.0, 0.0, 0.0}, 3.0);
		const double before = filter.position()[0];
		filter.update_gps(Vector<double>{1.0, 0.0, 0.0});
		require(filter.position()[0] > before, "GPS update did not move toward measurement");
		require(filter.position()[0] < 1.0, "GPS update overshot measurement");
	}

	void test_kalman_does_not_overtrust_stream_default_gps_noise() {
		KalmanFilter filter(Vector<double>{0.0, 0.0, 0.0}, Vector<double>{0.0, 0.0, 0.0});
		filter.predict(Vector<double>{0.0, 0.0, 0.0}, 3.0);
		filter.update_gps(Vector<double>{1.0, 0.0, 0.0});
		require(filter.position()[0] < 0.2, "filter overtrusted a noisy GPS sample");
	}

	void test_kalman_models_stream_default_accelerometer_noise() {
		KalmanFilter filter(Vector<double>{0.0, 0.0, 0.0}, Vector<double>{0.0, 0.0, 0.0});
		filter.predict(Vector<double>{0.0, 0.0, 0.0}, 3.0);
		require(filter.state_covariance()(3, 3) > 0.0105,
			"process covariance underestimates stream accelerometer noise");
	}

	void test_matrix_inverse_produces_identity() {
		const Matrix<double> matrix{{4.0, 7.0}, {2.0, 6.0}};
		const Matrix<double> product = matrix.mul_mat(matrix.inverse());
		require_close(product(0, 0), 1.0, "inverse identity (0,0) mismatch");
		require_close(product(0, 1), 0.0, "inverse identity (0,1) mismatch");
		require_close(product(1, 0), 0.0, "inverse identity (1,0) mismatch");
		require_close(product(1, 1), 1.0, "inverse identity (1,1) mismatch");
	}

	void test_kalman_covariance_remains_symmetric_and_finite() {
		KalmanFilter filter(Vector<double>{0.0, 0.0, 0.0}, Vector<double>{1.0, 2.0, 3.0});
		filter.predict(Vector<double>{0.1, -0.2, 0.3}, 0.01);
		filter.update_gps(Vector<double>{0.02, 0.01, 0.04});
		const Matrix<double>& covariance = filter.state_covariance();
		for (std::size_t row = 0; row < 6; ++row) {
			for (std::size_t col = 0; col < 6; ++col) {
				require(std::isfinite(covariance(row, col)), "covariance contains a non-finite value");
				require_close(covariance(row, col), covariance(col, row), "covariance is not symmetric");
			}
		}
	}

	void test_sensor_state_initializes_and_predicts_with_si_units() {
		SensorState state;
		SensorUpdate initial;
		initial.time = 0.0;
		initial.initial_position = Vector<double>{0.0, 0.0, 0.0};
		initial.initial_speed_kmh = 36.0;
		initial.direction = Vector<double>{0.0, 0.0, 0.0};
		initial.acceleration = Vector<double>{0.0, 0.0, 0.0};
		state.apply(initial);
		require(state.has_estimated_position(), "state did not initialize");

		SensorUpdate next;
		next.time = 1.0;
		next.acceleration = Vector<double>{1.0, 0.0, 0.0};
		state.apply(next);
		require_close(state.estimated_position()[0], 10.5, "state prediction did not use m/s");
	}

	void test_sensor_state_uses_yaw_for_initial_velocity() {
		SensorState state;
		SensorUpdate initial;
		initial.time = 10.0;
		initial.initial_position = Vector<double>{0.0, 0.0, 0.0};
		initial.initial_speed_kmh = 36.0;
		initial.direction = Vector<double>{0.0, 0.0, 1.5707963267948966};
		state.apply(initial);

		SensorUpdate next;
		next.time = 11.0;
		next.acceleration = Vector<double>{0.0, 0.0, 0.0};
		state.apply(next);
		require_close(state.estimated_position()[0], 0.0, "yaw left velocity on X axis");
		require_close(state.estimated_position()[1], 10.0, "yaw did not rotate velocity to Y axis");
	}

	void test_sensor_state_handles_midnight_rollover() {
		SensorState state;
		SensorUpdate initial;
		initial.time = 86399.99;
		initial.initial_position = Vector<double>{0.0, 0.0, 0.0};
		initial.initial_speed_kmh = 36.0;
		initial.direction = Vector<double>{0.0, 0.0, 0.0};
		state.apply(initial);

		SensorUpdate next;
		next.time = 0.0;
		next.acceleration = Vector<double>{0.0, 0.0, 0.0};
		state.apply(next);
		require_close(state.estimated_position()[0], 0.1, "midnight rollover produced the wrong dt");
	}

	void test_sensor_state_ignores_stale_updates_without_mutating_inputs() {
		SensorState state;
		SensorUpdate initial;
		initial.time = 0.0;
		initial.initial_position = Vector<double>{0.0, 0.0, 0.0};
		initial.initial_speed_kmh = 0.0;
		initial.direction = Vector<double>{0.0, 0.0, 0.0};
		state.apply(initial);

		SensorUpdate current;
		current.time = 1.0;
		current.acceleration = Vector<double>{1.0, 0.0, 0.0};
		state.apply(current);

		SensorUpdate stale;
		stale.time = 0.5;
		stale.acceleration = Vector<double>{100.0, 0.0, 0.0};
		stale.gps = Vector<double>{1000.0, 0.0, 0.0};
		state.apply(stale);
		require_close(state.estimated_position()[0], 0.5, "stale GPS changed the estimate");

		SensorUpdate next;
		next.time = 2.0;
		state.apply(next);
		require_close(state.estimated_position()[0], 2.0, "stale acceleration replaced current input");
	}

	void test_recognizes_sensor_stream_goodbye() {
		require(is_sensor_stream_goodbye("GOODBYE."), "GOODBYE was not recognized");
		require(!is_sensor_stream_goodbye("Trajectory Generated!"), "ordinary status was goodbye");
	}

	void test_sensor_stream_receive_timeout_is_finite() {
		require(sensor_stream_receive_timeout_ms() > 0, "sensor receive timeout is infinite");
	}
}

int main() {
	try {
		run_option_tests();
		run_fixed_kalman_filter_tests();
		test_assembles_split_markers();
		test_extracts_multiple_messages_from_one_chunk();
		test_assembles_line_per_datagram_protocol();
		test_rejects_oversized_incomplete_message();
		test_parses_position_as_gps_without_confusing_true_position();
		test_rejects_number_with_trailing_data();
		test_rejects_malformed_value_even_when_more_numbers_follow();
		test_rejects_extra_vector_component();
		test_rejects_invalid_timestamp();
		test_kalman_predicts_constant_acceleration();
		test_kalman_gps_update_moves_estimate_toward_measurement();
		test_kalman_does_not_overtrust_stream_default_gps_noise();
		test_kalman_models_stream_default_accelerometer_noise();
		test_matrix_inverse_produces_identity();
		test_kalman_covariance_remains_symmetric_and_finite();
		test_sensor_state_initializes_and_predicts_with_si_units();
		test_sensor_state_uses_yaw_for_initial_velocity();
		test_sensor_state_handles_midnight_rollover();
		test_sensor_state_ignores_stale_updates_without_mutating_inputs();
		test_recognizes_sensor_stream_goodbye();
		test_sensor_stream_receive_timeout_is_finite();
		std::cout << "23 test groups passed\n";
	} catch (const std::exception& e) {
		std::cerr << "test failure: " << e.what() << "\n";
		return 1;
	}
	return 0;
}
