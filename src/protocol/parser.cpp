#include "protocol/parser.hpp"

#include <cstdio>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace {
	bool contains(const std::string& line, const std::string& text) {
		return line.find(text) != std::string::npos;
	}

	std::vector<std::string> split_lines(const std::string& message) {
		std::vector<std::string> lines;
		std::istringstream iss(message);
		std::string line;
		while (std::getline(iss, line)) {
			if (!line.empty() && line[line.size() - 1] == '\r') {
				line.erase(line.size() - 1);
			}
			lines.push_back(line);
		}
		return lines;
	}

	bool parse_double(const std::string& line, double& out) {
		char* end = NULL;
		const char* str = line.c_str();
		out = std::strtod(str, &end);
		return end != str;
	}

	bool is_label_line(const std::string& line) {
		return contains(line, "TRUE POSITION") ||
			contains(line, "SPEED") ||
			contains(line, "ACCELERATION") ||
			contains(line, "DIRECTION") ||
			contains(line, "GPS") ||
			contains(line, "MSG_START") ||
			contains(line, "MSG_END");
	}

	bool read_next_number(const std::vector<std::string>& lines,
							std::size_t& idx,
							double& out) {
		while (idx < lines.size()) {
			if (lines[idx].empty()) {
				++idx;
				continue;
			}
			if (is_label_line(lines[idx])) {
				return false;
			}
			if (parse_double(lines[idx], out)) {
				++idx;
				return true;
			}
			++idx;
		}
		return false;
	}

	bool read_vector_after_label(const std::vector<std::string>& lines,
								std::size_t label_idx,
								Vector<double>& out) {
		std::size_t idx = label_idx + 1;
		return read_next_number(lines, idx, out[0]) &&
			read_next_number(lines, idx, out[1]) &&
			read_next_number(lines, idx, out[2]);
	}

	bool read_double_after_label(const std::vector<std::string>& lines,
								std::size_t label_idx,
								double& out) {
		std::size_t idx = label_idx + 1;
		return read_next_number(lines, idx, out);
	}

	double parse_time_from_label(const std::string& line) {
		std::size_t left = line.find('[');
		std::size_t right = line.find(']');
		if (left == std::string::npos || right == std::string::npos || right <= left + 1) {
			return 0.0;
		}
		std::string time_str = line.substr(left + 1, right - left - 1);
		int hour = 0;
		int minute = 0;
		double second = 0.0;
		if (std::sscanf(time_str.c_str(), "%d:%d:%lf", &hour, &minute, &second) != 3) {
			return 0.0;
		}
		return static_cast<double>(hour) * 3600.0 +
				static_cast<double>(minute) * 60.0 +
				second;
	}

	void parse_vector_field(const std::vector<std::string>& lines,
							std::size_t label_idx,
							const std::string& field_name,
							std::optional<Vector<double> >& out) {
		Vector<double> value(3);
		if (!read_vector_after_label(lines, label_idx, value)) {
			throw std::runtime_error("[parse_vector_field] failed to parse " + field_name);
		}
		out = value;
	}

	void parse_double_field(const std::vector<std::string>& lines,
							std::size_t label_idx,
							const std::string& field_name,
							std::optional<double>& out) {
		double value = 0.0;
		if (!read_double_after_label(lines, label_idx, value)) {
			throw std::runtime_error("[parse_double_field] failed to parse " + field_name);
		}
		out = value;
	}
}

SensorUpdate Parser::parse(const std::string& message) const {
	SensorUpdate update;
	std::vector<std::string> lines = split_lines(message);
	for (std::size_t i = 0; i < lines.size(); ++i) {
		const std::string& line = lines[i];
		if (contains(line, "TRUE POSITION")) {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "TRUE POSITION", update.initial_position);
		} else if (contains(line, "SPEED")) {
			update.time = parse_time_from_label(line);
			parse_double_field(lines, i, "SPEED", update.initial_speed_kmh);
		} else if (contains(line, "ACCELERATION")) {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "ACCELERATION", update.acceleration);
		} else if (contains(line, "DIRECTION")) {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "DIRECTION", update.direction);
		} else if (contains(line, "GPS")) {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "GPS", update.gps);
		}
	}
	return update;
}
