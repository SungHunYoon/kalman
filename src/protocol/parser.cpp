#include "protocol/parser.hpp"

#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace {
	std::string trim(const std::string& text) {
		std::size_t first = 0;
		while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) {
			++first;
		}
		std::size_t last = text.size();
		while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) {
			--last;
		}
		return text.substr(first, last - first);
	}

	std::string label_of(const std::string& line) {
		const std::size_t bracket = line.find(']');
		return trim(bracket == std::string::npos ? line : line.substr(bracket + 1));
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
		char* end = nullptr;
		const char* str = line.c_str();
		out = std::strtod(str, &end);
		if (end == str || !std::isfinite(out)) {
			return false;
		}
		while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
			++end;
		}
		return *end == '\0';
	}

	bool is_label_line(const std::string& line) {
		const std::string label = label_of(line);
		return label == "TRUE POSITION" || label == "POSITION" ||
			label == "SPEED" || label == "ACCELERATION" ||
			label == "DIRECTION" || label == "GPS" ||
			label == "MSG_START" || label == "MSG_END";
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
			throw std::runtime_error("[read_next_number] invalid numeric value: " + lines[idx]);
		}
		return false;
	}

	void require_field_end(const std::vector<std::string>& lines,
						   std::size_t idx,
						   const std::string& field_name) {
		while (idx < lines.size() && lines[idx].empty()) {
			++idx;
		}
		if (idx < lines.size() && !is_label_line(lines[idx])) {
			throw std::runtime_error("[require_field_end] extra value in " + field_name);
		}
	}

	bool read_vector_after_label(const std::vector<std::string>& lines,
								std::size_t label_idx,
								Vector<double>& out) {
		std::size_t idx = label_idx + 1;
		const bool parsed = read_next_number(lines, idx, out[0]) &&
			read_next_number(lines, idx, out[1]) &&
			read_next_number(lines, idx, out[2]);
		if (parsed) {
			require_field_end(lines, idx, label_of(lines[label_idx]));
		}
		return parsed;
	}

	bool read_double_after_label(const std::vector<std::string>& lines,
								std::size_t label_idx,
								double& out) {
		std::size_t idx = label_idx + 1;
		const bool parsed = read_next_number(lines, idx, out);
		if (parsed) {
			require_field_end(lines, idx, label_of(lines[label_idx]));
		}
		return parsed;
	}

	double parse_time_from_label(const std::string& line) {
		std::size_t left = line.find('[');
		std::size_t right = line.find(']');
		if (left == std::string::npos || right == std::string::npos || right <= left + 1) {
			throw std::runtime_error("[parse_time_from_label] missing timestamp");
		}
		std::string time_str = line.substr(left + 1, right - left - 1);
		int hour = 0;
		int minute = 0;
		double second = 0.0;
		char trailing = '\0';
		if (std::sscanf(time_str.c_str(), "%d:%d:%lf%c", &hour, &minute, &second,
						&trailing) != 3 || hour < 0 || hour >= 24 ||
			minute < 0 || minute >= 60 || !std::isfinite(second) ||
			second < 0.0 || second >= 60.0) {
			throw std::runtime_error("[parse_time_from_label] invalid timestamp: " + time_str);
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
		const std::string label = label_of(line);
		if (label == "TRUE POSITION") {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "TRUE POSITION", update.initial_position);
		} else if (label == "SPEED") {
			update.time = parse_time_from_label(line);
			parse_double_field(lines, i, "SPEED", update.initial_speed_kmh);
		} else if (label == "ACCELERATION") {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "ACCELERATION", update.acceleration);
		} else if (label == "DIRECTION") {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "DIRECTION", update.direction);
		} else if (label == "POSITION" || label == "GPS") {
			update.time = parse_time_from_label(line);
			parse_vector_field(lines, i, "POSITION", update.gps);
		}
	}
	return update;
}
