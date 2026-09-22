#include "protocol/stream_control.hpp"

#include <cctype>

bool is_sensor_stream_goodbye(const std::string& datagram) {
	std::size_t first = 0;
	while (first < datagram.size() &&
		std::isspace(static_cast<unsigned char>(datagram[first]))) {
		++first;
	}
	std::size_t last = datagram.size();
	while (last > first &&
		std::isspace(static_cast<unsigned char>(datagram[last - 1]))) {
		--last;
	}
	return datagram.substr(first, last - first) == "GOODBYE.";
}

int sensor_stream_receive_timeout_ms() {
	return 10000;
}
