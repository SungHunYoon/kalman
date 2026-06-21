#pragma once

#include "protocol/sensor_update.hpp"
#include <string>

class Parser {
	public:
		SensorUpdate parse(const std::string& message) const;
};
