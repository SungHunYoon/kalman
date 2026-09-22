#pragma once

#include <string>

bool is_sensor_stream_goodbye(const std::string& datagram);
int sensor_stream_receive_timeout_ms();
