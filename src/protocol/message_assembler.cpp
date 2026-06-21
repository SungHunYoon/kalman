#include "protocol/message_assembler.hpp"

MessageAssembler::MessageAssembler() : buffer(), collecting(false) {}

bool MessageAssembler::append(const std::string& chunk, std::string& completed_message) {
	if (!collecting) {
		std::size_t start_pos = chunk.find("MSG_START");
		if (start_pos == std::string::npos) {
			return false;
		}
		buffer.clear();
		buffer = chunk.substr(start_pos);
		collecting = true;
	} else {
		buffer += chunk;
	}
	std::size_t end_pos = buffer.find("MSG_END");
	if (end_pos == std::string::npos) {
		return false;
	}
	completed_message = buffer.substr(0, end_pos + std::string("MSG_END").size());
	buffer.erase(0, end_pos + std::string("MSG_END").size());
	std::size_t next_start_pos = buffer.find("MSG_START");
	if (next_start_pos == std::string::npos) {
		buffer.clear();
		collecting = false;
	} else {
		buffer.erase(0, next_start_pos);
		collecting = true;
	}
	return true;
}
