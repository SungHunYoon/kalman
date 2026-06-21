#pragma once

#include <string>

class MessageAssembler {
	private:
		std::string buffer;
		bool collecting;

	public:
		MessageAssembler();

		bool append(const std::string& chunk, std::string& completed_message);
};
