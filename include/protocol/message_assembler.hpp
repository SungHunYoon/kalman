#pragma once

#include <string>
#include <vector>

class MessageAssembler {
	private:
		std::string buffer;

	public:
		MessageAssembler();

		std::vector<std::string> append(const std::string& chunk);
};
