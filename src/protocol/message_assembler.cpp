#include "protocol/message_assembler.hpp"

namespace {
	const std::string START_MARKER = "MSG_START";
	const std::string END_MARKER = "MSG_END";

	std::size_t marker_prefix_length(const std::string& text, const std::string& marker) {
		const std::size_t maximum = std::min(text.size(), marker.size() - 1);
		for (std::size_t length = maximum; length > 0; --length) {
			if (text.compare(text.size() - length, length, marker, 0, length) == 0) {
				return length;
			}
		}
		return 0;
	}

	bool completes_split_marker(const std::string& left,
								const std::string& right,
								const std::string& marker) {
		for (std::size_t length = 1; length < marker.size(); ++length) {
			if (left.size() >= length && right.size() >= marker.size() - length &&
				left.compare(left.size() - length, length, marker, 0, length) == 0 &&
				right.compare(0, marker.size() - length, marker, length,
					marker.size() - length) == 0) {
				return true;
			}
		}
		return false;
	}
}

MessageAssembler::MessageAssembler() : buffer() {}

std::vector<std::string> MessageAssembler::append(const std::string& chunk) {
	std::vector<std::string> messages;
	if (!buffer.empty() &&
		!completes_split_marker(buffer, chunk, START_MARKER) &&
		!completes_split_marker(buffer, chunk, END_MARKER) &&
		buffer[buffer.size() - 1] != '\n' &&
		(chunk.empty() || chunk[0] != '\n')) {
		buffer += '\n';
	}
	buffer += chunk;

	while (true) {
		const std::size_t start_pos = buffer.find(START_MARKER);
		if (start_pos == std::string::npos) {
			const std::size_t keep = marker_prefix_length(buffer, START_MARKER);
			buffer.erase(0, buffer.size() - keep);
			break;
		}
		if (start_pos > 0) {
			buffer.erase(0, start_pos);
		}

		const std::size_t end_pos = buffer.find(END_MARKER, START_MARKER.size());
		if (end_pos == std::string::npos) {
			break;
		}

		const std::size_t message_size = end_pos + END_MARKER.size();
		messages.push_back(buffer.substr(0, message_size));
		buffer.erase(0, message_size);
	}
	return messages;
}
