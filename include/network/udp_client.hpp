#pragma once

#include <string>
#include <cstdint>

class UDPClient {
	private:
		int sock;

	public:
		UDPClient(const std::string& host, const std::uint16_t port);
		~UDPClient();
		UDPClient(const UDPClient&) = delete;
		UDPClient& operator=(const UDPClient&) = delete;

		void send_text(const std::string& msg) const;
		std::string recv_text(int timeout_ms = -1, std::size_t buffer_size = 8192) const;
};
