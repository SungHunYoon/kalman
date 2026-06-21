#include "network/udp_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>

UDPClient::UDPClient(const std::string& host, const std::uint16_t port)
	: sock(-1) {
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		throw std::runtime_error(std::string("[UDPClient] socket failed: ") + std::strerror(errno));
	}

	sockaddr_in server_addr{};
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);

	if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) != 1) {
		close(sock);
		throw std::runtime_error("invalid IPv4 address: " + host);
	}

	if (connect(sock, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
		close(sock);
		throw std::runtime_error(std::string("[UDPClient] udp connect failed: ") + std::strerror(errno));
	}
}

UDPClient::~UDPClient() {
	if (sock >= 0) {
		close(sock);
	}
}

void UDPClient::send_text(const std::string& msg) const {
	ssize_t sent = send(sock, msg.c_str(), msg.size(), 0);
	if (sent < 0) {
		throw std::runtime_error(std::string("[send_text] send failed: ") + std::strerror(errno));
	}

	if (static_cast<std::size_t>(sent) != msg.size()) {
		throw std::runtime_error("[send_text] send failed: partial datagram sent");
	}
}

std::string UDPClient::recv_text(int timeout_ms, std::size_t buffer_size) const {
	fd_set read_set;
	FD_ZERO(&read_set);
	FD_SET(sock, &read_set);

	timeval timeout{};
	timeval* timeout_ptr = nullptr;

	if (timeout_ms >= 0) {
		timeout.tv_sec = timeout_ms / 1000;
		timeout.tv_usec = (timeout_ms % 1000) * 1000;
		timeout_ptr = &timeout;
	}

	int ret = select(sock + 1, &read_set, nullptr, nullptr, timeout_ptr);
	if (ret < 0) {
		throw std::runtime_error(std::string("[recv_text] select failed: ") + std::strerror(errno));
	}

	if (ret == 0) {
		throw std::runtime_error("[recv_text] recv timeout");
	}

	std::vector<char> buffer(buffer_size + 1, '\0');

	ssize_t received = recv(sock, buffer.data(), buffer_size, 0);
	if (received < 0) {
		throw std::runtime_error(std::string("[recv_text] recv failed: ") + std::strerror(errno));
	}

	buffer[received] = '\0';
	return std::string(buffer.data(), static_cast<std::size_t>(received));
}
