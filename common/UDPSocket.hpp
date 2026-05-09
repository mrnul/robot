#pragma once

#ifdef _WIN32

#include <WinSock2.h>
#include <WS2tcpip.h>

#define INVALID_SOCKET_VALUE INVALID_SOCKET
#pragma comment(lib, "Ws2_32.lib")

#else

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#define INVALID_SOCKET_VALUE -1
#define SOCKET int
#define closesocket close

#endif

#include <array>
#include <span>
#include <mutex>

using std::array;
using std::span;
using std::mutex;

#include "sockerr.hpp"

using sockerr::SockErr;

template <int N>
struct UDPPacket
{
	int count;
	sockaddr_storage from;
	array<uint8_t, N> buffer;

	UDPPacket(int c, sockaddr_storage f) : count(c), from(f) {}
	UDPPacket() : count(0), from({}) {}
};

class UDPSocket
{
private:
	UDPSocket(const UDPSocket &) = delete;

protected:
	SOCKET s;
	bool connected;

#ifdef _WIN32
	static std::mutex wsaMutex;
	static bool wsaInitialized;

	void initWsa()
	{
		std::lock_guard<std::mutex> lock(wsaMutex);
		if (!wsaInitialized)
		{
			WSADATA wsaData;
			if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0)
				wsaInitialized = true;
		}
	}

	void cleanupWsa()
	{
		std::lock_guard<std::mutex> lock(wsaMutex);
		if (wsaInitialized)
		{
			WSACleanup();
			wsaInitialized = false;
		}
	}
#else
	void initWsa() {}
	void cleanupWsa() {}
#endif

public:
	UDPSocket() : s(INVALID_SOCKET_VALUE), connected(false)
	{
		initWsa();
	}

	bool fatalSocketError() const
	{
#ifdef _WIN32
		const int error = WSAGetLastError();
		return error != WSAEWOULDBLOCK;
#else
		const int error = errno;
		return error != EAGAIN && error != EWOULDBLOCK;
#endif
	}

	bool setBlocking(const bool blocking) const
	{
#ifdef _WIN32
		u_long mode = blocking ? 0 : 1;
		return ioctlsocket(s, FIONBIO, &mode) == 0;
#else
		int flags = fcntl(s, F_GETFL, 0);
		if (flags == -1)
			return false;
		flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
		return fcntl(s, F_SETFL, flags) == 0;
#endif
	}

public:
	bool IsSocketValid()
	{
		return s != INVALID_SOCKET_VALUE;
	}

	SockErr createUDPSocket()
	{
		s = socket(AF_INET, SOCK_DGRAM, 0);
		return IsSocketValid() ? SockErr::ERR_OK : SockErr::ERR_CREATE;
	}

	SockErr createUDPServerSocket(const int port)
	{
		SockErr res = createUDPSocket();
		if (res != SockErr::ERR_OK)
			return res;

		sockaddr_in servaddr = {};
		servaddr.sin_family = AF_INET;
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(port);

		if (bind(s, (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
		{
			closeSocket();
			return SockErr::ERR_BIND;
		}
		return SockErr::ERR_OK;
	}

	bool writeReady(const int timeout_ms)
	{
		fd_set write_fds = {};

		FD_ZERO(&write_fds);
		FD_SET(s, &write_fds);

		timeval tv = {
			.tv_sec = timeout_ms / 1000,
			.tv_usec = (timeout_ms % 1000) * 1000};

		const int sel =
#ifdef _WIN32
			select(0, &write_fds, nullptr, nullptr, &tv);
#else
			select(s + 1, &write_fds, nullptr, nullptr, &tv);
#endif
		if (sel < 0)
		{
			closeSocket();
			return false;
		}
		else if (sel == 0)
		{
			return false;
		}
		return true;
	}

	bool readReady(const int timeout_ms)
	{
		fd_set read_fds = {};
		FD_ZERO(&read_fds);
		FD_SET(s, &read_fds);

		timeval tv = {
			.tv_sec = timeout_ms / 1000,
			.tv_usec = (timeout_ms % 1000) * 1000};

		const int sel =
#ifdef _WIN32
			select(0, &read_fds, nullptr, nullptr, &tv);
#else
			select(s + 1, &read_fds, nullptr, nullptr, &tv);
#endif
		if (sel < 0)
		{
			closeSocket();
			return false;
		}
		else if (sel == 0)
		{
			return false;
		}
		return true;
	}
	void closeSocket()
	{
		closesocket(s);
		s = INVALID_SOCKET_VALUE;
		connected = false;
	}

	SockErr connect(const uint32_t ip, const int port)
	{
		sockaddr_in serverAddr = {};
		serverAddr.sin_family = AF_INET;
		serverAddr.sin_port = htons(port);
		serverAddr.sin_addr.s_addr = ip;

		if (::connect(s, (sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
		{
			return SockErr::ERR_CONNECT;
		}
		connected = true;
		return SockErr::ERR_OK;
	}

	template <int N>
	UDPPacket<N> read()
	{
		UDPPacket<N> result;
		sockaddr_storage from = {};
		socklen_t len = sizeof(from);
		const int n = recvfrom(s, result.buffer.data(), N, 0, (sockaddr *)&from, &len);
		if (n < 0)
		{
			if (fatalSocketError())
			{
				closeSocket();
			}
		}
		result.count = n;
		result.from = from;
		return result;
	}

	SockErr write(const uint8_t *data, const int count, sockaddr_storage *to = nullptr)
	{
		int n = 0;
		socklen_t addrlen;

		if (to == nullptr)
		{
			n = send(s, (char *)data, count, 0);
		}
		else
		{
			if (to->ss_family == AF_INET)
				addrlen = sizeof(sockaddr_in);
			else if (to->ss_family == AF_INET6)
				addrlen = sizeof(sockaddr_in6);
			else
				return SockErr::ERR_INVALID_ADDR;

			n = sendto(s, (char *)data, count, 0, (sockaddr *)to, addrlen);
		}

		if (n < 0)
		{
			if (fatalSocketError())
				closeSocket();
			return SockErr::ERR_SEND;
		}

		if (n != count)
			return SockErr::ERR_PARTIAL_SEND;

		return SockErr::ERR_OK;
	}

	SockErr write(span<const uint8_t> data, sockaddr_storage *to = nullptr)
	{
		return write(data.data(), data.size(), to);
	}

	bool isConnected() const
	{
		return connected;
	}

	~UDPSocket()
	{
		closeSocket();
		cleanupWsa();
	}
};