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

#include <chrono>

using std::chrono::duration;
using std::chrono::steady_clock;
using std::chrono::time_point;

#include "sockerr.hpp"

using sockerr::SockErr;

struct RCV
{
	int count;
	sockaddr_storage from;

	RCV(int c, sockaddr_storage f) : count(c), from(f) {}
};

class UDPSocket
{
private:
	UDPSocket(const UDPSocket &) = delete;
	UDPSocket &operator=(const UDPSocket &) = delete;

protected:
	time_point<steady_clock> last_rx;
	time_point<steady_clock> last_tx;
	SOCKET s;
	bool wsaStartup;
	bool connected;

	void initWsa()
	{
#ifdef _WIN32
		if (!wsaStartup)
		{
			WSADATA wsaData;
			wsaStartup = (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0);
		}
#endif
	}

	void cleanupWsa()
	{
#ifdef _WIN32
		if (wsaStartup)
		{
			WSACleanup();
			wsaStartup = false;
		}
#endif
	}

public:
	UDPSocket() : s(INVALID_SOCKET_VALUE), wsaStartup(false), connected(false)
	{
		initWsa();
		last_rx = steady_clock::now();
		last_tx = steady_clock::now();
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
	SockErr createUDPSocket()
	{
		if (s != INVALID_SOCKET_VALUE)
		{
			return SockErr::ERR_ALREADY_INIT;
		}
		s = socket(AF_INET, SOCK_DGRAM, 0);
		return s != INVALID_SOCKET_VALUE ? SockErr::ERR_OK : SockErr::ERR_CREATE;
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
		if (s != INVALID_SOCKET_VALUE)
		{
			closesocket(s);
			s = INVALID_SOCKET_VALUE;
		}
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

	RCV read(uint8_t *buffer, const int count)
	{
		sockaddr_storage from = {};
		socklen_t len = sizeof(from);
		const int n = recvfrom(s, (char *)buffer, count, 0, (sockaddr *)&from, &len);
		if (n < 0)
		{
			if (fatalSocketError())
			{
				closeSocket();
			}
		}
		else if (n > 0)
		{
			last_rx = steady_clock::now();
		}
		return RCV(n, from);
	}

	SockErr write(uint8_t *data, const int count, sockaddr_storage *to = nullptr)
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

		last_tx = steady_clock::now();
		return SockErr::ERR_OK;
	}

	float idleRx()
	{
		return duration<float>(steady_clock::now() - last_rx).count();
	}

	float idleTx()
	{
		return duration<float>(steady_clock::now() - last_tx).count();
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