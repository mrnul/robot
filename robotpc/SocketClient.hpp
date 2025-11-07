#pragma once

#define NOMINMAX

#include <algorithm>
#include <optional>
#include <iostream>
#include <vector>
#include <chrono>
#include <winsock2.h>
#include <WS2tcpip.h>
#include <string>

#include "ByteBuffer.hpp"

#pragma comment(lib, "Ws2_32.lib")

using std::vector;
using std::string;
using std::memcpy;
using std::cout;
using std::endl;
using std::min;
using std::optional;
using std::nullopt;
using std::chrono::time_point;
using std::chrono::steady_clock;
using std::chrono::duration;


class SocketClient
{
private:
	static constexpr int BUFFER_SIZE = 5 * 1024;

	ByteBuffer<BUFFER_SIZE> rxBuffer;
	SOCKET socket;

	time_point<steady_clock> last_rx;

	SocketClient(const SocketClient&) = delete;
	SocketClient& operator=(const SocketClient&) = delete;
public:
	SocketClient(SOCKET socket) : socket(socket)
	{
		int v = 1;
		int result = setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, (const char*)&v, sizeof(int));
		if (result != 0)
			cout << "SocketClient TCP_NODELAY: " << result << endl;

		u_long non_blocking = 1;
		result = ioctlsocket(socket, FIONBIO, &non_blocking);
		if (result != 0)
			cout << "SocketClient FIONBIO: " << result << endl;

		last_rx = steady_clock::now();
	}

	int sendData(const uint8_t* data, const int size)
	{
		int count = 0;
		while (size > count)
		{
			const int tmp = send(socket, (char*)data + count, size - count, 0);
			if (tmp > 0)
			{
				count += tmp;
			}
			else {
				if (tmp == 0)
				{
					close();
				}
				else
				{
					const int er = WSAGetLastError();
					if (er != WSAEWOULDBLOCK)
					{
						close();
					}
				}
				break;
			}
		}
		return count;
	}

	int rxData()
	{
		const int initial_tail = rxBuffer.tail();
		while (true)
		{
			const int tmp = recv(socket, (char*)rxBuffer.data() + rxBuffer.tail(), rxBuffer.available(), 0);
			if (tmp == 0)
			{
				close();
				break;
			}
			if (tmp < 0)
			{
				const int er = WSAGetLastError();
				if (er != WSAEWOULDBLOCK)
				{
					close();
				}
				break;
			}
			rxBuffer.updateTail(tmp);
		}

		last_rx = steady_clock::now();
		return rxBuffer.tail() - initial_tail;
	}

	int rxedDataSize() const
	{
		return rxBuffer.tail();
	}

	void consumeRxedData(const int n)
	{
		rxBuffer.removeFirstN(n);
	}

	uint8_t* rxedData()
	{
		return rxBuffer.data();
	}

	bool isDead(const float seconds = 3.f)
	{
		return duration<float>(steady_clock::now() - last_rx).count() > seconds;
	}

	SOCKET operator()() const
	{
		return socket;
	}

	void close()
	{
		if (socket != INVALID_SOCKET)
		{
			const int result = closesocket(socket);
			if (result != 0)
			{
				cout << "closesocket() failed: " << WSAGetLastError() << endl;
			}
		}
		socket = INVALID_SOCKET;
	}

	~SocketClient()
	{
		close();
	}
};

