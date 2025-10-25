#pragma once

#define NOMINMAX

#include <algorithm>
#include <mutex>
#include <optional>
#include <iostream>
#include <vector>
#include <chrono>
#include <winsock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

using std::vector;
using std::string;
using std::memcpy;
using std::cout;
using std::endl;
using std::min;
using std::optional;
using std::nullopt;
using std::mutex;
using std::lock_guard;

constexpr int BATCH_SIZE = 32 * 1024;


class SocketClient
{
private:
	mutex rx_mutex;
	mutex tx_mutex;
	vector<uint8_t> rx_buffer;
	vector<uint8_t> tx_buffer;
	SOCKET socket;
	bool alive;

	SocketClient(const SocketClient&) = delete;
	SocketClient& operator=(const SocketClient&) = delete;
public:
	SocketClient(SOCKET socket) :
		rx_mutex(),
		tx_mutex(),
		rx_buffer(vector<uint8_t>()),
		tx_buffer(vector<uint8_t>()),
		socket(socket),
		alive(true)
	{
		int v = 1;
		int result = setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, (const char*)&v, sizeof(int));
		if (result != 0)
			cout << "SocketClient TCP_NODELAY: " << result << endl;

		u_long non_blocking = 1;
		result = ioctlsocket(socket, FIONBIO, &non_blocking);
		if (result != 0)
			cout << "SocketClient FIONBIO: " << result << endl;
	}

	int read_available_data(const int batch = BATCH_SIZE)
	{
		lock_guard<mutex> l(rx_mutex);

		vector<uint8_t> tmp_buffer(batch);
		int count = 0;
		while (true)
		{
			const int tmp = recv(socket, (char*)tmp_buffer.data(), batch, 0);
			if (tmp > 0)
			{
				count += tmp;
				rx_buffer.insert(rx_buffer.end(), tmp_buffer.begin(), tmp_buffer.begin() + tmp);
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

	void insert_tx_data(const uint8_t* data, const int size)
	{
		lock_guard<mutex> l(tx_mutex);
		tx_buffer.insert(tx_buffer.end(), data, data + size);
	}

	int send_all_pending_data(const int batch = BATCH_SIZE)
	{
		lock_guard<mutex> l(tx_mutex);

		if (tx_buffer_length() == 0)
			return 0;

		int count = 0;
		while (tx_buffer_length())
		{
			const int to_send = min(batch, tx_buffer_length());
			const int tmp = send(socket, (const char*)tx_buffer.data(), to_send, 0);
			if (tmp > 0)
			{
				tx_buffer.erase(tx_buffer.begin(), tx_buffer.begin() + tmp);
				count += tmp;
			}
			else
			{
				const int er = WSAGetLastError();
				if (er != WSAEWOULDBLOCK)
				{
					close();
				}
				break;
			}
		}
		return count;
	}

	int rx_buffer_length()
	{
		return (int)rx_buffer.size();
	}

	int tx_buffer_length()
	{
		return (int)tx_buffer.size();
	}

	optional<int8_t> peek_byte(const int offset = 0)
	{
		lock_guard<mutex> l(rx_mutex);
		if (rx_buffer_length() < 1 + offset)
		{
			return nullopt;
		}
		int8_t val = rx_buffer[offset];
		return val;
	}

	optional<int32_t> peek_int32(const int offset = 0)
	{
		lock_guard<mutex> l(rx_mutex);
		if (rx_buffer_length() < 4 + offset)
		{
			return nullopt;
		}
		int32_t val = 0;
		memcpy(&val, rx_buffer.data() + offset, 4);
		val = ntohl(val);
		return val;
	}

	optional<string> peek_string(const int length, const int offset = 0)
	{
		lock_guard<mutex> l(rx_mutex);

		if (rx_buffer_length() < length + offset)
		{
			return nullopt;
		}
		return string(rx_buffer.begin() + offset, rx_buffer.begin() + offset + length);
	}

	void remove_rx_bytes(const int bytes)
	{
		lock_guard<mutex> l(rx_mutex);
		rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + bytes);
	}

	SOCKET operator()() const
	{
		return socket;
	}

	void close()
	{
		alive = false;
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

	bool is_alive() const
	{
		return alive;
	}

	~SocketClient()
	{
		close();
	}
};

