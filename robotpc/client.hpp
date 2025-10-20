#pragma once

#include <iostream>
#include <vector>
#include <chrono>
#include <winsock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

using std::vector;;
using std::string;
using std::memcpy;
using std::cout;
using std::endl;

class Client
{
private:
	SOCKET client;
	bool alive;
public:
	Client(SOCKET socket, DWORD rx_timeout_ms = 5000) : client(socket), alive(true)
	{
		int v = 1;
		int result = setsockopt(client, IPPROTO_TCP, TCP_NODELAY, (const char*)&v, sizeof(int));
		if (result != 0)
			cout << "Client TCP_NODELAY: " << result << endl;

		result = setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&rx_timeout_ms, sizeof(DWORD));
		if (result != 0)
			cout << "Client SO_RCVTIMEO: " << result << endl;
	}

	bool read_ready(const int timeout_ms) const
	{
		WSAPOLLFD fd[1] = {};
		fd[0].fd = client;
		fd[0].events = POLLRDNORM;
		return WSAPoll(fd, 1, timeout_ms) == 1;
	}

	vector<unsigned char> read_n(const int n)
	{
		vector<unsigned char> result(n);
		int count = 0;
		while (count < n)
		{
			const int tmp = recv(client, (char*)result.data() + count, n - count, 0);
			if (tmp <= 0)
			{
				alive = false;
				break;
			}
			count += tmp;
		}
		return result;
	}

	string read_string(const int length)
	{
		vector<unsigned char> res = read_n(length);
		return string(res.begin(), res.end());
	}

	unsigned char read_byte()
	{
		vector<unsigned char> res = read_n(1);
		unsigned char val = 0;
		memcpy(&val, res.data(), res.size());
		return val;
	}

	int read_int()
	{
		vector<unsigned char> res = read_n(4);
		int val = 0;
		memcpy(&val, res.data(), res.size());
		val = ntohl(val);
		return val;
	}

	int send_n(const unsigned char* data, const int n) const
	{
		int count = 0;
		while (count < n)
		{
			const int tmp = send(client, (const char*)data + count, n - count, 0);
			if (tmp <= 0)
				break;
			count += tmp;
		}
		return count;
	}

	SOCKET get_socket() const
	{
		return client;
	}

	void close()
	{
		if (client != INVALID_SOCKET)
			closesocket(client);
		client = INVALID_SOCKET;
	}

	bool is_alive() const
	{
		return alive;
	}

	~Client()
	{
		close();
	}
};

