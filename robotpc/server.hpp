#pragma once

#include <WinSock2.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <iostream>
#include <vector>
#include "robot.hpp"
#include "utils.hpp"

using std::thread;
using std::mutex;
using std::map;
using std::pair;
using std::shared_ptr;
using std::vector;
using std::make_shared;
using std::lock_guard;
using std::scoped_lock;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::atomic;
using std::for_each;
using std::erase_if;


class Server
{
private:
	SOCKET server;

	thread server_thread;
	thread clients_rx_thread;

	mutex mutex_map_socket_robot;
	map<SOCKET, shared_ptr<Robot>> map_socket_robot;

	mutex mutex_uid_robot;
	map<int, shared_ptr<Robot>> map_uid_robot;

	atomic<bool> running;

	bool server_read_ready(const int timeout_ms) const
	{
		WSAPOLLFD fd[1] = {};
		fd[0].fd = server;
		fd[0].events = POLLIN;
		return WSAPoll(fd, 1, timeout_ms) == 1;
	}

public:
	Server() : server(INVALID_SOCKET), running(false)
	{
		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			cout << "WSAStartup failed" << endl;
			return;
		}
	}

	bool start()
	{
		server = socket(AF_INET, SOCK_STREAM, 0);

		if (server == INVALID_SOCKET)
		{
			WSACleanup();
			return false;
		}

		sockaddr_in serverAddress = {};
		serverAddress.sin_family = AF_INET;
		serverAddress.sin_port = htons(8080);
		serverAddress.sin_addr.s_addr = INADDR_ANY;

		if (bind(server, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR)
		{
			WSACleanup();
			return false;
		}
		if (listen(server, 100) == SOCKET_ERROR)
		{
			WSACleanup();
			return false;
		}

		running = true;

		auto _clients_rx_lambda = [this]()
			{
				vector<WSAPOLLFD> pollfds;
				while (running.load())
				{
					if (map_socket_robot.size() == 0)
					{
						sleep_for(milliseconds(200));
						continue;
					}

					pollfds.clear();
					{
						lock_guard<mutex> l(mutex_map_socket_robot);
						for_each(map_socket_robot.begin(), map_socket_robot.end(), [&pollfds](const pair<SOCKET, shared_ptr<Robot>>& pair)
							{
								pollfds.push_back({ .fd = pair.first, .events = POLLIN, .revents = 0 });
							});
					}

					const int count = WSAPoll(pollfds.data(), (ULONG)pollfds.size(), 2000);
					if (count < 0)
					{
						cout << "Error: " << WSAGetLastError() << endl;
						continue;
					}

					
					lock_guard<mutex> l(mutex_map_socket_robot);
					for (const WSAPOLLFD& item : pollfds)
					{
						cout << "Loop..." << endl;

						if ((item.revents & (POLLERR | POLLHUP)) != 0)
						{
							shared_ptr<Robot> robot = map_socket_robot[item.fd];
							cout << "Deleting: " << robot->get_uid() << endl;
							if (robot->get_uid() != 0)
							{
								lock_guard<mutex> l(mutex_uid_robot);
								map_uid_robot.erase(robot->get_uid());
							}
							map_socket_robot.erase(item.fd);
							continue;
						}
						if ((item.revents & POLLIN) == 0)
						{
							cout << item.revents << endl;
							continue;
						}

						shared_ptr<Robot> robot = map_socket_robot[item.fd];
						robot->read_all_available_data();

						optional<int> msg_id = robot->peek_msg_id();
						if (msg_id == nullopt)
						{
							cout << "msg_id == nullopt" << endl;
							continue;
						}

						const int32_t message_id = msg_id.value();
						// cout << "\t_clients_rx_lambda - message_id: " << message_id << endl;

						if (message_id == Heartbeat::id)
						{
							optional<Heartbeat> message = robot->get_heartbeat();
							if (message == nullopt)
								continue;

							// cout << "_clients_rx_lambda - rssi: " << (int)message.value().rssi << endl;
						}
						else if (message_id == WhoAmI::id)
						{
							optional<WhoAmI> message = robot->get_who_am_i();
							if (message == nullopt)
								continue;
							const int32_t uid = message.value().uid;
							cout << "_clients_rx_lambda - uid: " << uid << endl;
							lock_guard<mutex> l(mutex_uid_robot);
							map_uid_robot[uid] = robot;
						}
						else if (message_id == TextMessage::id)
						{
							optional<TextMessage> message = robot->get_text_message();
							if (message == nullopt)
								continue;

							cout << "_clients_rx_lambda - text: " << message.value().msg << endl;
							cout << "_clients_rx_lambda - rssi: " << (int)message.value().rssi << endl;
						}
					}
				}
			};

		auto _server_lambda = [this]()
			{
				cout << "Server thread running..." << endl;
				while (running)
				{
					sockaddr addr = {};
					int sockaddr_len = sizeof(sockaddr);
					if (!server_read_ready(1000))
						continue;

					SOCKET client_socket = accept(server, &addr, &sockaddr_len);
					if (client_socket == INVALID_SOCKET)
						continue;

					char str[32] = {};
					sockaddr_in* addr_in = reinterpret_cast<sockaddr_in*>(&addr);
					cout << "New client: " << inet_ntop(AF_INET, &(addr_in->sin_addr), str, 31) << endl;

					lock_guard<mutex> l(mutex_map_socket_robot);
					map_socket_robot[client_socket] = make_shared<Robot>(client_socket);
				}
			};
		server_thread = thread(_server_lambda);
		clients_rx_thread = thread(_clients_rx_lambda);
		return true;
	}

	void stop()
	{
		running.store(false);
	}

	void wait()
	{
		if (server != INVALID_SOCKET) {
			closesocket(server);
			server = INVALID_SOCKET;
		}
		if (server_thread.joinable())
			server_thread.join();
		if (clients_rx_thread.joinable())
			clients_rx_thread.join();
		WSACleanup();
	}

	void stop_wait()
	{
		stop();
		wait();
	}

	bool update_kinematics(const cv::Vec3f pos_c, const cv::Vec3f pos_f, const int robot_id)
	{
		lock_guard<mutex> l(mutex_uid_robot);
		const map<int, shared_ptr<Robot>>::iterator& it = map_uid_robot.find(robot_id);
		if (it == map_uid_robot.end())
			return false;

		it->second->update_position_c(pos_c, 0.8f);
		it->second->update_position_f(pos_f, 0.8f);
		return true;
	}

	int inform_robot(cv::Vec3f desired_location, const int robot_id)
	{
		lock_guard<mutex> lock(mutex_uid_robot);
		const map<int, shared_ptr<Robot>>::iterator& it = map_uid_robot.find(robot_id);
		if (it == map_uid_robot.end())
			return -1;

		return it->second->calc_and_send_data(desired_location);
	}

	~Server()
	{
		stop_wait();
	}
};