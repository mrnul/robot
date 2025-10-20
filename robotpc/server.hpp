#pragma once

#include <WinSock2.h>
#include <thread>
#include <mutex>
#include <map>
#include <iostream>
#include <vector>
#include "robot_socket_client.hpp"
#include "utils.hpp"

using std::thread;
using std::mutex;
using std::map;
using std::shared_ptr;
using std::vector;
using std::make_shared;
using std::lock_guard;


class Server
{
private:
	SOCKET server;

	thread server_thread;

	mutex mutex_map_uid_robot;
	map<int, shared_ptr<RobotSocketClient>> map_uid_robot;

	mutex mutex_vec_robot_threads;
	vector<thread> robot_threads;

	bool running;

	bool read_ready(const int timeout_ms) const
	{
		WSAPOLLFD fd[1] = {};
		fd[0].fd = server;
		fd[0].events = POLLRDNORM;
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

	void start()
	{
		server = socket(AF_INET, SOCK_STREAM, 0);

		if (server == INVALID_SOCKET)
		{
			WSACleanup();
			return;
		}

		sockaddr_in serverAddress = {};
		serverAddress.sin_family = AF_INET;
		serverAddress.sin_port = htons(8080);
		serverAddress.sin_addr.s_addr = INADDR_ANY;

		if (bind(server, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR)
		{
			WSACleanup();
			return;
		}
		if (listen(server, 100) == SOCKET_ERROR)
		{
			WSACleanup();
			return;
		}

		running = true;

		auto _client_rx_lambda = [this](SOCKET socket)
			{
				cout << "New client thread | Socket: " << socket << endl;
				shared_ptr<RobotSocketClient> robot = make_shared<RobotSocketClient>(socket);
				do
				{
					int msg_id = robot->read_int();
					if (msg_id == 1000)
					{
						const int rssi = robot->read_byte();
						// cout << "Client: " << client->get_uid() << " | rssi: " << rssi << endl;
					}
					else if (msg_id == 1001)
					{
						const int len = robot->read_int();
						string msg = robot->read_string(len);
						const int rssi = robot->read_byte();
						cout << "Robot: " << robot->get_uid() << " | rssi: " << rssi << " | msg: " << msg << endl;
					}
					else if (msg_id == 1003)
					{
						const int uid = robot->read_int();
						robot->set_uid(uid);

						{
							lock_guard<mutex> l(mutex_map_uid_robot);
							map_uid_robot.emplace(std::pair<int, shared_ptr<RobotSocketClient>>(uid, robot));
						}

						cout << "Got uid: " << robot->get_uid() << endl;
					}

				} while (robot->is_alive() && running);

				cout << "Client: " << robot->get_uid() << " Goodbye" << endl;

				{
					lock_guard<mutex> l(mutex_map_uid_robot);
					map_uid_robot.erase(robot->get_uid());
				}

			};

		auto _server_lambda = [this, _client_rx_lambda]()
			{
				cout << "Server thread running..." << endl;
				while (running)
				{
					sockaddr addr = {};
					int sockaddr_len = sizeof(sockaddr);
					if (!read_ready(1000))
						continue;

					SOCKET client_socket = accept(server, &addr, &sockaddr_len);
					if (client_socket == INVALID_SOCKET)
						continue;

					char str[32] = {};
					sockaddr_in* addr_in = reinterpret_cast<sockaddr_in*>(&addr);
					cout << "New client: " << inet_ntop(AF_INET, &(addr_in->sin_addr), str, 31) << endl;


					{
						lock_guard<mutex> l(mutex_vec_robot_threads);
						robot_threads.push_back(thread(_client_rx_lambda, client_socket));
					}

				}
			};
		server_thread = thread(_server_lambda);
	}

	void stop()
	{
		running = false;
	}

	void wait()
	{
		if (server_thread.joinable())
			server_thread.join();
		{
			lock_guard<mutex> l(mutex_vec_robot_threads);
			for (thread& t : robot_threads)
			{
				if (t.joinable())
					t.join();
			}
			robot_threads.clear();
		}
		WSACleanup();
	}

	void stop_wait()
	{
		stop();
		wait();
	}

	bool update_kinematics(Vec2fT pos_c, Vec2fT pos_f, const int client_id)
	{
		lock_guard<mutex> l(mutex_map_uid_robot);
		const map<int, shared_ptr<RobotSocketClient>>::iterator& it = map_uid_robot.find(client_id);
		if (it == map_uid_robot.end())
			return false;

		it->second->update_position_c(pos_c, 0.9f);
		it->second->update_position_f(pos_f, 0.9f);
		it->second->update_theta(1.f);
		return true;
	}

	bool inform_client(Vec2fT desired_location, const int client_id)
	{
		lock_guard<mutex> lock(mutex_map_uid_robot);
		const map<int, shared_ptr<RobotSocketClient>>::iterator& it = map_uid_robot.find(client_id);
		if (it == map_uid_robot.end())
			return false;

		return it->second->calc_and_send_data(desired_location);
	}

	~Server()
	{
		stop_wait();
	}
};