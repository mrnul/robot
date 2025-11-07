#pragma once

#include <atomic>
#include <iostream>
#include <array>
#include <map>
#include <thread>
#include <memory>
#include <mutex>
#include "UDPSocket.hpp"
#include "UDPRobot.hpp"
#include "mappings.hpp"


using std::atomic;
using std::cout;
using std::endl;
using std::array;
using std::map;
using std::thread;
using std::mutex;
using std::lock_guard;
using std::shared_ptr;
using std::make_shared;

class UDPServer
{
private:
	static constexpr int BUFFER_SIZE = 1 * 1024;
	UDPSocket s;

	mutex uid_to_robot_mutex;
	map<int, shared_ptr<UDPRobot>> uid_to_robot_map;

	thread serverThread;
	atomic<bool> running;
public:

	void start()
	{
		if (s.createUDPServerSocket(8080) != SockErr::ERR_OK)
		{
			cout << "createUDPServerSocket failed" << endl;
			return;
		}
		running.store(true);

		auto _server_lambda = [&]()
			{
				array<uint8_t, BUFFER_SIZE> buffer = {};
				cout << "************* Server running *************" << endl;
				while (running.load())
				{
					cout << "Loop" << endl;
					if (!s.readReady(1000))
						continue;

					RCV incomingData = s.read(buffer.data(), (int)buffer.size());
					if (incomingData.count < 4)
					{
						cout << "Too few bytes: " << incomingData.count << endl;
						continue;
					}

					uint32_t msg_id = 0;
					memcpy(&msg_id, buffer.data(), 4);
					msg_id = htonl(msg_id);

					cout << "Got msg_id: " << msg_id << endl;
					uint8_t uid = 0;
					if (msg_id == Heartbeat::id)
					{
						optional<Heartbeat> msg = Heartbeat::fromBuffer(span<uint8_t>(buffer).subspan(0, incomingData.count));
						if (!msg)
							continue;

						uid = msg.value().uid;
						cout << "Heartbeat from: " << (int)uid << " | " << (int)msg.value().rssi << endl;

						if (uid != 0)
						{
							lock_guard<mutex> l(uid_to_robot_mutex);
							if (!uid_to_robot_map.contains(uid))
								s.write(RequestWhoAmI().toBytes().data(), RequestWhoAmI::msg_size, &incomingData.from);
						}
					}
					else if (msg_id == TextMessage::id)
					{
						optional<TextMessage> msg = TextMessage::fromBuffer(span<uint8_t>(buffer).subspan(0, incomingData.count));
						if (!msg)
							continue;

						uid = msg.value().uid;
						cout << uid << " | " << msg.value().message << endl;
					}
					else if (msg_id == WhoAmI::id)
					{
						optional<WhoAmI> msg = WhoAmI::fromBuffer(span<uint8_t>(buffer).subspan(0, incomingData.count));
						if (!msg)
							continue;
						uid = msg.value().uid;
						if (!uid)
							continue;

						RobotLEDColors colors = Mappings::getColors(uid);
						if (!colors.valid)
							continue;

						s.write(colors.center.toBytes().data(), LEDData::msg_size, &incomingData.from);
						s.write(colors.front.toBytes().data(), LEDData::msg_size, &incomingData.from);

						lock_guard<mutex> l(uid_to_robot_mutex);
						if (!uid_to_robot_map.contains(uid))
							uid_to_robot_map[uid] = make_shared<UDPRobot>(incomingData.from, uid);
					}
					else
					{
						cout << "Unknown msg_id: " << msg_id << endl;
					}
				}
			};

		serverThread = thread(_server_lambda);
	}

	optional<shared_ptr<UDPRobot>> getRobotFromUID(const uint8_t uid) const
	{
		try
		{
			return uid_to_robot_map.at(uid);
		}
		catch (std::out_of_range& ex)
		{
			cout << ex.what() << endl;
			return nullopt;
		}
	}

	bool informRobot(cv::Vec3f desiredLocation, const uint8_t uid)
	{
		lock_guard<mutex> l(uid_to_robot_mutex);
		optional<shared_ptr<UDPRobot>> robot = getRobotFromUID(uid);
		if (!robot)
			return false;
		ControlData data = robot.value()->calcControlData(desiredLocation);
		sockaddr_storage addr = robot.value()->getAddr();
		return s.write(data.toBytes().data(), ControlData::msg_size, &addr) == SockErr::ERR_OK;

	}

	bool updateKinematics(cv::Vec3f c_position, cv::Vec3f f_position, const uint8_t uid)
	{
		lock_guard<mutex> l(uid_to_robot_mutex);
		optional<shared_ptr<UDPRobot>> robot = getRobotFromUID(uid);
		if (!robot)
			return false;

		robot.value()->updatePositionC(c_position, 0.8f);
		robot.value()->updatePositionF(f_position, 0.8f);
		return true;
	}

	void stop()
	{
		running.store(false);
		serverThread.join();
	}
};