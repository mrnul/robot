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
#include "Point.hpp"


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
	static constexpr int READ_READY_TIMEOUT = 5000;

	UDPSocket s;

	mutex uid_to_robot_mutex;
	map<int, shared_ptr<UDPRobot>> uid_to_robot_map;

	thread serverThread;
	atomic<bool> running;
private:
	optional<shared_ptr<UDPRobot>> getRobotFromUID(const uint8_t uid) const
	{
		try
		{
			return uid_to_robot_map.at(uid);
		}
		catch (std::out_of_range&)
		{
			return nullopt;
		}
	}

private:
	void handleHeartbeat(const span<uint8_t> buffer, RCV& incomingData)
	{
		cout << "Heartbeat" << endl;
		optional<Heartbeat> msg = Heartbeat::fromBuffer(buffer.subspan(0, incomingData.count));

		if (!msg)
			return;

		const uint8_t uid = msg.value().uid;
		const uint8_t rssi = msg.value().rssi;
		cout << "\tuid: " << (int)uid << " | rssi: " << (int)rssi << endl;

		lock_guard<mutex> l(uid_to_robot_mutex);
		if (!uid_to_robot_map.contains(uid))
		{
			cout << "\tRequestWhoAmI: " << (int)uid << endl;
			s.write(RequestWhoAmI().toBytes().data(), RequestWhoAmI::msg_size, &incomingData.from);
		}

	}

	void handleTextMessage(const span<uint8_t> buffer, RCV& incomingData)
	{
		cout << "TextMessage" << endl;
		optional<TextMessage> msg = TextMessage::fromBuffer(buffer.subspan(0, incomingData.count));
		if (!msg)
			return;

		const uint8_t uid = msg.value().uid;
		const uint8_t rssi = msg.value().rssi;
		cout << "\tuid: " << (int)uid << " | rssi: " << (int)rssi << " | msg: " << msg.value().message << endl;
	}

	void handleWhoAmI(const span<uint8_t> buffer, RCV& incomingData)
	{
		cout << "WhoAmI" << endl;
		optional<WhoAmI> msg = WhoAmI::fromBuffer(buffer.subspan(0, incomingData.count));
		if (!msg)
			return;

		const uint8_t uid = msg.value().uid;

		cout << "\tuid: " << (int)uid << endl;

		const RobotLEDColors& colors = Mappings::getColors(uid);
		if (!colors.valid)
			return;

		s.write(colors.center.toBytes().data(), LEDData::msg_size, &incomingData.from);
		s.write(colors.front.toBytes().data(), LEDData::msg_size, &incomingData.from);

		lock_guard<mutex> l(uid_to_robot_mutex);
		if (uid_to_robot_map.contains(uid))
		{
			cout << "\tRenew: " << (int)uid << endl;
			uid_to_robot_map.erase(uid);
		}
		uid_to_robot_map[uid] = make_shared<UDPRobot>(incomingData.from, uid);
	}
public:
	void start()
	{
		if (s.createUDPServerSocket(8080) != SockErr::ERR_OK)
		{
			cout << "createUDPServerSocket failed" << endl;
			return;
		}
		if (!s.setBlocking(false))
		{
			cout << "setBlocking(false) failed" << endl;
			return;
		}
		running.store(true);

		auto _server_lambda = [&]()
			{
				array<uint8_t, BUFFER_SIZE> buffer = {};
				cout << "************* Server running *************" << endl;
				while (running.load())
				{
					if (!s.readReady(READ_READY_TIMEOUT))
					{
						cout << "Nothing to receive..." << endl;
						continue;
					}

					RCV incomingData = s.read(buffer.data(), (int)buffer.size());
					if (incomingData.count < 4)
					{
						cout << "Too few bytes: " << incomingData.count << endl;
						continue;
					}

					uint32_t msg_id = 0;
					memcpy(&msg_id, buffer.data(), 4);
					msg_id = ntohl(msg_id);

					if (msg_id == Heartbeat::id)
					{
						handleHeartbeat(buffer, incomingData);
					}
					else if (msg_id == TextMessage::id)
					{
						handleTextMessage(buffer, incomingData);
					}
					else if (msg_id == WhoAmI::id)
					{
						handleWhoAmI(buffer, incomingData);
					}
					else
					{
						cout << "Unknown msg_id: " << msg_id << endl;
					}
				}
			};

		serverThread = thread(_server_lambda);
	}

	bool informRobot(Point3D desiredLocation, const uint8_t uid)
	{
		lock_guard<mutex> l(uid_to_robot_mutex);
		optional<shared_ptr<UDPRobot>> robot = getRobotFromUID(uid);
		if (!robot)
			return false;
		ControlData data = robot.value()->calcControlData(desiredLocation);
		sockaddr_storage addr = robot.value()->getAddr();
		return s.write(data.toBytes().data(), ControlData::msg_size, &addr) == SockErr::ERR_OK;

	}

	bool updateKinematics(Point3D c_position, Point3D f_position, const uint8_t uid)
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
		if (serverThread.joinable())
			serverThread.join();
	}
};