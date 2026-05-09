#pragma once

#include <atomic>
#include <iostream>
#include <array>
#include <map>
#include <thread>
#include <memory>
#include <mutex>
#include <optional>

#include "UDPServerSocket.hpp"
#include "Robot.hpp"
#include "UIDManager.hpp"
#include "Config.hpp"
#include "Point.hpp"

using std::array;
using std::atomic;
using std::cout;
using std::endl;
using std::lock_guard;
using std::make_shared;
using std::map;
using std::mutex;
using std::optional;
using std::shared_ptr;
using std::thread;

class UDPRobotServer
{
private:
	static constexpr int BUFFER_SIZE = 1 * 1024;
	static constexpr int READ_READY_TIMEOUT_MS = 2000;
	static constexpr int ROBOT_HB_TIMEOUT_S = 2;

	UDPServerSocket s;

	mutex uid_to_robot_mutex;
	map<uint8_t, shared_ptr<Robot>> uid_to_robot_map;

	UIDManager uidManager;

	thread serverThread;
	atomic<bool> running;

private:
	optional<shared_ptr<Robot>> getRobotFromUID(const uint8_t uid) const
	{
		try
		{
			return uid_to_robot_map.at(uid);
		}
		catch (std::out_of_range &)
		{
			return nullopt;
		}
	}

private:
	void handleHeartbeat(UDPPacket<200> &incomingData)
	{
		cout << "Heartbeat" << endl;
		optional<Heartbeat> msg = Heartbeat::fromBuffer(incomingData.buffer);

		if (!msg)
			return;

		const uint8_t uid = msg.value().uid;
		const uint8_t rssi = msg.value().rssi;
		cout << "\tuid: " << (int)uid << " | rssi: " << (int)rssi << endl;

		lock_guard<mutex> l(uid_to_robot_mutex);
		if (!uid_to_robot_map.contains(uid))
		{
			uid_to_robot_map[uid] = make_shared<Robot>(incomingData.from, uid);
		}
		else
		{
			uid_to_robot_map[uid]->updateLastHBRx();
		}
	}

	void handleTextMessage(UDPPacket<200> &incomingData)
	{
		cout << "TextMessage" << endl;
		optional<TextMessage> msg = TextMessage::fromBuffer(incomingData.buffer);
		if (!msg)
			return;

		const uint8_t uid = msg.value().uid;
		const uint8_t rssi = msg.value().rssi;
		cout << "\tuid: " << (int)uid << " | rssi: " << (int)rssi << " | msg: " << msg.value().message << endl;
	}

	void handleWhoAmI(UDPPacket<200> &incomingData)
	{
		cout << "WhoAmI" << endl;
		optional<WhoAmI> msg = WhoAmI::fromBuffer(incomingData.buffer);
		if (!msg)
			return;

		const uint8_t uid = msg.value().uid;

		cout << "\tuid: " << (int)uid << endl;

		const optional<RobotLEDColors> colors = Config::getColors(uid);
		if (!colors)
			return;

		s.write(colors.value().center.toBytes(), &incomingData.from);
		s.write(colors.value().front.toBytes(), &incomingData.from);

		lock_guard<mutex> l(uid_to_robot_mutex);
		if (uid_to_robot_map.contains(uid))
		{
			cout << "\tRenew: " << (int)uid << endl;
			uid_to_robot_map.erase(uid);
		}
		uid_to_robot_map[uid] = make_shared<Robot>(incomingData.from, uid);
	}

	void handleRequestWhoAmI(UDPPacket<200> &incomingData)
	{
		cout << "RequestWhoAmI" << endl;
		optional<RequestWhoAmI> msg = RequestWhoAmI::fromBuffer(incomingData.buffer);
		if (!msg)
			return;

		const optional<uint8_t> uid = uidManager.getFirstAvailable();
		if (!uid)
		{
			cout << "\tCould not find available UID" << endl;
			return;
		}

		cout << "\tYou are uid: " << (int)uid.value() << endl;

		const optional<RobotLEDColors> colors = Config::getColors(uid.value());
		if (!colors)
		{
			cout << "\tCould not find colors for uid: " << uid.value() << endl;
			return;
		}

		s.write(WhoAmI(uid.value()).toBytes(), &incomingData.from);
		s.write(colors.value().center.toBytes(), &incomingData.from);
		s.write(colors.value().front.toBytes(), &incomingData.from);

		lock_guard<mutex> l(uid_to_robot_mutex);
		if (uid_to_robot_map.contains(uid.value()))
		{
			cout << "\tRenew: " << (int)uid.value() << endl;
			uid_to_robot_map.erase(uid.value());
		}
		uid_to_robot_map[uid.value()] = make_shared<Robot>(incomingData.from, uid.value());
	}

	void performCleanup()
	{
		lock_guard<mutex> l(uid_to_robot_mutex);
		for (auto it = uid_to_robot_map.begin(); it != uid_to_robot_map.end();)
		{
			if (it->second->timeSinceLastHB() >= ROBOT_HB_TIMEOUT_S)
			{
				cout << "Removing robot: " << it->second->getUID() << endl;
				uidManager.releaseUID(it->second->getUID());
				it = uid_to_robot_map.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

public:
	void start()
	{
		if (s.create(8080) != SockErr::ERR_OK)
		{
			cout << "Filed to create server socket" << endl;
			return;
		}
		running.store(true);

		auto _server_lambda = [&]()
		{
			cout << "************* Server running *************" << endl;
			while (running.load())
			{
				performCleanup();
				if (!s.readReady(READ_READY_TIMEOUT_MS))
				{
					cout << "Nothing to receive..." << endl;
					continue;
				}

				UDPPacket<200> incomingData = s.read<200>();
				if (incomingData.count < 4)
				{
					cout << "Too few bytes: " << incomingData.count << endl;
					continue;
				}

				uint32_t msg_id = 0;
				memcpy(&msg_id, incomingData.buffer.data(), 4);
				msg_id = ntohl(msg_id);

				if (msg_id == Heartbeat::id)
				{
					handleHeartbeat(incomingData);
				}
				else if (msg_id == TextMessage::id)
				{
					handleTextMessage(incomingData);
				}
				else if (msg_id == WhoAmI::id)
				{
					handleWhoAmI(incomingData);
				}
				else if (msg_id == RequestWhoAmI::id)
				{
					handleRequestWhoAmI(incomingData);
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
		optional<shared_ptr<Robot>> robot = getRobotFromUID(uid);
		if (!robot)
			return false;
		ControlData data = robot.value()->calcControlData(desiredLocation);
		sockaddr_storage addr = robot.value()->getAddr();
		return s.write(data.toBytes(), &addr) == SockErr::ERR_OK;
	}

	bool updateKinematics(Point3D c_position, Point3D f_position, const uint8_t uid)
	{
		lock_guard<mutex> l(uid_to_robot_mutex);
		optional<shared_ptr<Robot>> robot = getRobotFromUID(uid);
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