#pragma once

#include <cmath>
#include <optional>
#include <chrono>
#include "socket_client.hpp"
#include "utils.hpp"
#include "structures.hpp"
#include "messages.hpp"
#include "control_law.hpp"

#pragma comment(lib, "Ws2_32.lib")

using std::chrono::duration;
using std::atan2f;
using std::optional;
using std::nullopt;

class Robot
{
private:
	SocketClient client;
	int uid;
	Vec2fT c_position;
	Vec2fT f_position;
	float theta;
public:
	Robot(SOCKET socket)
		:
		client(socket),
		uid(0),
		c_position(Vec2fT()),
		f_position(Vec2fT()),
		theta(0.f)
	{
	}

	int get_uid() const
	{
		return uid;
	}

	void set_uid(const int uid)
	{
		this->uid = uid;
	}

	void update_position_c(const Vec2fT& pos, const float lambda)
	{
		c_position = pos * lambda + (1.f - lambda) * c_position;
	}

	void update_position_f(const Vec2fT& pos, const float lambda)
	{
		f_position = pos * lambda + (1.f - lambda) * f_position;
	}

	void update_theta(const float lambda)
	{

		const Vec2fT cur_direction = f_position - c_position;
		const float new_theta = atan2f(cur_direction[1], cur_direction[0]);

		theta = new_theta * lambda + (1.f - lambda) * theta;
	}

	int calc_and_send_data(Vec2fT& desired_location)
	{
		const float dx = f_position[0] - desired_location[0];
		const float dy = f_position[1] - desired_location[1];

		const float k = 600.f;
		const float c = 0.5f;

		cout << "theta: " << rad_to_deg(theta) << endl;

		Data result = control_law(k, theta, c, dx, dy);
		client.insert_tx_data(result.to_bytes().data(), result.size());
		return client.send_all_pending_data();
	}

	optional<Heartbeat> get_heartbeat()
	{
		const optional<int8_t> rssi = client.peek_byte(4);
		if (rssi == nullopt)
			return nullopt;

		client.remove_rx_bytes(5);
		return Heartbeat(rssi.value());
	}

	optional<TextMessage> get_text_message()
	{
		const optional<int32_t> msg_length = client.peek_int32(4);
		if (msg_length == nullopt)
			return nullopt;
		const optional<string> msg = client.peek_string(msg_length.value(), 8);
		if (msg == nullopt)
			return nullopt;
		const optional<int8_t> rssi = client.peek_byte(8 + msg_length.value());
		if (rssi == nullopt)
			return nullopt;

		client.remove_rx_bytes(8 + msg_length.value() + 1);
		return TextMessage(msg.value().c_str(), rssi.value());
	}

	optional<WhoAmI> get_who_am_i()
	{
		const optional<uint32_t> uid = client.peek_int32(4);
		if (uid == nullopt)
			return nullopt;
		set_uid(uid.value());
		client.remove_rx_bytes(8);
		return WhoAmI(uid.value());
	}

	int read_all_available_data()
	{
		return client.read_available_data();
	}

	optional<int32_t> peek_msg_id()
	{
		return client.peek_int32();
	}

	bool is_alive() const
	{
		return client.is_alive();
	}
};
