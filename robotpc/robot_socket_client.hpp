#pragma once

#include <cmath>
#include <chrono>
#include "client.hpp"
#include "utils.hpp"
#include "structures.hpp"
#include "circular_buffer.hpp"
#include "control_law.hpp"

#pragma comment(lib, "Ws2_32.lib")

using std::chrono::duration;
using std::atan2f;

constexpr int POS_COUNT = 2;
constexpr int VEL_COUNT = 2;
constexpr int THETA_COUNT = 2;

class RobotSocketClient : public Client
{
private:
	int uid;
	CircularBuffer<Vec2fT> c_positions; // latest at [0], oldest at [POS_COUNT - 1]
	CircularBuffer<Vec2fT> f_positions; // latest at [0], oldest at [POS_COUNT - 1]
	CircularBuffer<Vec2fT> c_velocities; // latest at [0], oldest at [VEL_COUNT - 1]
	CircularBuffer<float> thetas; // latest at [0], oldest at [THETA_COUNT - 1]

	template<typename T>
	void insert_data(T& data, CircularBuffer<T>& where, const float lambda)
	{
		// data =  data * lambda + (1.f - lambda) * data[0];
		data *= lambda;
		data += (1.f - lambda) * where[0];

		where.insert(data);
	}
public:
	RobotSocketClient(SOCKET socket)
		:
		Client(socket),
		uid(0),
		c_positions(CircularBuffer<Vec2fT>(POS_COUNT)),
		f_positions(CircularBuffer<Vec2fT>(POS_COUNT)),
		c_velocities(CircularBuffer<Vec2fT>(VEL_COUNT)),
		thetas(CircularBuffer<float>(THETA_COUNT))
	{
	}

	bool send_data(Data data) const
	{
		return send_n(data.get_bytes(), 12) == 12;
	}

	int get_uid() const
	{
		return uid;
	}

	void set_uid(const int uid)
	{
		this->uid = uid;
	}

	void update_position_c(Vec2fT& pos, const float lambda)
	{
		insert_data(pos, c_positions, lambda);
	}

	void update_position_f(Vec2fT& pos, const float lambda)
	{
		insert_data(pos, f_positions, lambda);
	}

	void update_theta(const float lambda)
	{
		const Vec2fT& cur_c_pos = get_c_position(0);
		const Vec2fT& cur_f_pos = get_f_position(0);

		const Vec2fT cur_direction = cur_f_pos - cur_c_pos;
		float new_theta = atan2f(cur_direction[1], cur_direction[0]);

		insert_data(new_theta, thetas, lambda);
	}

	bool calc_and_send_data(Vec2fT& desired_location)
	{
		const Vec2fT& current_f_location = get_f_position();
		const float theta = get_theta();

		const float dx = current_f_location[0] - desired_location[0];
		const float dy = current_f_location[1] - desired_location[1];

		const float k = 1000.f;
		const float c = 0.5f;

		return send_data(control_law(k, theta, c, dx, dy));
	}

	Vec2fT& get_velocity(const int i = 0)
	{
		return c_velocities[i >= 0 ? i : -i];
	}

	Vec2fT& get_c_position(const int i = 0)
	{
		return c_positions[i >= 0 ? i : -i];
	}

	Vec2fT& get_f_position(const int i = 0)
	{
		return f_positions[i >= 0 ? i : -i];
	}

	float get_theta(const int i = 0)
	{
		return thetas[i >= 0 ? i : -i];
	}
};

