#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include "UDPSocket.hpp"
#include "Messages.hpp"
#include "Utils.hpp"
#include "Point.hpp"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::cos;
using std::sin;

class UDPRobot
{
private:
	sockaddr_storage addr;
	int uid;
	Point3D c_position;
	Point3D f_position;
	float theta;

	UDPRobot(const UDPRobot&) = delete;
	UDPRobot& operator=(const  UDPRobot&) = delete;
public:
	UDPRobot(sockaddr_storage addr, const uint8_t uid)
		:
		addr(addr),
		uid(uid),
		c_position(0.f, 0.f),
		f_position(0.f, 0.f),
		theta(0.f)
	{
	}

	int getUID() const
	{
		return uid;
	}

	void setUID(const int uid)
	{
		this->uid = uid;
	}

	void setAddr(sockaddr_storage address)
	{
		addr = address;
	}

	sockaddr_storage getAddr() const
	{
		return addr;
	}

	void updatePositionC(const Point3D& pos, const float lambda)
	{
		c_position.smoothUpdate(pos, lambda);
	}

	void updatePositionF(const Point3D& pos, const float lambda)
	{
		f_position.smoothUpdate(pos, lambda);
	}

	static ControlData controlLaw(const float k, const float theta, const float c, const float dx, const float dy)
	{
		constexpr float MAX_VALUE = 100.f;
		const float v = -k * cos(theta) * dx - k * sin(theta) * dy;
		const float w = k * sin(theta) * dx - k * cos(theta) * dy;

		float vr = v + c * w;
		float vl = v - c * w;

		const float max_abs = max(abs(vr), abs(vl));
		if (max_abs > MAX_VALUE)
		{
			vr /= max_abs;
			vl /= max_abs;

			vr *= MAX_VALUE;
			vl *= MAX_VALUE;
		}

		return ControlData((int32_t)vr, (int32_t)vl);
	}

	ControlData calcControlData(const Point3D& desiredLocation)
	{
		const Point3D direction = f_position - c_position;
		const float dx = f_position.x - desiredLocation.x;
		const float dy = f_position.y - desiredLocation.y;

		const float k = 800.f;
		const float c = 0.5f;

		cout << "theta: " << radToDeg(theta) << endl;

		theta = atan2f(direction.y, direction.x);
		const ControlData result = controlLaw(k, theta, c, dx, dy);

		cout << "vr: " << result.vr << endl;
		cout << "vl: " << result.vl << endl;
		cout << "===================" << endl;

		return result;
	}
};