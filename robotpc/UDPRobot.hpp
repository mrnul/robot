#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include "UDPSocket.hpp"
#include "Messages.hpp"
#include "Utils.hpp"

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
	cv::Vec3f c_position;
	cv::Vec3f f_position;
	float theta;

	UDPRobot(const UDPRobot&) = delete;
	UDPRobot& operator=(const  UDPRobot&) = delete;
public:
	UDPRobot(sockaddr_storage addr, const uint8_t uid)
		:
		addr(addr),
		uid(uid),
		c_position(cv::Vec3f()),
		f_position(cv::Vec3f()),
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

	void updatePositionC(const cv::Vec3f& pos, const float lambda)
	{
		c_position = pos * lambda + (1.f - lambda) * c_position;
	}

	void updatePositionF(const cv::Vec3f& pos, const float lambda)
	{
		f_position = pos * lambda + (1.f - lambda) * f_position;
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

	ControlData calcControlData(const cv::Vec3f& desiredLocation)
	{
		const cv::Vec3f direction = f_position - c_position;
		const float dx = f_position[0] - desiredLocation[0];
		const float dy = f_position[1] - desiredLocation[1];

		const float k = 800.f;
		const float c = 0.5f;

		cout << "theta: " << Utils::radToDeg(theta) << endl;

		theta = atan2f(direction[1], direction[0]);
		const ControlData result = controlLaw(k, theta, c, dx, dy);

		cout << "vr: " << result.vr << endl;
		cout << "vl: " << result.vl << endl;
		cout << "===================" << endl;

		return result;
	}
};