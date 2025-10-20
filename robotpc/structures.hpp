#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <chrono>

using std::array;
using std::memcpy;
using std::chrono::time_point;
using std::chrono::duration;
using std::chrono::high_resolution_clock;


class Data
{
private:
	const int uid = 1002;
	const float vr = 0.0f;
	const float vl = 0.0f;

	array<unsigned char, 12> bytes = { 0 };

	void to_bytes()
	{
		memcpy(bytes.data(), &uid, 4);
		memcpy(bytes.data() + 4, &vr, 4);
		memcpy(bytes.data() + 8, &vl, 4);
	}
public:
	Data(const float vr, const float vl) : vr(vr), vl(vl)
	{
		to_bytes();
	}
	const unsigned char* get_bytes() const
	{	
		return bytes.data();
	}
};

struct Vec2fT : public cv::Vec2f
{
	time_point<high_resolution_clock> time;

	Vec2fT(double x, double y)
		: cv::Vec2f((float)x, (float)y), time(high_resolution_clock::now())
	{
	}

	Vec2fT(float x, float y)
		: cv::Vec2f(x, y), time(high_resolution_clock::now())
	{
	}

	Vec2fT(cv::Vec2f other)
		: cv::Vec2f(other), time(high_resolution_clock::now())
	{
	}

	Vec2fT()
		: cv::Vec2f(0.f, 0.f), time(high_resolution_clock::now())
	{
	}

	float time_diff_seconds(const Vec2fT& other) const
	{
		return duration<float>(time - other.time).count();
	}
};
