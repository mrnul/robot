#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <chrono>

using std::array;
using std::memcpy;
using std::chrono::time_point;
using std::chrono::duration;
using std::chrono::high_resolution_clock;

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
