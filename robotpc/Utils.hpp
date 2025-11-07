#pragma once

#include <numbers>

using std::numbers::pi;

class Utils
{
public:
	static double radToDeg(const double rad)
	{
		return rad * 180.0 / pi;
	}

	static double degToRad(const double deg)
	{
		return deg * pi / 180.0;
	}
};
