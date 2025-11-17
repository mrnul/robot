#pragma once

#include <numbers>

using std::numbers::pi;

inline float radToDeg(const float rad)
{
	return float(rad * 180. / pi);
}

inline float degToRad(const double deg)
{
	return float(deg * pi / 180.);
}
