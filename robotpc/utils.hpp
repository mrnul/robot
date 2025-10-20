#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

double rad_to_deg(const double rad)
{
	return rad * 180.0 / M_PI;
}

double deg_to_rad(const double deg)
{
	return deg * M_PI / 180.0;
}
