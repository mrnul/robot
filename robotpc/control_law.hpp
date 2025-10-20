#pragma once

#include <algorithm>
#include <cmath>

#include "structures.hpp"

using std::cout;
using std::endl;
using std::max;
using std::sin;
using std::cos;
using std::abs;

Data control_law(const float k, const float theta, const float c, const float dx, const float dy)
{
	const float v = -k * cos(theta) * dx - k * sin(theta) * dy;
	const float w = k * sin(theta) * dx - k * cos(theta) * dy;

	float vr = v + c * w;
	float vl = v - c * w;

	const float max_abs = max(abs(vr), abs(vl));
	if (max_abs > 100.f)
	{
		vr /= max_abs;
		vl /= max_abs;

		vr *= 100.f;
		vl *= 100.f;
	}

	cout << "v:" << v << endl;
	cout << "w:" << w << endl;
	cout << "vr:" << vr << endl;
	cout << "vl:" << vl << endl;
	cout << "==========" << endl;

	return Data(vr, vl);
}
