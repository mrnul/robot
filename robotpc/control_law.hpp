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

constexpr float MAX_VALUE = 100.f;

Data control_law(const float k, const float theta, const float c, const float dx, const float dy)
{

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

	cout << "v:" << v << endl;
	cout << "w:" << w << endl;
	cout << "vr:" << vr << endl;
	cout << "vl:" << vl << endl;
	cout << "==========" << endl;

	return Data(vr, vl);
}
