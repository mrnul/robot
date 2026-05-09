#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <optional>

#include "Messages.hpp"

using std::map;
using std::optional;
using std::vector;

struct RobotLEDColors
{
	// rgb color
	LEDData center;

	// grb color
	LEDData front;

	// hsv colors
	cv::Vec3b centerLow;
	cv::Vec3b centerHigh;

	cv::Vec3b frontLow;
	cv::Vec3b frontHigh;
};

class Config
{
private:
	static inline const vector<RobotLEDColors> UID_TO_ROBOT_COLORS_VECTOR = {
		{
			.center = {8, 255, 0, 255, ColorOrder::RGB},
			.front = {3, 255, 0, 0, ColorOrder::GRB},
			.centerLow = {130, 20, 145}, .centerHigh = {170, 255, 255},
			.frontLow = {0, 55, 175},
			.frontHigh = {10, 255, 255}
		},
		{
			.center = {8, 255, 0, 255, ColorOrder::RGB},
			.front = {3, 0, 255, 0, ColorOrder::GRB},
			.centerLow = {130, 20, 145},
			.centerHigh = {170, 255, 255},
			.frontLow = {60, 30, 170},
			.frontHigh = {90, 255, 255}
		}
	};

public:
	static const optional<RobotLEDColors> getColors(const uint8_t uid)
	{
		try
		{
			return UID_TO_ROBOT_COLORS_VECTOR[uid];
		}
		catch (std::out_of_range &)
		{
			return std::nullopt;
		}
	}

	static const int maxRobotCount()
	{
		return UID_TO_ROBOT_COLORS_VECTOR.size();
	}
};
