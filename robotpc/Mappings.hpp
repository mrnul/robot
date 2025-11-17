#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>

#include "Messages.hpp"

using std::map;
using std::vector;

struct RobotLEDColors
{
	// rgb color
	LEDData	center;

	// grb color
	LEDData	front;

	//hsv colors
	cv::Vec3b centerLow;
	cv::Vec3b centerHigh;

	cv::Vec3b frontLow;
	cv::Vec3b frontHigh;

	bool valid;
};

inline const RobotLEDColors noData = RobotLEDColors(LEDData(0, 0, 0, 0, 0), LEDData(0, 0, 0, 0, 0), false);


class Mappings
{
private:
	static inline const map<uint8_t, RobotLEDColors> UID_TO_ROBOT_COLORS_MAP = {
		{1, {.center = {8, 255, 0, 255, ColorOrder::RGB}, .front = {3, 255, 0, 0, ColorOrder::GRB}, .centerLow = { 130, 20, 145 }, .centerHigh = { 170, 255, 255 }, .frontLow = { 0, 55, 175 }, .frontHigh = { 10, 255, 255 }, .valid = true} },
		{2, {.center = {8, 255, 0, 255, ColorOrder::RGB}, .front = {3, 0, 255, 0, ColorOrder::GRB}, .centerLow = { 130, 20, 145 }, .centerHigh = { 170, 255, 255 }, .frontLow = { 60, 30, 170 }, .frontHigh = { 90, 255, 255 }, .valid = true} },
	};


public:
	static const RobotLEDColors& getColors(const uint8_t uid)
	{
		try
		{
			return UID_TO_ROBOT_COLORS_MAP.at(uid);
		}
		catch (std::out_of_range&)
		{
			return noData;
		}
	}

	static const vector<uint8_t> getAllUIDs()
	{
		vector<uint8_t> colors;
		for (const auto& item : UID_TO_ROBOT_COLORS_MAP)
		{
			colors.push_back(item.first);
		}
		return colors;
	}
};
