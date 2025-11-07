#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>

#include "Messages.hpp"

using std::map;
using std::vector;

const cv::Vec3b blue_low = { 100, 150, 230 };
const cv::Vec3b blue_high = { 110, 255, 255 };

const cv::Vec3b red_low = { 0, 130, 200 };
const cv::Vec3b red_high = { 10, 255, 255 };

struct RobotLEDColors
{
	// rgb colors
	LEDData	center;
	LEDData	front;

	//hsv colors
	cv::Vec3b centerLow;
	cv::Vec3b centerHigh;

	cv::Vec3b frontLow;
	cv::Vec3b frontHigh;
	bool valid;
};


class Mappings
{
private:
	static inline const map<uint8_t, RobotLEDColors> UID_TO_ROBOT_COLORS_MAP = {
		{1, {.center = {8, 0, 0, 255}, .front = {3, 0, 30, 0}, .centerLow = { 90, 100, 190 }, .centerHigh = { 130, 255, 255 }, .frontLow = { 160, 60, 120 }, .frontHigh = { 200, 255, 255 }, .valid = true} },
	};


public:
	static const RobotLEDColors getColors(const uint8_t uid)
	{
		try
		{
			return UID_TO_ROBOT_COLORS_MAP.at(uid);
		}
		catch (std::out_of_range&)
		{
			return RobotLEDColors(LEDData(0, 0, 0, 0), LEDData(0, 0, 0, 0), false);
		}
	}

	static const vector<uint8_t> getAllUIDs()
	{
		vector<uint8_t> colors;
		for (auto& item : UID_TO_ROBOT_COLORS_MAP)
		{
			colors.push_back(item.first);
		}
		return colors;
	}
};
