#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include "utils.hpp"

using std::sin;
using std::cos;
using std::to_string;
using namespace std::chrono;

const cv::Vec2f NotFound2fC(-10000.f, -10000.f);
const cv::Vec3f NotFound3fC(-10000.f, -10000.f, -10000.f);

struct LocatorParams
{
	float width;
	float height;
	float zc;
	float theta_deg;
	float d;
	float sw;
	float sh;
};

class Locator
{
private:
	const LocatorParams params;

	const float theta;
	const float cos_theta;
	const float sin_theta;
	const float tan_theta;

	const cv::Vec3f c_xyz;
	const cv::Vec3f n_xyz;
	const cv::Vec3f pp_xyz;

	cv::VideoCapture videoCapture;

private:
	cv::Mat _frame;
	cv::Mat _hsv_frame;
	cv::Mat _frame_threshold;
	cv::Vec2f _pixel_xy;
	cv::Vec2f _image_plane_xy;
	cv::Vec3f _image_plane_xyz;
	cv::Vec3f _real_xyz;

	time_point<high_resolution_clock> time;
public:
	Locator(const LocatorParams params) :
		params(params),
		theta((float)deg_to_rad(params.theta_deg)),
		cos_theta(std::cos(theta)),
		sin_theta(std::sin(theta)),
		tan_theta(std::tan(theta)),
		c_xyz(cv::Vec3f(0.f, 0.f, params.zc)),
		n_xyz(cv::Vec3f(0.f, cos_theta, -sin_theta)),
		pp_xyz(c_xyz + params.d * n_xyz)
	{
		videoCapture.open(0, cv::CAP_ANY);
		videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, params.width);
		videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, params.height);
	}

	bool new_frame(const bool blur = true)
	{
		if (!videoCapture.read(_frame))
			return false;
		if (blur)
			cv::stackBlur(_frame, _frame, cv::Size(25, 25));
		cvtColor(_frame, _hsv_frame, cv::COLOR_BGR2HSV);
		return true;
	}

	const cv::Vec2f& locate_pixel_xy(const cv::Vec3b lower_hsv, const cv::Vec3b upper_hsv)
	{
		cv::inRange(_hsv_frame, lower_hsv, upper_hsv, _frame_threshold);
		cv::Moments m = cv::moments(_frame_threshold);

		if (m.m00 == 0.0)
		{
			_pixel_xy = NotFound2fC;
		}
		else
		{
			_pixel_xy[0] = float(m.m10 / m.m00);
			_pixel_xy[1] = float(m.m01 / m.m00);
		}
		return _pixel_xy;
	}

	const cv::Vec2f& pixel_xy_to_image_plane_xy(const cv::Vec2f& pixel_xy)
	{
		_image_plane_xy[0] = (pixel_xy[0] - params.width * 0.5f) * params.sw;
		_image_plane_xy[1] = (params.height * 0.5f - pixel_xy[1]) * params.sh;
		return _image_plane_xy;
	}

	const cv::Vec3f& image_plane_xy_to_image_plane_xzy(const cv::Vec2f& image_plane_xy)
	{
		_image_plane_xyz[0] = image_plane_xy[0];
		_image_plane_xyz[1] = pp_xyz[1] + image_plane_xy[1] * sin_theta;
		_image_plane_xyz[2] = pp_xyz[2] + image_plane_xy[1] * cos_theta;
		return _image_plane_xyz;
	}

	const cv::Vec3f& image_plane_xyz_to_real_xyz(const cv::Vec3f& image_plane_xyz)
	{
		if (image_plane_xyz[2] >= c_xyz[2])
		{
			_real_xyz = NotFound3fC;
		}
		else
		{
			const float l = c_xyz[2] / (c_xyz[2] - image_plane_xyz[2]);
			_real_xyz = c_xyz + l * (image_plane_xyz - c_xyz);
		}
		return _real_xyz;
	}

	const cv::Vec3f& locate_mark_and_get(cv::Vec3b lower_hsv, cv::Vec3b upper_hsv)
	{
		locate_pixel_xy(lower_hsv, upper_hsv);
		if (_pixel_xy == NotFound2fC)
			return NotFound3fC;

		cv::Mat3b bgrAvgColor;
		cv::Mat3b hsvAvgColor(1, 1, lower_hsv * 0.5f + upper_hsv * 0.5f);
		cv::cvtColor(hsvAvgColor, bgrAvgColor, cv::COLOR_HSV2BGR);
		const cv::Vec3b& avg_color = bgrAvgColor[0][0];
		add_circle(_pixel_xy, avg_color);

		pixel_xy_to_image_plane_xy(_pixel_xy);
		add_text(_pixel_xy + cv::Vec2f(0, 0), "ip: (" + to_string(_image_plane_xy[0]) + ", " + to_string(_image_plane_xy[1]) + ")", avg_color);

		image_plane_xy_to_image_plane_xzy(_image_plane_xy);
		add_text(_pixel_xy + cv::Vec2f(0, 30), "ip: (" + to_string(_image_plane_xyz[0]) + ", " + to_string(_image_plane_xyz[1]) + ", " + to_string(_image_plane_xyz[2]) + ")", avg_color);

		image_plane_xyz_to_real_xyz(_image_plane_xyz);
		if (_real_xyz == NotFound3fC)
			return NotFound3fC;
		add_text(_pixel_xy + cv::Vec2f(0, 60), " r: (" + to_string(_real_xyz[0]) + ", " + to_string(_real_xyz[1]) + ", " + to_string(_real_xyz[2]) + ")", avg_color);
		return _real_xyz;
	}

	bool again(const int interval_ms) const
	{
		return cv::waitKey(interval_ms) != 27;
	}

	// ======================= Visual things =======================

	void add_text(const cv::Point2f xy, const string text, const cv::Vec3i color)
	{
		cv::putText(
			_frame,
			text,
			xy,
			cv::FONT_HERSHEY_SIMPLEX,
			0.6,
			color,
			2,
			cv::LINE_4
		);
	}

	void add_line(const cv::Point2f p1, const cv::Point2f p2, cv::Vec3i color)
	{
		cv::line(
			_frame,
			p1,
			p2,
			color
		);
	}

	void add_circle(const cv::Point2f xy, cv::Vec3i color)
	{
		cv::circle(
			_frame,
			xy,
			8,
			color,
			2
		);
	}

	void print(const bool mark_screen_center = true, const bool mark_horizon = true, const bool fps = true)
	{
		if (fps)
		{
			const float seconds = (high_resolution_clock::now() - time).count() * 1e-9f;
			const float frames_per_second = 1.f / seconds;
			add_text(cv::Point2f(5, 50), "FPS: " + to_string(frames_per_second), cv::Vec3i(255, 255, 255));
			time = high_resolution_clock::now();
		}
		if (mark_screen_center)
		{
			add_line({ params.width / 2, 0 }, { params.width / 2, params.height }, { 255, 255, 255 });
			add_line({ 0, params.height / 2 }, { params.width, params.height / 2 }, { 255, 255, 255 });
		}
		if (mark_horizon)
		{
			const float yh = params.height / 2 - params.d * tan_theta / params.sh;
			add_line({ 0, yh }, { params.width, yh }, { 0, 0, 255 });
		}
		const float scale = 1280.f / params.width;
		cv::resize(_frame, _frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
		imshow("result", _frame);
	}
};