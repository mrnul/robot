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
using std::chrono::steady_clock;
using std::chrono::duration;
using std::chrono::time_point;

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
	bool helper;
};


class Locator
{
private:
	// static const variables

	// unit vector x on image plane is always the same
	inline static const cv::Vec3f ipx_xyz = cv::Vec3f(1.f, 0.f, 0.f);

	// just to smooth the fps calculation
	inline static const float LAMBDA_FPS = 0.1f;

private:
	// parameters and things that depend on parameters
	LocatorParams params;

	float theta;
	float cos_theta;
	float sin_theta;

	cv::Vec3f c_xyz;
	cv::Vec3f n_xyz;
	cv::Vec3f pp_xyz;
	cv::Vec3f ipy_xyz;

private:
	// things to help us
	float fps;

	cv::Vec3b helper_low;
	cv::Vec3b helper_high;

	time_point<steady_clock> time;

private:
	// to capture images from the camera
	cv::VideoCapture videoCapture;

	// keep these here instead of allocating them each time
	cv::Mat _frame;
	cv::Mat _hsv_frame;
	cv::Mat _frame_threshold;
	cv::Vec2f _pixel_xy;
	cv::Vec2f _image_plane_uv;
	cv::Vec3f _image_plane_xyz;
	cv::Vec3f _real_xyz;
	vector<vector<cv::Point>> _contours;

public:
	Locator(const LocatorParams& params)
	{
		set_params(params);

		videoCapture.open(0, cv::CAP_ANY);
		videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, params.width);
		videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, params.height);

		if (params.helper)
		{
			cv::namedWindow("Helper", cv::WINDOW_AUTOSIZE);

			cv::createTrackbar("Low H", "Helper", nullptr, 255, [](int pos, void* obj) {*(unsigned char*)obj = pos; }, &helper_low[0]);
			cv::createTrackbar("High H", "Helper", nullptr, 255, [](int pos, void* obj) {*(unsigned char*)obj = pos; }, &helper_high[0]);

			cv::createTrackbar("Low S", "Helper", nullptr, 255, [](int pos, void* obj) {*(unsigned char*)obj = pos; }, &helper_low[1]);
			cv::createTrackbar("High S", "Helper", nullptr, 255, [](int pos, void* obj) {*(unsigned char*)obj = pos; }, &helper_high[1]);

			cv::createTrackbar("Low V", "Helper", nullptr, 255, [](int pos, void* obj) {*(unsigned char*)obj = pos; }, &helper_low[2]);
			cv::createTrackbar("High V", "Helper", nullptr, 255, [](int pos, void* obj) {*(unsigned char*)obj = pos; }, &helper_high[2]);
		}
	}

	void set_params(const LocatorParams& params)
	{
		this->params = params;

		theta = (float)deg_to_rad(params.theta_deg);
		cos_theta = std::cos(theta);
		sin_theta = std::sin(theta);
		c_xyz = cv::Vec3f(0.f, 0.f, params.zc);
		n_xyz = cv::Vec3f(0.f, cos_theta, -sin_theta);
		pp_xyz = c_xyz + params.d * n_xyz;
		ipy_xyz = cv::Vec3f(0.f, sin_theta, cos_theta);
	}

	bool new_frame()
	{
		if (!videoCapture.read(_frame))
			return false;
		cvtColor(_frame, _hsv_frame, cv::COLOR_BGR2HSV);
		return true;
	}

	const cv::Vec2f& locate_pixel_xy(const cv::Vec3b lower_hsv, const cv::Vec3b upper_hsv)
	{
		cv::inRange(_hsv_frame, lower_hsv, upper_hsv, _frame_threshold);
		cv::findContours(_frame_threshold, _contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		double max_area = 0;
		int largest_contour = -1;
		for (int i = 0; i < _contours.size(); i++)
		{
			const double area = cv::contourArea(_contours[i]);
			if (area > max_area)
			{
				max_area = area;
				largest_contour = i;
			}
		}

		if (largest_contour >= 0)
		{
			cv::Moments m = cv::moments(_contours[largest_contour], true);
			_pixel_xy[0] = float(m.m10 / m.m00);
			_pixel_xy[1] = float(m.m01 / m.m00);
		}
		else {
			_pixel_xy = NotFound2fC;
		}

		return _pixel_xy;
	}

	const cv::Vec2f& pixel_xy_to_image_plane_uv(const cv::Vec2f& pixel_xy)
	{
		_image_plane_uv[0] = (pixel_xy[0] - params.width * 0.5f) * params.sw;
		_image_plane_uv[1] = (params.height * 0.5f - pixel_xy[1]) * params.sh;
		return _image_plane_uv;
	}

	const cv::Vec3f& image_plane_uv_to_image_plane_xyz(const cv::Vec2f& image_plane_uv)
	{
		_image_plane_xyz = pp_xyz + _image_plane_uv[0] * ipx_xyz + _image_plane_uv[1] * ipy_xyz;
		return _image_plane_xyz;
	}

	const cv::Vec3f& image_plane_xyz_to_real_xyz(const cv::Vec3f& image_plane_xyz, const float a = 0.f)
	{
		const float denom = params.d * sin_theta - _image_plane_uv[1] * cos_theta;
		if (denom <= 0.f)
		{
			_real_xyz = NotFound3fC;
		}
		else
		{
			const float l = (c_xyz[2] - a) / denom;
			_real_xyz = c_xyz + l * (image_plane_xyz - c_xyz);
		}
		return _real_xyz;
	}

	const cv::Vec3f& locate_mark_and_get(cv::Vec3b lower_hsv, cv::Vec3b upper_hsv, const float a = 0.f)
	{
		locate_pixel_xy(lower_hsv, upper_hsv);
		if (_pixel_xy == NotFound2fC)
			return NotFound3fC;

		cv::Mat3b bgrAvgColor;
		cv::Mat3b hsvAvgColor(1, 1, lower_hsv * 0.5f + upper_hsv * 0.5f);
		cv::cvtColor(hsvAvgColor, bgrAvgColor, cv::COLOR_HSV2BGR);
		const cv::Vec3b& avg_color = bgrAvgColor[0][0];
		add_circle(_pixel_xy, avg_color);
		cv::drawContours(_frame, _contours, -1, avg_color, 5);

		pixel_xy_to_image_plane_uv(_pixel_xy);
		add_text(_pixel_xy, cv::format("uv: (%f, %f)", _image_plane_uv[0], _image_plane_uv[1]), avg_color);

		image_plane_uv_to_image_plane_xyz(_image_plane_uv);
		add_text(_pixel_xy + cv::Vec2f(0, 30), cv::format("ip: (%f, %f, %f)", _image_plane_xyz[0], _image_plane_xyz[1], _image_plane_xyz[2]), avg_color);

		image_plane_xyz_to_real_xyz(_image_plane_xyz, a);
		if (_real_xyz == NotFound3fC)
			return NotFound3fC;
		add_text(_pixel_xy + cv::Vec2f(0, 60), cv::format(" r: (%f, %f, %f)", _real_xyz[0], _real_xyz[1], _real_xyz[2]), avg_color);

		return _real_xyz;
	}

	bool again(const int interval_ms) const
	{
		return cv::waitKey(interval_ms) != 27;
	}

	// ======================= Visual things =======================

	void add_text(const cv::Point2f xy, const string text, const cv::Scalar color) const
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

	void add_line(const cv::Point2f p1, const cv::Point2f p2, cv::Scalar color) const
	{
		cv::line(
			_frame,
			p1,
			p2,
			color
		);
	}

	void add_circle(const cv::Point2f xy, cv::Scalar color) const
	{
		cv::circle(
			_frame,
			xy,
			8,
			color,
			2
		);
	}

	void print(const bool mark_screen_center = true, const bool mark_horizon = true, const bool show_fps = true)
	{
		if (params.helper)
		{
			locate_mark_and_get(helper_low, helper_high);
		}
		if (show_fps)
		{
			const float seconds = duration<float>(steady_clock::now() - time).count();
			const float frames_per_second = 1.f / seconds;
			fps = frames_per_second * LAMBDA_FPS + (1.f - LAMBDA_FPS) * fps;
			add_text(cv::Point2f(5, 50), "FPS: " + to_string(fps), cv::Scalar(255, 255, 255));
			time = steady_clock::now();
		}
		if (mark_screen_center)
		{
			add_line({ params.width / 2, 0 }, { params.width / 2, params.height }, { 255, 255, 255 });
			add_line({ 0, params.height / 2 }, { params.width, params.height / 2 }, { 255, 255, 255 });
		}
		if (mark_horizon)
		{
			const float yh = params.height * 0.5f - params.d * sin_theta / (params.sh * cos_theta);
			add_line({ 0, yh }, { params.width, yh }, { 0, 0, 255 });
		}
		const float scale = 1280.f / params.width;
		cv::resize(_frame, _frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
		imshow("result", _frame);
	}
};