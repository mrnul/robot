#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>

#include "Utils.hpp"

using std::sin;
using std::cos;
using std::to_string;
using std::string;
using std::chrono::steady_clock;
using std::chrono::duration;
using std::chrono::time_point;
using std::vector;

const cv::Vec2f NotFound2fC(-10000.f, -10000.f);
const cv::Vec3f NotFound3fC(-10000.f, -10000.f, -10000.f);

struct LocatorParams
{
	int width;
	int height;
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
	cv::Vec3f _world_xyz;
	vector<vector<cv::Point>> _contours;

public:
	Locator(const LocatorParams& params)
	{
		setParams(params);

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

			cv::setTrackbarPos("High S", "Helper", 255);
			cv::setTrackbarPos("High V", "Helper", 255);
		}
	}

	void setParams(const LocatorParams& params)
	{
		this->params = params;

		theta = (float)Utils::degToRad(params.theta_deg);
		cos_theta = std::cos(theta);
		sin_theta = std::sin(theta);
		c_xyz = cv::Vec3f(0.f, 0.f, params.zc);
		n_xyz = cv::Vec3f(0.f, cos_theta, -sin_theta);
		pp_xyz = c_xyz + params.d * n_xyz;
		ipy_xyz = cv::Vec3f(0.f, sin_theta, cos_theta);
	}

	bool newFrame()
	{
		if (!videoCapture.read(_frame))
			return false;
		params.width = _frame.size().width;
		params.height = _frame.size().height;

		cv::stackBlur(_frame, _frame, cv::Size(7, 7));
		cvtColor(_frame, _hsv_frame, cv::COLOR_BGR2HSV);
		return true;
	}

	const cv::Vec2f& locatePixelXY(const cv::Vec3b lower_hsv, const cv::Vec3b upper_hsv, const bool useContours = false)
	{
		cv::inRange(_hsv_frame, lower_hsv, upper_hsv, _frame_threshold);

		cv::findContours(_frame_threshold, _contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		cv::Moments m;
		if(useContours)
		{
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
				m = cv::moments(_contours[largest_contour], true);
			}
		}
		else
		{
			m = cv::moments(_frame_threshold, true);
		}

		if (m.m00 != 0.)
		{
			_pixel_xy[0] = float(m.m10 / m.m00);
			_pixel_xy[1] = float(m.m01 / m.m00);
		}
		else
		{
			_pixel_xy = NotFound2fC;
		}
		return _pixel_xy;
	}

	const cv::Vec2f& pixelXYToImagePlaneUV(const cv::Vec2f& pixel_xy)
	{
		_image_plane_uv[0] = (pixel_xy[0] - params.width * 0.5f) * params.sw;
		_image_plane_uv[1] = (params.height * 0.5f - pixel_xy[1]) * params.sh;
		return _image_plane_uv;
	}

	const cv::Vec3f& imagePlaneUVToImagePlaneXYZ(const cv::Vec2f& image_plane_uv)
	{
		_image_plane_xyz = pp_xyz + _image_plane_uv[0] * ipx_xyz + _image_plane_uv[1] * ipy_xyz;
		return _image_plane_xyz;
	}

	const cv::Vec3f& imagePlaneXYZToWorldXYZ(const cv::Vec3f& image_plane_xyz, const float a = 0.f)
	{
		const float denom = params.d * sin_theta - _image_plane_uv[1] * cos_theta;
		if (denom <= 0.f)
		{
			_world_xyz = NotFound3fC;
		}
		else
		{
			const float l = (c_xyz[2] - a) / denom;
			_world_xyz = c_xyz + l * (image_plane_xyz - c_xyz);
		}
		return _world_xyz;
	}

	const cv::Vec3f& locateMarkAndGet(cv::Vec3b lower_hsv, cv::Vec3b upper_hsv, const float a = 0.f, const bool useContours = false)
	{
		locatePixelXY(lower_hsv, upper_hsv, useContours);
		if (_pixel_xy == NotFound2fC)
			return NotFound3fC;

		cv::Mat3b bgrAvgColor;
		cv::Mat3b hsvAvgColor(1, 1, lower_hsv * 0.5f + upper_hsv * 0.5f);
		cv::cvtColor(hsvAvgColor, bgrAvgColor, cv::COLOR_HSV2BGR);
		const cv::Vec3b& avg_color = bgrAvgColor[0][0];
		addCircle(_pixel_xy, avg_color);
		cv::drawContours(_frame, _contours, -1, avg_color, 5);

		pixelXYToImagePlaneUV(_pixel_xy);
		addText(_pixel_xy, cv::format("uv: (%f, %f)", _image_plane_uv[0], _image_plane_uv[1]), avg_color);

		imagePlaneUVToImagePlaneXYZ(_image_plane_uv);
		addText(_pixel_xy + cv::Vec2f(0, 30), cv::format("ip: (%f, %f, %f)", _image_plane_xyz[0], _image_plane_xyz[1], _image_plane_xyz[2]), avg_color);

		imagePlaneXYZToWorldXYZ(_image_plane_xyz, a);
		if (_world_xyz == NotFound3fC)
			return NotFound3fC;
		addText(_pixel_xy + cv::Vec2f(0, 60), cv::format(" r: (%f, %f, %f)", _world_xyz[0], _world_xyz[1], _world_xyz[2]), avg_color);

		return _world_xyz;
	}

	bool again(const int interval_ms) const
	{
		return cv::waitKey(interval_ms) != 27;
	}

	// ======================= Visual things =======================

	void addText(const cv::Point2f xy, const string text, const cv::Scalar color) const
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

	void addLine(const cv::Point2f p1, const cv::Point2f p2, cv::Scalar color) const
	{
		cv::line(
			_frame,
			p1,
			p2,
			color
		);
	}

	void addCircle(const cv::Point2f xy, cv::Scalar color) const
	{
		cv::circle(
			_frame,
			xy,
			8,
			color,
			2
		);
	}

	void print(const bool mark_screen_center = true, const bool mark_horizon = true)
	{
		if (params.helper)
		{
			locateMarkAndGet(helper_low, helper_high, 0.f, true);
		}
		if (mark_screen_center)
		{
			addLine({ params.width / 2.f, 0.f }, { params.width / 2.f, (float)params.height }, { 255, 255, 255 });
			addLine({ 0.f, params.height / 2.f }, { (float)params.width, params.height / 2.f }, { 255, 255, 255 });
		}
		if (mark_horizon)
		{
			const float yh = params.height * 0.5f - params.d * sin_theta / (params.sh * cos_theta);
			addLine({ 0.f, yh }, { (float)params.width, yh }, { 0, 0, 255 });
		}
		const float seconds = duration<float>(steady_clock::now() - time).count();
		const float frames_per_second = 1.f / seconds;
		fps = frames_per_second * LAMBDA_FPS + (1.f - LAMBDA_FPS) * fps;
		addText(cv::Point2f(5, 50), cv::format("(%d, %d) | FPS: %f", params.width, params.height, fps), cv::Scalar(255, 255, 255));

		const float scale = 1280.f / params.width;
		cv::resize(_frame, _frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
		imshow("result", _frame);
		time = steady_clock::now();
	}
};
