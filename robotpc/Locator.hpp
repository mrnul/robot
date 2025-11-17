#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>

#include "Utils.hpp"
#include "Point.hpp"

using std::sin;
using std::cos;
using std::to_string;
using std::string;
using std::chrono::steady_clock;
using std::chrono::duration;
using std::chrono::time_point;
using std::vector;

const Point2D NotFound2fC(-10000.f, -10000.f);
const Point3D NotFound3fC(-10000.f, -10000.f, -10000.f);

struct LocatorParams
{
	int width;
	int height;
	float zc;
	float thetaDeg;
	float d;
	float sw;
	float sh;
	bool helper;
};


struct RegionOfInterest
{
	cv::Rect rect;
	bool valid;

	RegionOfInterest() : valid(false)
	{
	}

	RegionOfInterest(const Point2D& center, const int width = 0, const int height = 0)
		: rect(int(center.x - width / 2.f), int(center.y - height / 2.f), width, height), valid(width > 0 && height > 0)
	{
	}

	void ensureWithinImage(const cv::Mat& img)
	{
		if (!valid)
			return;

		if (rect.x < 0)
			rect.x = 0;
		if (rect.y < 0)
			rect.y = 0;

		if (rect.x + rect.width > img.size().width)
			rect.x = img.size().width - rect.width;

		if (rect.y + rect.height > img.size().height)
			rect.y = img.size().height - rect.height;
	}
};


class Locator
{
private:
	// static const variables

	// unit vector x on image plane is always the same
	inline static const Point3D ipx_xyz = Point3D(1.f, 0.f, 0.f);

	// just to smooth the fps calculation
	inline static const float LAMBDA_FPS = 0.1f;

private:
	// parameters and things that depend on parameters
	LocatorParams params;

	float theta;
	float cosTheta;
	float sinTheta;

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
	Point2D _pixel_xy;
	Point2D _image_plane_uv;
	Point3D _image_plane_xyz;
	Point3D _world_xyz;
	cv::Vec3b avgColor;
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

		theta = degToRad(params.thetaDeg);
		cosTheta = std::cos(theta);
		sinTheta = std::sin(theta);
	}

	bool newFrame()
	{
		if (!videoCapture.read(_frame))
			return false;
		params.width = _frame.size().width;
		params.height = _frame.size().height;

		cv::stackBlur(_frame, _frame, cv::Size(5, 5));
		cvtColor(_frame, _hsv_frame, cv::COLOR_BGR2HSV);
		return true;
	}

	const Point2D& locatePixelXY(const cv::Vec3b lower_hsv, const cv::Vec3b upper_hsv, const bool useContours, RegionOfInterest& roi)
	{
		if (roi.valid)
		{
			roi.ensureWithinImage(_hsv_frame);
			cv::inRange(_hsv_frame(roi.rect), lower_hsv, upper_hsv, _frame_threshold);
		}
		else
		{
			cv::inRange(_hsv_frame, lower_hsv, upper_hsv, _frame_threshold);
		}

		cv::Moments m;
		if (useContours)
		{
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
				cv::drawContours(_frame, _contours, -1, avgColor, 3, 8, cv::noArray(), INT_MAX, { roi.rect.x, roi.rect.y });
				m = cv::moments(_contours[largest_contour], true);
			}
		}
		else
		{
			m = cv::moments(_frame_threshold, true);
		}

		if (m.m00 != 0.)
		{
			_pixel_xy.x = float(m.m10 / m.m00 + roi.rect.x);
			_pixel_xy.y = float(m.m01 / m.m00 + roi.rect.y);
		}
		else
		{
			_pixel_xy = NotFound2fC;
		}
		return _pixel_xy;
	}

	const Point2D& pixelXYToImagePlaneUV(const Point2D& pixel_xy)
	{
		_image_plane_uv.x = (pixel_xy.x - params.width * 0.5f) * params.sw;
		_image_plane_uv.y = (params.height * 0.5f - pixel_xy.y) * params.sh;
		return _image_plane_uv;
	}

	const Point3D& imagePlaneUVToImagePlaneXYZ(const Point2D& image_plane_uv)
	{
		_image_plane_xyz.x = _image_plane_uv.x;
		_image_plane_xyz.y = params.d * cosTheta + _image_plane_uv.y * sinTheta;
		_image_plane_xyz.z = params.zc + _image_plane_uv.y * cosTheta - params.d * sinTheta;

		return _image_plane_xyz;
	}

	const Point3D& imagePlaneXYZToWorldXYZ(const Point3D& image_plane_xyz, const float a = 0.f)
	{
		const float denom = params.d * sinTheta - _image_plane_uv.y * cosTheta;
		if (denom <= 0.f)
		{
			_world_xyz = NotFound3fC;
			return _world_xyz;
		}
		const float l = (params.zc - a) / denom;
		if (l <= 1.f)
		{
			_world_xyz = NotFound3fC;
			return _world_xyz;
		}

		_world_xyz.x = l * _image_plane_uv.x;
		_world_xyz.y = l * (params.d * cosTheta + _image_plane_uv.y * sinTheta);
		_world_xyz.z = a;

		return _world_xyz;
	}

	const Point3D& locateMarkAndGet(cv::Vec3b lower_hsv, cv::Vec3b upper_hsv, const float a, const bool useContours, RegionOfInterest roi = {})
	{
		cv::Mat3b bgrAvgColor;
		cv::Mat3b hsvAvgColor(1, 1, lower_hsv * 0.5f + upper_hsv * 0.5f);
		cv::cvtColor(hsvAvgColor, bgrAvgColor, cv::COLOR_HSV2BGR);
		avgColor = bgrAvgColor[0][0];

		locatePixelXY(lower_hsv, upper_hsv, useContours, roi);
		if (_pixel_xy == NotFound2fC)
			return NotFound3fC;
		addCircle(_pixel_xy, avgColor);

		pixelXYToImagePlaneUV(_pixel_xy);
		addText(_pixel_xy, cv::format("uv: (%f, %f)", _image_plane_uv.x, _image_plane_uv.y), avgColor);

		imagePlaneUVToImagePlaneXYZ(_image_plane_uv);
		addText(_pixel_xy, cv::format("ip: (%f, %f, %f)", _image_plane_xyz.x, _image_plane_xyz.y, _image_plane_xyz.z), avgColor, 30);

		imagePlaneXYZToWorldXYZ(_image_plane_xyz, a);
		if (_world_xyz == NotFound3fC)
			return NotFound3fC;
		addText(_pixel_xy, cv::format(" r: (%f, %f, %f)", _world_xyz.x, _world_xyz.y, _world_xyz.z), avgColor, 60);

		return _world_xyz;
	}

	const Point3D& getWorldXYZ() const
	{
		return _world_xyz;
	}

	const Point2D& getPixelXY() const
	{
		return _pixel_xy;
	}

	bool again(const int interval_ms) const
	{
		return cv::waitKey(interval_ms) != 27;
	}

	// ======================= Visual things =======================

	void addText(const Point2D xy, const string text, const cv::Scalar color, const int offsetY = 0) const
	{
		cv::putText(
			_frame,
			text,
			{ (int)xy.x, (int)xy.y + offsetY },
			cv::FONT_HERSHEY_SIMPLEX,
			0.6,
			color,
			1,
			cv::LINE_4
		);
	}

	void addLine(const Point2D p1, const Point2D p2, cv::Scalar color) const
	{
		cv::line(
			_frame,
			{ (int)p1.x, (int)p1.y },
			{ (int)p2.x, (int)p2.y },
			color
		);
	}

	void addCircle(const Point2D xy, cv::Scalar color) const
	{
		cv::circle(
			_frame,
			{ (int)xy.x, (int)xy.y },
			8,
			color,
			1
		);
	}

	void print()
	{
		if (params.helper)
		{
			locateMarkAndGet(helper_low, helper_high, 0.f, true, {});
		}

		addLine({ params.width / 2.f, 0.f }, { params.width / 2.f, (float)params.height }, { 255, 255, 255 });
		addLine({ 0.f, params.height / 2.f }, { (float)params.width, params.height / 2.f }, { 255, 255, 255 });

		const float yh = params.height * 0.5f - params.d * sinTheta / (params.sh * cosTheta);
		addLine({ 0.f, yh }, { (float)params.width, yh }, { 0, 0, 255 });

		const float seconds = duration<float>(steady_clock::now() - time).count();
		const float frames_per_second = 1.f / seconds;
		fps = frames_per_second * LAMBDA_FPS + (1.f - LAMBDA_FPS) * fps;
		addText(Point2D(5, 50), cv::format("(%d, %d) | FPS: %f", params.width, params.height, fps), cv::Scalar(255, 255, 255));

		const float scale = 1280.f / params.width;
		cv::resize(_frame, _frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
		imshow("result", _frame);
		time = steady_clock::now();
	}
};
