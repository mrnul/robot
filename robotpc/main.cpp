#include "server.hpp"
#include "locator.hpp"

const float resolutions[5][2] = {
	{2560.f, 1440.f},
	{1280.f, 720.f},
	{1024.f, 768.f},
	{800.f, 600.f},
	{640.f, 480.f},
};
const int current_resolution = 0;


const cv::Vec3b blue_low = { 100, 150, 230 };
const cv::Vec3b blue_high = { 110, 255, 255 };

const cv::Vec3b red_low = { 0, 130, 200 };
const cv::Vec3b red_high = { 10, 255, 255 };


int main()
{
	Server server;
	server.start();

	LocatorParams params = {
		.width = resolutions[current_resolution][0],
		.height = resolutions[current_resolution][1],
		.zc = 0.72f,
		.theta_deg = 30.0f,
		.d = 0.002f,
		.sw = 1.5e-6f,
		.sh = 1.5e-6f,
		.helper = false
	};

	Locator l(params);

	const cv::Vec3f desired = l.image_plane_xyz_to_real_xyz(
		l.image_plane_uv_to_image_plane_xyz(
			cv::Vec2f(0.f, 0.f)
		));

	while (l.again(10))
	{
		if (!l.new_frame())
			continue;

		const cv::Vec3f blue_real = l.locate_mark_and_get(blue_low, blue_high);
		const cv::Vec3f red_real = l.locate_mark_and_get(red_low, red_high);

		l.print();

		if (blue_real == NotFound3fC)
			continue;

		if (red_real == NotFound3fC)
			continue;

		if (!server.update_kinematics(blue_real, red_real, 1))
			cout << "Could not update kinematics" << endl;
		if (!server.inform_robot(desired, 1))
			cout << "Could not inform robot" << endl;
	}
}