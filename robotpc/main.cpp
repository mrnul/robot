#include "Locator.hpp"
#include "UDPServer.hpp"

const float resolutions[5][2] = {
	{2560.f, 1440.f},
	{1280.f, 720.f},
	{1024.f, 768.f},
	{800.f, 600.f},
	{640.f, 480.f},
};
const int current_resolution = 0;


int main()
{
	UDPServer server;
	server.start();

	LocatorParams params = {
		.width = resolutions[current_resolution][0],
		.height = resolutions[current_resolution][1],
		.zc = 0.72f,
		.theta_deg = 30.0f,
		.d = 0.002f,
		.sw = 1.5e-6f,
		.sh = 1.5e-6f,
		.helper = true
	};

	Locator l(params);

	const cv::Vec3f desired = l.imagePlaneXYZToWorldXYZ(
		l.imagePlaneUVToImagePlaneXYZ(
			cv::Vec2f(0.f, 0.f)
		));

	const vector<uint8_t> uids = Mappings::getAllUIDs();
	while (l.again(10))
	{
		if (!l.newFrame())
			continue;

		for (const uint8_t uid : uids)
		{
			const RobotLEDColors& colors = Mappings::getColors(uid);

			const cv::Vec3f centerReal = l.locateMarkAndGet(colors.centerLow, colors.centerHigh);
			const cv::Vec3f frontReal = l.locateMarkAndGet(colors.frontLow, colors.frontHigh);

			l.print();

			if (centerReal == NotFound3fC)
				continue;

			if (frontReal == NotFound3fC)
				continue;

			if (!server.updateKinematics(centerReal, frontReal, uid))
				cout << "Could not update kinematics" << endl;
			if (!server.informRobot(desired, uid))
				cout << "Could not inform robot" << endl;
		}
	}

	server.stop();
}