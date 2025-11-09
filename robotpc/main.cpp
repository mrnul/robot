#include "Locator.hpp"
#include "UDPServer.hpp"

const int resolutions[5][2] = {
	{2560, 1440},
	{1280, 720},
	{1024, 768},
	{800, 600},
	{640, 480},
};
const int current_resolution = 1;


int main()
{
	UDPServer server;
	server.start();

	LocatorParams params = {
		.width = resolutions[current_resolution][0],
		.height = resolutions[current_resolution][1],
		.zc = 0.72f,
		.thetaDeg = 30.0f,
		.d = 0.002f,
		.sw = 1.5e-6f,
		.sh = 1.5e-6f,
		.helper = false
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

			const cv::Vec3f frontWorld = l.locateMarkAndGet(colors.frontLow, colors.frontHigh, 0.f, true);
			if (frontWorld == NotFound3fC)
				continue;

			const cv::Vec3f centerWorld = l.locateMarkAndGet(colors.centerLow, colors.centerHigh, 0.f, true, RegionOfInterest(l.getPixelXY(), 100, 100));
			if (centerWorld == NotFound3fC)
				continue;

			if (!server.updateKinematics(centerWorld, frontWorld, uid))
				cout << "Could not update kinematics: " << (int)uid << endl;
			if (!server.informRobot(desired + cv::Vec3f(uid == 1? -0.15f : 0.15f, 0.f, 0.f), uid))
				cout << "Could not inform robot: " << (int)uid << endl;
		}
		l.print();
	}

	server.stop();
}