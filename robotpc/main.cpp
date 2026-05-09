#include "Locator.hpp"
#include "UDPRobotServer.hpp"
#include "UIDManager.hpp"

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
	UDPRobotServer server;
	server.start();

	LocatorParams params = {
		.camID = 0,
		.width = resolutions[current_resolution][0],
		.height = resolutions[current_resolution][1],
		.zc = 0.72f,
		.thetaDeg = 30.0f,
		.d = 0.0002f,
		.sw = 1.5e-6f,
		.sh = 1.5e-6f,
		.helper = false};

	Locator l(params);

	const Point2D frameCenter{0.f, 0.f};
	const Point3D desiredRobotLocation = l.imagePlaneXYZToFloor(l.imagePlaneUVToImagePlaneXYZ(frameCenter)).value();

	while (l.again(10))
	{
		if (!l.newFrame())
			continue;

		for (uint8_t uid = 0; uid < Config::maxRobotCount(); uid++)
		{
			optional<RobotLEDColors> colors = Config::getColors(uid);
			if (!colors)
				continue;

			optional<Point3D> frontWorld = l.locateMarkAndGet(
				colors.value().frontLow,
				colors.value().frontHigh,
				0.f);
			if (!frontWorld)
				continue;

			optional<Point3D> centerWorld = l.locateMarkAndGet(
				colors.value().centerLow,
				colors.value().centerHigh,
				0.f,
				RegionOfInterest(l.getPixelXY(), 100, 100));
			if (!centerWorld)
				continue;

			if (!server.updateKinematics(centerWorld.value(), frontWorld.value(), uid))
				cout << "Could not update kinematics: " << (int)uid << endl;
			if (!server.informRobot(desiredRobotLocation, uid))
				cout << "Could not inform robot: " << (int)uid << endl;
		}
		l.print();
	}
	server.stop();
}
