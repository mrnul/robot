#pragma once

struct Point2D
{
	float x;
	float y;

	bool operator==(const Point2D& other) const
	{
		return x == other.x && y == other.y;
	}

	Point2D operator*(const float a) const
	{
		return Point2D(x * a, y * a);
	}

	Point2D operator+(const Point2D& other) const
	{
		return Point2D(x + other.x, y + other.y);
	}

	Point2D operator-(const Point2D& other) const
	{
		return Point2D(x - other.x, y - other.y);
	}
};

struct Point3D
{
	float x;
	float y;
	float z;

	bool operator==(const Point3D& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}

	Point3D operator*(const float a) const
	{
		return Point3D(x * a, y * a, z * a);
	}

	Point3D operator+(const Point3D& other) const
	{
		return Point3D(x + other.x, y + other.y, z + other.z);
	}

	Point3D operator-(const Point3D& other) const
	{
		return Point3D(x - other.x, y - other.y, z - other.z);
	}

	void smoothUpdate(const Point3D& other, const float lambda)
	{
		x = other.x * lambda + x * (1.f - lambda);
		y = other.y * lambda + y * (1.f - lambda);
		z = other.z * lambda + z * (1.f - lambda);
	}
};
