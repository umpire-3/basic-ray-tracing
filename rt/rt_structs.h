#pragma once

struct ray
{
	// Origin
	float ox, oy, oz;
	// Direction
	float dx, dy, dz;
	// Intersection distance
	float maxt;
};


struct sphere
{
	// Center
	float cx, cy, cz;
	// Radius
	float radius;
	// Shpere color
	float r, g, b;

	int id;
	static int IDGen;

	sphere()
		: id(++IDGen)
	{}
};


// Intersect the ray against the sphere
// If there is an intersection:
// * return true
// * update r.maxt to intersection distance
bool intersect_sphere(sphere const& sphere, ray& r);
