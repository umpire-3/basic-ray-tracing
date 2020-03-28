#include <cmath>
#include "rt_structs.h"

int sphere::IDGen = 0;

// Solve quadratic equations and return roots if exist
// Returns true if roots exist and are returned in x1 and x2
// Returns false if no roots exist and x1 and x2 are undefined
bool solve_quadratic(float a, float b, float c, float & x1, float & x2)
{
	float d = b * b - 4 * a*c;

	if (d < 0)
		return false;
	else
	{
		float den = 1 / (2 * a);
		x1 = (-b - std::sqrt(d))*den;
		x2 = (-b + std::sqrt(d))*den;
		return true;
	}
}

bool intersect_sphere(sphere const & sphere, ray & r)
{
	ray rtemp = r;
	rtemp.ox -= sphere.cx;
	rtemp.oy -= sphere.cy;
	rtemp.oz -= sphere.cz;

	float a = rtemp.dx * rtemp.dx + rtemp.dy * rtemp.dy + rtemp.dz * rtemp.dz;
	float b = 2 * (rtemp.ox * rtemp.dx + rtemp.oy * rtemp.dy + rtemp.oz * rtemp.dz);
	float c = rtemp.ox * rtemp.ox + rtemp.oy * rtemp.oy + rtemp.oz * rtemp.oz - sphere.radius * sphere.radius;

	float t0, t1;

	if (solve_quadratic(a, b, c, t0, t1))
	{
		if (t0 > r.maxt || t1 < 0.f)
			return false;

		r.maxt = t0 > 0.f ? t0 : t1;
		return true;
	}

	return false;
}
