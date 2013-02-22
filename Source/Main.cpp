// Example code for: Collision Detection with Swept Spheres and Ellipsoids
// See: http://www.three14.demon.nl/sweptellipsoid/SweptEllipsoid.pdf
//
// Copyright (C) 2003 Jorrit Rouwe
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// This is free software, you can do with it what you want.
//
// This file contains a test program for the swept sphere / ellipsoid routines.

#include <stdio.h>
#include "SweptEllipsoid.h"

#define COLLIDES(x)				if (!(x)) printf("ERROR: " #x " should hold\n");
#define F_IS_NEAR(x, y)			if (Abs((x)-(y)) > 0.01f) printf("ERROR: (%f) is not close to (%f)\n", x, y);
#define V_IS_NEAR(x, y)			if (((x) - (y)).GetLength() > 0.01f) printf("ERROR: (%f %f %f) is not close to (%f %f %f)\n", x.mX, x.mY, x.mZ, y.mX, y.mY, y.mZ);

int main()
{
	// Plane through the origin with normal in the Z direction
	Plane plane;
	plane.mNormal = Vector3(0, 0, 1);
	plane.mConstant = 0;

	// Simple polygon on this plane
	const int cNumVertices = 4;
	Vector2 vertices[cNumVertices] = 
	{
		plane.ConvertWorldToPlane(Vector3(0, 0, 0)),
		plane.ConvertWorldToPlane(Vector3(1, 0, 0)),
		plane.ConvertWorldToPlane(Vector3(1, 1, 0)),
		plane.ConvertWorldToPlane(Vector3(0, 1, 0))
	};

	// Radius of our test sphere
	const float cRadius = 0.1f;

	// Our test ellipsoid, an ellipsoid aligned with the coordinate axis
	const Vector3 cAxis1(0.1f, 0, 0);
	const Vector3 cAxis2(0, 0.2f, 0);
	const Vector3 cAxis3(0, 0, 0.3f);

	// This number makes sure that we don't miss the polygon due to numerical roundoff
	const float cEpsilon = 1.0e-6f;

	// Output parameters
	float fraction;
	Vector3 point;

	// Swept sphere tests
	printf("Performing swept sphere tests\n");

	// Check swept sphere miss (parallel to plane)
	COLLIDES(!PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(0, 0, 0.2f), Vector3(4, 0, 0), cRadius, point, fraction));

	// Check swept sphere miss (perpendicular to plane)
	COLLIDES(!PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(2, 0, 0.2f), Vector3(0, 0, -0.4f), cRadius, point, fraction));

	// Check swept sphere face (embedded initially)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(0, 0, 0), Vector3(0, 0, -1), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.0f);
	V_IS_NEAR(point, Vector3(0, 0, 0));

	// Check swept sphere edge (embedded initially)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(1.1f - cEpsilon, 0.5f, 0), Vector3(0, 0, -1), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.0f);
	V_IS_NEAR(point, Vector3(1.0f, 0.5f, 0));

	// Check swept sphere vertex (embedded initially)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(1.0f + 0.05f - cEpsilon, 1.0f + Sqrt(0.0075f) - cEpsilon, 0), Vector3(0, 0, -1), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.0f);
	V_IS_NEAR(point, Vector3(1.0f, 1.0f, 0));

	// Check swept sphere face (perpendicular to plane)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(0, 0, 0.2f), Vector3(0, 0, -0.4f), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.25f);
	V_IS_NEAR(point, Vector3(0, 0, 0));

	// Check swept sphere edge (perpendicular to plane)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(1.1f - cEpsilon, 0.5f, 0.2f), Vector3(0, 0, -0.4f), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.5f);
	V_IS_NEAR(point, Vector3(1.0f, 0.5f, 0));

	// Check swept sphere vertex (perpendicular to plane)
	// x^2 + y^2 + z^2 = 0.1^2 -> if x = 0.05 and z = 0 -> y = sqrt(0.0075)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(1.0f + 0.05f - cEpsilon, 1.0f + Sqrt(0.0075f) - cEpsilon, 0.2f), Vector3(0, 0, -0.4f), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.5f);
	V_IS_NEAR(point, Vector3(1.0f, 1.0f, 0));

	// Check swept sphere edge (parallel to plane)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(-1, 0.5f, 0.1f), Vector3(4, 0, 0), cRadius, point, fraction));
	F_IS_NEAR(fraction, 0.25f);
	V_IS_NEAR(point, Vector3(0, 0.5f, 0));

	// Check swept sphere vertex (parallel to edge)
	COLLIDES(PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(-5, 0, 0), Vector3(8, 0, 0), 1, point, fraction));
	F_IS_NEAR(fraction, 0.5f);
	V_IS_NEAR(point, Vector3(0, 0, 0));

	// Swept ellipsoid tests
	printf("Performing swept ellipsoid tests\n");

	// Check swept sphere miss (parallel to plane)
	COLLIDES(!PolygonSweptSphereIntersect(plane, vertices, cNumVertices, Vector3(0, 0, 1), Vector3(4, 0, 0), cRadius, point, fraction));

	// Check swept ellipsoid miss (perpendicular to plane)
	COLLIDES(!PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(2, 0, 0.2f), Vector3(0, 0, -0.4f), cAxis1, cAxis2, cAxis3, point, fraction));

	// Check swept ellipsoid face (embedded initially)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(0, 0, 0), Vector3(0, 0, -1), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.0f);
	V_IS_NEAR(point, Vector3(0, 0, 0));

	// Check swept ellipsoid edge (embedded initially)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(1.1f - cEpsilon, 0.5f, 0), Vector3(0, 0, -1), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.0f);
	V_IS_NEAR(point, Vector3(1.0f, 0.5f, 0));

	// Check swept ellipsoid vertex (embedded initially)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(1.0f + 0.05f - cEpsilon, 1.0f + Sqrt(0.03f) - cEpsilon, 0), Vector3(0, 0, -1), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.0f);
	V_IS_NEAR(point, Vector3(1.0f, 1.0f, 0));

	// Check swept ellipsoid face (perpendicular to plane)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(0, 0, 0.6f), Vector3(0, 0, -1.2f), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.25f);
	V_IS_NEAR(point, Vector3(0, 0, 0));

	// Check swept ellipsoid edge (perpendicular to plane)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(1.1f - cEpsilon, 0.5f, 0.2f), Vector3(0, 0, -0.4f), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.5f);
	V_IS_NEAR(point, Vector3(1.0f, 0.5f, 0));

	// Check swept ellipsoid vertex (perpendicular to plane)
	// x^2 / 0.1^2 + y^2 / 0.2^2 + z^2 / 0.3^2 == 1 -> if x = 0.05 and z = 0 -> y = sqrt(0.03)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(1.0f + 0.05f - cEpsilon, 1.0f + Sqrt(0.03f) - cEpsilon, 0.2f), Vector3(0, 0, -0.4f), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.5f);
	V_IS_NEAR(point, Vector3(1.0f, 1.0f, 0));

	// Check swept ellipsoid edge (parallel to plane)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(-1, 0.5f, 0.3f), Vector3(4, 0, 0), cAxis1, cAxis2, cAxis3, point, fraction));
	F_IS_NEAR(fraction, 0.25f);
	V_IS_NEAR(point, Vector3(0, 0.5f, 0));

	// Check swept ellipsoid vertex (parallel to edge)
	COLLIDES(PolygonSweptEllipsoidIntersect(plane, vertices, cNumVertices, Vector3(-5, 0, 0), Vector3(8, 0, 0), Vector3(1, 0, 0), Vector3(0, 2, 0), Vector3(0, 0, 3), point, fraction));
	F_IS_NEAR(fraction, 0.5f);
	V_IS_NEAR(point, Vector3(0, 0, 0));

	printf("Done\n");

	return 1;
}
