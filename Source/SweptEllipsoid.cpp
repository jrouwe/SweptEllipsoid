// Example code for: Collision Detection with Swept Spheres and Ellipsoids
// See: https://github.com/jrouwe/SweptEllipsoid
//
// Copyright (C) 2003 Jorrit Rouwe
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// This is free software, you can do with it what you want.
//
// This file contains the main swept sphere / ellipsoid intersection tests.
//
// Please have a look at the notes. They indicate obvious places for optimization 
// if you are using a swept ellipsoid against a large number of polygons.

#include "SweptEllipsoid.h"

// Test intersection between a plane inPlane and a swept sphere with radius inRadius moving from inBegin to inBegin + inDelta
// If there is an intersection the function returns true and the intersection range is from 
// inBegin + outT1 * inDelta to inBegin + outT2 * inDelta
bool PlaneSweptSphereIntersect(const Plane &inPlane, const Vector3 &inBegin, const Vector3 &inDelta, float inRadius, float &outT1, float &outT2)
{
	// If the center of the sphere moves like: center = inBegin + t * inDelta for t e [0, 1]
	// then the sphere intersects the plane if: -R <= distance plane to center <= R
	float n_dot_d = inPlane.mNormal.Dot(inDelta);
	float dist_to_b = inPlane.GetSignedDistance(inBegin);
	if (n_dot_d == 0.0f)
	{
		// The sphere is moving nearly parallel to the plane, check if the distance
		// is smaller than the radius
		if (Abs(dist_to_b) > inRadius)
			return false;

		// Intersection on the entire range
		outT1 = 0.0f;
		outT2 = 1.0f;
	}
	else
	{
		// Determine interval of intersection
		outT1 = (inRadius - dist_to_b) / n_dot_d;
		outT2 = (-inRadius - dist_to_b) / n_dot_d;

		// Order the results
		if (outT1 > outT2) 
			Swap(outT1, outT2);

		// Early out if no hit possible
		if (outT1 > 1.0f || outT2 < 0.0f)
			return false;
		
		// Clamp it to the range [0, 1], the range of the swept sphere
		if (outT1 < 0.0f) outT1 = 0.0f;
		if (outT2 > 1.0f) outT2 = 1.0f;
	}

	return true;
}

// Check if a polygon contains inPoint, returns true when it does
bool PolygonContains(const Vector2 *inVertices, int inNumVertices, const Vector2 &inPoint)
{
	// Loop through edges
	for (const Vector2 *v1 = inVertices, *v2 = inVertices + inNumVertices - 1; v1 < inVertices + inNumVertices; v2 = v1, ++v1)
	{
		// If the point is outside this edge, the point is outside the polygon
		Vector2 v1_v2 = *v2 - *v1;
		Vector2 v1_point = inPoint - *v1;
		if (v1_v2.mX * v1_point.mY - v1_point.mX * v1_v2.mY > 0.0f)
			return false;
	}

	return true;
}

// Check if circle at inCenter with radius^2 = inRadiusSq intersects with a polygon.
// Function returns true when it does and the intersection point is in outPoint
bool PolygonCircleIntersect(const Vector2 *inVertices, int inNumVertices, const Vector2 &inCenter, float inRadiusSq, Vector2 &outPoint)
{
	// Check if the center is inside the polygon
	if (PolygonContains(inVertices, inNumVertices, inCenter))
	{
		outPoint = inCenter;
		return true;
	}

	// Loop through edges
	bool collision = false;
	for (const Vector2 *v1 = inVertices, *v2 = inVertices + inNumVertices - 1; v1 < inVertices + inNumVertices; v2 = v1, ++v1)
	{
		// Get fraction where the closest point to this edge occurs
		Vector2 v1_v2 = *v2 - *v1;
		Vector2 v1_center = inCenter - *v1;
		float fraction = v1_center.Dot(v1_v2);
		if (fraction < 0.0f)
		{
			// Closest point is v1
			float dist_sq = v1_center.GetLengthSquared();
			if (dist_sq <= inRadiusSq)
			{
				collision = true;
				outPoint = *v1;
				inRadiusSq = dist_sq;
			}
		}
		else 
		{
			float v1_v2_len_sq = v1_v2.GetLengthSquared();
			if (fraction <= v1_v2_len_sq)
			{
				// Closest point is on line segment
				Vector2 point = *v1 + v1_v2 * (fraction / v1_v2_len_sq);
				float dist_sq = (point - inCenter).GetLengthSquared();
				if (dist_sq <= inRadiusSq)
				{
					collision = true;
					outPoint = point;
					inRadiusSq = dist_sq;
				}
			}
		}
	}

	return collision;
}

// Solve the equation inA * x^2 + inB * x + inC == 0 for the lowest x in [0, inUpperBound].
// Returns true if there is such a solution and returns the solution in outX
bool FindLowestRootInInterval(float inA, float inB, float inC, float inUpperBound, float &outX)
{
	// Check if a solution exists
	float determinant = inB * inB - 4.0f * inA * inC;
	if (determinant < 0.0f)
		return false;

	// The standard way of doing this is by computing: x = (-b +/- Sqrt(b^2 - 4 a c)) / 2 a 
	// is not numerically stable when a is close to zero. 
	// Solve the equation according to "Numerical Recipies in C" paragraph 5.6
	float q = -0.5f * (inB + (inB < 0.0f? -1.0f : 1.0f) * Sqrt(determinant));

	// Both of these can return +INF, -INF or NAN that's why we test both solutions to be in the specified range below
	float x1 = q / inA;
	float x2 = inC / q;

	// Order the results
	if (x2 < x1)
		Swap(x1, x2);

	// Check if x1 is a solution
	if (x1 >= 0.0f && x1 <= inUpperBound)
	{
		outX = x1;
		return true;
	}

	// Check if x2 is a solution
	if (x2 >= 0.0f && x2 <= inUpperBound)
	{
		outX = x2;
		return true;
	}

	return false;
}

// Checks intersection between a polygon an moving circle at inBegin + t * inDelta with radius^2 = inA * t^2 + inB * t + inC, t in [0, 1]
// Returns true when it does and returns the intersection position in outPoint and the intersection fraction (value for t) in outFraction
bool SweptCircleEdgeVertexIntersect(const Vector2 *inVertices, int inNumVertices, const Vector2 &inBegin, const Vector2 &inDelta, float inA, float inB, float inC, Vector2 &outPoint, float &outFraction)
{
	// Loop through edges
	float upper_bound = 1.0f;
	bool collision = false;
	for (const Vector2 *v1 = inVertices, *v2 = inVertices + inNumVertices - 1; v1 < inVertices + inNumVertices; v2 = v1, ++v1)
	{
		float t;
		
		// Check if circle hits the vertex
		Vector2 bv1 = *v1 - inBegin;
		float a1 = inA - inDelta.GetLengthSquared();				
		float b1 = inB + 2.0f * inDelta.Dot(bv1);
		float c1 = inC - bv1.GetLengthSquared();
		if (FindLowestRootInInterval(a1, b1, c1, upper_bound, t))
		{
			// We have a collision
			collision = true;
			upper_bound = t;
			outPoint = *v1;
		}

		// Check if circle hits the edge
		Vector2 v1v2 = *v2 - *v1;
		float v1v2_dot_delta = v1v2.Dot(inDelta);
		float v1v2_dot_bv1 = v1v2.Dot(bv1);
		float v1v2_len_sq = v1v2.GetLengthSquared();
		float a2 = v1v2_len_sq * a1 + v1v2_dot_delta * v1v2_dot_delta;
		float b2 = v1v2_len_sq * b1 - 2.0f * v1v2_dot_bv1 * v1v2_dot_delta;
		float c2 = v1v2_len_sq * c1 + v1v2_dot_bv1 * v1v2_dot_bv1;
		if (FindLowestRootInInterval(a2, b2, c2, upper_bound, t))
		{
			// Check if the intersection point is on the edge
			float f = t * v1v2_dot_delta - v1v2_dot_bv1;
			if (f >= 0.0f && f <= v1v2_len_sq)
			{
				// We have a collision
				collision = true;
				upper_bound = t;
				outPoint = *v1 + v1v2 * (f / v1v2_len_sq);
			}
		}
	}

	// Check if we had a collision
	if (!collision)
		return false;
	outFraction = upper_bound;
	return true;
}

// Test between a polygon and a swept sphere with radius inRadius moving from inBegin to inBegin + inDelta
// If there is an intersection the intersection position is returned in outPoint and the center of the
// sphere is at inBegin + outFraction * inDelta when it collides
bool PolygonSweptSphereIntersect(const Plane &inPlane, const Vector2 *inVertices, int inNumVertices, const Vector3 &inBegin, const Vector3 &inDelta, float inRadius, Vector3 &outPoint, float &outFraction)
{
	// Determine the range over which the sphere intersects the plane
	float t1, t2;
	if (!PlaneSweptSphereIntersect(inPlane, inBegin, inDelta, inRadius, t1, t2))
		return false;

	// The radius of the circle is defined as: radius^2 = (sphere radius)^2 - (distance plane to center)^2
	// this can be written as: radius^2 = a * t^2 + b * t + c
	float n_dot_d = inPlane.mNormal.Dot(inDelta);
	float dist_to_b = inPlane.GetSignedDistance(inBegin);
	float a = -n_dot_d * n_dot_d;
	float b = -2.0f * n_dot_d * dist_to_b;
	float c = inRadius * inRadius - dist_to_b * dist_to_b;

	// Get basis
	Vector3 u, v;
	inPlane.GetBasisVectors(u, v);

	// Get begin and delta in plane space
	Vector2 begin = Plane::sConvertWorldToPlane(u, v, inBegin);
	Vector2 delta = Plane::sConvertWorldToPlane(u, v, inDelta);

	// Test if sphere intersects at t1
	Vector2 p;
	if (PolygonCircleIntersect(inVertices, inNumVertices, begin + delta * t1, a * t1 * t1 + b * t1 + c, p))
	{
		outFraction = t1;
		outPoint = inPlane.ConvertPlaneToWorld(u, v, p);
		return true;
	}
	
	// Test if sphere intersects with one of the edges or vertices
	if (SweptCircleEdgeVertexIntersect(inVertices, inNumVertices, begin, delta, a, b, c, p, outFraction))
	{
		outPoint = inPlane.ConvertPlaneToWorld(u, v, p);
		return true;
	}

	return false;
}

// Multiply matrix with 2D vector (multiply with (x, y, 0))
Vector2 Transform2x2(const Matrix &inMatrix, const Vector2 &inVector)
{
	return Vector2(inMatrix.E(0, 0) * inVector.mX + inMatrix.E(0, 1) * inVector.mY,
				   inMatrix.E(1, 0) * inVector.mX + inMatrix.E(1, 1) * inVector.mY);
}

// Test intersection with a swept ellipsoid with principal axis inAxis1, inAxis2, inAxis3 moving from inBegin to inBegin + inDelta
// If there is an intersection the intersection position is returned in outPoint and the center of the
// sphere is at inBegin + outFraction * inDelta when it collides
bool PolygonSweptEllipsoidIntersect(const Plane &inPlane, const Vector2 *inVertices, int inNumVertices, const Vector3 &inBegin, const Vector3 &inDelta, const Vector3 &inAxis1, const Vector3 &inAxis2, const Vector3 &inAxis3, Vector3 &outPoint, float &outFraction)
{
	// Compute matrix that takes a point from unit sphere space to world space
	// NOTE: When colliding with lots of polygons this can be cached
	Matrix unit_sphere_to_world;
	unit_sphere_to_world.Column(0) = inAxis1;
	unit_sphere_to_world.Column(1) = inAxis2;
	unit_sphere_to_world.Column(2) = inAxis3;

	// Compute matrix that takes a point from world space to unit sphere space
	// NOTE: When colliding with lots of polygons this can be cached
	Matrix world_to_unit_sphere = unit_sphere_to_world.GetInversed();

	// Compute begin and delta in unit sphere space
	// NOTE: When colliding with lots of polygons this can be cached
	Vector3 begin_uss = world_to_unit_sphere * inBegin;
	Vector3 delta_uss = world_to_unit_sphere * inDelta;

	// Transform the plane into unit sphere local space
	Plane transformed_plane;
	transformed_plane = inPlane.GetTransformedByInverse(unit_sphere_to_world);

	// Determine the range over which the unit sphere intersects the transformed plane
	float t1, t2;
	if (!PlaneSweptSphereIntersect(transformed_plane, begin_uss, delta_uss, 1.0f, t1, t2))
		return false;

	// Get matrix that transforms a point from plane space to world space
	Matrix plane_to_world = inPlane.GetPlaneToWorldMatrix();

	// Get matrix that transforms a point from the transformed plane to unit sphere space
	Matrix transformed_plane_to_unit_sphere = transformed_plane.GetPlaneToWorldMatrix();

	// Get matrix that takes a 2d polygon vertex from the original space to the space of the
	// transformed plane so that the unit sphere is still a unit sphere
	Matrix plane_to_transformed_plane = transformed_plane_to_unit_sphere.GetInversed() * world_to_unit_sphere * plane_to_world;

	// The radius of the circle is defined as: radius^2 = 1 - (distance plane to center)^2
	// this can be written as: radius^2 = a * t^2 + b * t + c
	float n_dot_d = transformed_plane.mNormal.Dot(delta_uss);
	float dist_to_b = transformed_plane.GetSignedDistance(begin_uss);
	float a = -n_dot_d * n_dot_d;
	float b = -2.0f * n_dot_d * dist_to_b;
	float c = 1.0f - dist_to_b * dist_to_b;
	
	// Get the basis vectors for the transformed plane
	const Vector3 &u = transformed_plane_to_unit_sphere.Column(0);
	const Vector3 &v = transformed_plane_to_unit_sphere.Column(1);

	// To avoid translating the polygon we subtract the translation from the begin point 
	// and then later add it to the collision result again
	Vector2 trans(plane_to_transformed_plane.E(0, 3), plane_to_transformed_plane.E(1, 3));
	
	// Get the equation for the intersection circle between the plane and the unit sphere: center = begin + t * delta
	Vector2 begin = Plane::sConvertWorldToPlane(u, v, begin_uss) - trans;
	Vector2 delta = Plane::sConvertWorldToPlane(u, v, delta_uss);

	// Transform the polygon
	Vector2 *transformed_vertices = (Vector2 *)alloca(inNumVertices * sizeof(Vector2));
	for (int i = 0; i < inNumVertices; ++i)
		transformed_vertices[i] = Transform2x2(plane_to_transformed_plane, inVertices[i]);
	
	// Test if sphere intersects at t1
	Vector2 p;
	if (PolygonCircleIntersect(transformed_vertices, inNumVertices, begin + delta * t1, a * t1 * t1 + b * t1 + c, p))
	{
		outFraction = t1;
		outPoint = unit_sphere_to_world * (transformed_plane_to_unit_sphere * Vector3(p + trans));
		return true;
	}
	
	// Test if sphere intersects with one of the edges or vertices
	if (SweptCircleEdgeVertexIntersect(transformed_vertices, inNumVertices, begin, delta, a, b, c, p, outFraction))
	{
		outPoint = unit_sphere_to_world * (transformed_plane_to_unit_sphere * Vector3(p + trans));
		return true;
	}

	return false;
}

