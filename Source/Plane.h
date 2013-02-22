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
// Simple plane equation class.

#ifndef _PLANE_H_
#define _PLANE_H_

#include "Vector3.h"
#include "Matrix.h"

class Plane
{
public:
	// Constructor
							Plane() :
		mConstant(0.0f)
	{
	}

	// Get signed distance to inPoint
	inline float			GetSignedDistance(const Vector3 &inPoint) const
	{
		return inPoint.Dot(mNormal) + mConstant;
	}

	// Get two vectors that together with mNormal form a basis for the plane
	inline void				GetBasisVectors(Vector3 &outU, Vector3 &outV) const
	{ 
		outU = mNormal.GetPerpendicular();
		outV = mNormal.Cross(outU); 
	} 

	// Convert a point from world space to plane space (3D -> 2D)
	inline static Vector2	sConvertWorldToPlane(const Vector3 &inU, const Vector3 &inV, const Vector3 &inPoint)
	{
		return Vector2(inU.Dot(inPoint), inV.Dot(inPoint));
	}

	// Convert a point from world space to plane space (3D -> 2D)
	inline Vector2			ConvertWorldToPlane(const Vector3 &inPoint) const
	{
		Vector3 u, v;
		GetBasisVectors(u, v);
		return sConvertWorldToPlane(u, v, inPoint);
	}

	// Convert a point from plane space to world space (2D -> 3D)
	inline Vector3			ConvertPlaneToWorld(const Vector3 &inU, const Vector3 &inV, const Vector2 &inPoint) const
	{
		return Vector3(inU * inPoint.mX + inV * inPoint.mY - mNormal * mConstant);
	}

	// Get matrix that converts a point from plane space to world space (2D -> 3D)
	inline Matrix			GetPlaneToWorldMatrix() const
	{
		Matrix mat;

		GetBasisVectors(mat.Column(0), mat.Column(1));
		mat.Column(2) = mNormal;
		mat.Column(3) = mNormal * (-mConstant);

		return mat;
	}

	// Transform a plane by the inverse of inInverseMatrix
	inline Plane			GetTransformedByInverse(const Matrix &inInverseMatrix) const
	{ 
		Plane plane;
		
		// Multiply normal by transpose of inverse -> directions are preserved
		plane.mConstant = mConstant + mNormal.Dot(inInverseMatrix.Column(3));
		plane.mNormal = inInverseMatrix.GetTransposed() * mNormal;
		
		// Renormalize normal of plane equation
		float length = plane.mNormal.GetLength();
		plane.mConstant = plane.mConstant / length;
		plane.mNormal = plane.mNormal / length;

		return plane;
	}

	// Plane equation: mNormal.Dot(point) + mConstant == 0
	Vector3					mNormal;
	float					mConstant;
};

#endif // _PLANE_H_