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
// Simple 3D vector class.

#ifndef _VECTOR3_H_
#define _VECTOR3_H_

#include "Vector2.h"

class Vector3
{
public:
	// Constructor
							Vector3(float inX = 0.0f, float inY = 0.0f, float inZ = 0.0f) : 
		mX(inX), 
		mY(inY),
		mZ(inZ)
	{ 
	}

							Vector3(const Vector2 &inOther) :
		mX(inOther.mX),
		mY(inOther.mY),
		mZ(0.0f)
	{
	}
		
	// Add vector
	inline Vector3			operator + (const Vector3 &inOther) const
	{
		return Vector3(mX + inOther.mX, mY + inOther.mY, mZ + inOther.mZ);
	}

	// Substract vector
	inline Vector3			operator - (const Vector3 &inOther) const
	{
		return Vector3(mX - inOther.mX, mY - inOther.mY, mZ - inOther.mZ);
	}

	// Multiply with constant
	inline Vector3			operator * (float inConstant) const
	{
		return Vector3(mX * inConstant, mY * inConstant, mZ * inConstant);
	}

	// Divide by constant
	inline Vector3			operator / (float inConstant) const
	{
		return Vector3(mX / inConstant, mY / inConstant, mZ / inConstant);
	}

	// Dot product
	inline float			Dot(const Vector3 &inOther) const
	{
		return mX * inOther.mX + mY * inOther.mY + mZ * inOther.mZ;
	}

	// Length of the vector
	inline float			GetLengthSquared() const
	{
		return mX * mX + mY * mY + mZ * mZ;
	}

	inline float			GetLength() const
	{
		return Sqrt(GetLengthSquared());
	}

	// The cross product
	inline Vector3			Cross(const Vector3 &inOther) const
	{
		return Vector3(mY * inOther.mZ - mZ * inOther.mY, 
					   mZ * inOther.mX - mX * inOther.mZ, 
					   mX * inOther.mY - mY * inOther.mX); 
	}

	// Perpendicular vector
	inline Vector3			GetPerpendicular() const
	{
		if (Abs(mX) > Abs(mY))
		{
			float len = Sqrt(mX * mX + mZ * mZ);		
			return Vector3(mZ / len, 0.0f, -mX / len);
		}
		else
		{
			float len = Sqrt(mY * mY + mZ * mZ);		
			return Vector3(0.0f, mZ / len, -mY / len);
		}
	}

	// Data
	float					mX, mY, mZ;
};

#endif // _VECTOR3_H_