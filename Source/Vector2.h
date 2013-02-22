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
// Simple 2D vector class.

#ifndef _VECTOR2_H_
#define _VECTOR2_H_

#include "Helpers.h"

class Vector2
{
public:
	// Constructor
							Vector2(float inX = 0.0f, float inY = 0.0f) : 
		mX(inX), 
		mY(inY) 
	{ 
	}

	// Add vector
	inline Vector2			operator + (const Vector2 &inOther) const
	{
		return Vector2(mX + inOther.mX, mY + inOther.mY);
	}

	// Substract vector
	inline Vector2			operator - (const Vector2 &inOther) const
	{
		return Vector2(mX - inOther.mX, mY - inOther.mY);
	}

	// Multiply with constant
	inline Vector2			operator * (float inConstant) const
	{
		return Vector2(mX * inConstant, mY * inConstant);
	}

	// The dot product
	inline float			Dot(const Vector2 &inOther) const
	{
		return mX * inOther.mX + mY * inOther.mY;
	}

	// Length of the vector
	inline float			GetLengthSquared() const
	{
		return mX * mX + mY * mY;
	}

	// Data
	float					mX, mY;
};

#endif // _VECTOR2_H_