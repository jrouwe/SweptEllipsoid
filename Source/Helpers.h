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
// This file contains some small helper functions.

#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <math.h>
#include <malloc.h>

// Swap two numbers
inline void		Swap(float &ioV1, float &ioV2)
{
	float tmp = ioV1;
	ioV1 = ioV2;
	ioV2 = tmp;
}

// Float version of absolute value
inline float	Abs(float inX)
{
	return (float)fabs(inX);
}

// Float version of square root
inline float	Sqrt(float inX)
{
	return (float)sqrt(inX);
}

#endif // _HELPERS_H_