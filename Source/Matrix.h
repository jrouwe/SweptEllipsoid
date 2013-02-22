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
// Simple 4x4 matrix with lower row [0 0 0 1] used to transform 3D vectors

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include "Vector3.h"

class Matrix
{
public:
	// Access column
	inline const Vector3 &	Column(int inCol) const
	{
		return mColumn[inCol];
	}

	inline Vector3 &		Column(int inCol)
	{
		return mColumn[inCol];
	}

	// Access element
	inline float			E(int inRow, int inCol) const
	{
		return *(&Column(inCol).mX + inRow);
	}

	inline float &			E(int inRow, int inCol)
	{
		return *(&Column(inCol).mX + inRow);
	}

	// Multiply with vector
	inline Vector3			operator * (const Vector3 &inVector) const
	{
		return Vector3(E(0, 0) * inVector.mX + E(0, 1) * inVector.mY + E(0, 2) * inVector.mZ + E(0, 3),
					   E(1, 0) * inVector.mX + E(1, 1) * inVector.mY + E(1, 2) * inVector.mZ + E(1, 3),
					   E(2, 0) * inVector.mX + E(2, 1) * inVector.mY + E(2, 2) * inVector.mZ + E(2, 3));
	}

	// Multiply with matrix
	inline Matrix			operator * (const Matrix &inOther) const
	{
		Matrix mat;

		for (int i = 0; i < 3; i++) 
		{
			mat.E(i, 0) = E(i, 0) * inOther.E(0, 0) + E(i, 1) * inOther.E(1, 0) + E(i, 2) * inOther.E(2, 0);
			mat.E(i, 1) = E(i, 0) * inOther.E(0, 1) + E(i, 1) * inOther.E(1, 1) + E(i, 2) * inOther.E(2, 1);
			mat.E(i, 2) = E(i, 0) * inOther.E(0, 2) + E(i, 1) * inOther.E(1, 2) + E(i, 2) * inOther.E(2, 2);
			mat.E(i, 3) = E(i, 0) * inOther.E(0, 3) + E(i, 1) * inOther.E(1, 3) + E(i, 2) * inOther.E(2, 3) + E(i, 3);
		}

		return mat;		
	}

	// Get transpose of this matrix
	inline Matrix			GetTransposed() const
	{
		Matrix mat;

		for (int c = 0; c < 3; ++c)
			for (int r = 0; r < 3; ++r)
				mat.E(r, c) = E(c, r);

		mat.E(0, 3) = 0.0f;
		mat.E(1, 3) = 0.0f;
		mat.E(2, 3) = 0.0f;

		return mat;
	}

	// Get the inverse of this matrix
	inline Matrix			GetInversed() const
	{
		Matrix mat;
		
		float det =   E(0, 0) * (E(1, 1) * E(2, 2) - E(1, 2) * E(2, 1)) 
		            - E(0, 1) * (E(1, 0) * E(2, 2) - E(1, 2) * E(2, 0)) 
		            + E(0, 2) * (E(1, 0) * E(2, 1) - E(1, 1) * E(2, 0));

		mat.E(0, 0) =  (E(1, 1) * E(2, 2) - E(1, 2) * E(2, 1)) / det;
		mat.E(0, 1) = -(E(0, 1) * E(2, 2) - E(2, 1) * E(0, 2)) / det;
		mat.E(0, 2) =  (E(0, 1) * E(1, 2) - E(1, 1) * E(0, 2)) / det;

		mat.E(1, 0) = -(E(1, 0) * E(2, 2) - E(1, 2) * E(2, 0)) / det;
		mat.E(1, 1) =  (E(0, 0) * E(2, 2) - E(0, 2) * E(2, 0)) / det;
		mat.E(1, 2) = -(E(0, 0) * E(1, 2) - E(0, 2) * E(1, 0)) / det;

		mat.E(2, 0) =  (E(1, 0) * E(2, 1) - E(1, 1) * E(2, 0)) / det;
		mat.E(2, 1) = -(E(0, 0) * E(2, 1) - E(0, 1) * E(2, 0)) / det;
		mat.E(2, 2) =  (E(0, 0) * E(1, 1) - E(0, 1) * E(1, 0)) / det;

		mat.E(0, 3) = -(E(0, 3) * mat.E(0, 0) + E(1, 3) * mat.E(0, 1) + E(2, 3) * mat.E(0, 2));
		mat.E(1, 3) = -(E(0, 3) * mat.E(1, 0) + E(1, 3) * mat.E(1, 1) + E(2, 3) * mat.E(1, 2));
		mat.E(2, 3) = -(E(0, 3) * mat.E(2, 0) + E(1, 3) * mat.E(2, 1) + E(2, 3) * mat.E(2, 2));

		return mat;
	}

private:
	// First three columns contain rotation, last one translation component
	Vector3					mColumn[4];
};

#endif // _MATRIX_H_