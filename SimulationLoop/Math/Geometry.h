#pragma once
#include "Vector2D.h"

//--------------------LineIntersection2D-------------------------
//
//	Given 2 lines in 2D space AB, CD this returns true if an 
//	intersection occurs.
//	Author: Mat Buckland (fup@ai-junkie.com)
//----------------------------------------------------------------- 

inline bool HasLineIntersection2D(
	math::Vector2D A, 
	math::Vector2D B, 
	math::Vector2D C, 
	math::Vector2D D)
{
	double rTop = (A.y - C.y)*(D.x - C.x) - (A.x - C.x)*(D.y - C.y);
	double sTop = (A.y - C.y)*(B.x - A.x) - (A.x - C.x)*(B.y - A.y);

	double Bot = (B.x - A.x)*(D.y - C.y) - (B.y - A.y)*(D.x - C.x);

	if (Bot == 0) return false;	//parallel

	double invBot = 1.0 / Bot;
	double r = rTop * invBot;
	double s = sTop * invBot;

	if ((r > 0) && (r < 1) && (s > 0) && (s < 1)) return true;	//lines intersect
	
	return false;	//lines do not intersect
}

//-------------------- LineIntersection2D-------------------------
//
//	Given 2 lines in 2D space AB, CD this returns true if an 
//	intersection occurs and sets dist to the distance the intersection
//  occurs along AB. Also sets the 2d vector point to the point of
//  intersection
//	Author: Mat Buckland (fup@ai-junkie.com)
//----------------------------------------------------------------- 
inline bool HasLineIntersection2D(
	math::Vector2D		A,
	math::Vector2D		B,
	math::Vector2D		C,
	math::Vector2D		D,
	double&				dist,
	math::Vector2D&		point)
{
	double rTop = (A.y - C.y)*(D.x - C.x) - (A.x - C.x)*(D.y - C.y);
	double rBot = (B.x - A.x)*(D.y - C.y) - (B.y - A.y)*(D.x - C.x);

	double sTop = (A.y - C.y)*(B.x - A.x) - (A.x - C.x)*(B.y - A.y);
	double sBot = (B.x - A.x)*(D.y - C.y) - (B.y - A.y)*(D.x - C.x);

	if ((rBot == 0) || (sBot == 0))
	{
		return false;	//lines are parallel
	}

	double r = rTop / rBot;
	double s = sTop / sBot;

	if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
	{
		dist = math::distance(A, B) * r;

		point = A + r * (B - A);

		return true;
	}
	else
	{
		dist = 0;

		return false;
	}
}