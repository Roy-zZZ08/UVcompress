#ifndef UTIL_H
#define UTIL_H

#include <Misha/Geometry.h>

#include <algorithm>

void GetTriangleIntegerBBox2(Point2D< double > tPos[2], const double invCellSizeW, const double invCellSizeH, int minCorner[2], int maxCorner[2])
{
	double fminx = std::min< double >(tPos[0][0], tPos[1][0]);
	fminx = std::max< double >(fminx, 0.);
	double fminy = std::min< double >(tPos[0][1], tPos[1][1]);
	fminy = std::max< double >(fminy, 0.);
	double fmaxx = std::max< double >(tPos[0][0], tPos[1][0]);
	fmaxx = std::min< double >(fmaxx, 1.);
	double fmaxy = std::max< double >(tPos[0][1], tPos[1][1]);
	fmaxy = std::min< double >(fmaxy, 1.);

	minCorner[0] = static_cast<int>(floor(fminx * invCellSizeW));
	minCorner[1] = static_cast<int>(floor(fminy * invCellSizeH));
	maxCorner[0] = static_cast<int>(ceil(fmaxx * invCellSizeW));
	maxCorner[1] = static_cast<int>(ceil(fmaxy * invCellSizeH));
}

void GetTriangleIntegerBBox3(Point2D< double > tPos[3], const double invCellSizeW, const double invCellSizeH, int minCorner[2], int maxCorner[2])
{
	double fminx = std::min< double >(std::min< double >(tPos[0][0], tPos[1][0]), tPos[2][0]);
	fminx = std::max< double >(fminx, 0.);
	double fminy = std::min< double >(std::min< double >(tPos[0][1], tPos[1][1]), tPos[2][1]);
	fminy = std::max< double >(fminy, 0.);
	double fmaxx = std::max< double >(std::max< double >(tPos[0][0], tPos[1][0]), tPos[2][0]);
	fmaxx = std::min< double >(fmaxx, 1.);
	double fmaxy = std::max< double >(std::max< double >(tPos[0][1], tPos[1][1]), tPos[2][1]);
	fmaxy = std::min< double >(fmaxy, 1.);

	minCorner[0] = static_cast< int >( floor( fminx * invCellSizeW ));
	minCorner[1] = static_cast< int >( floor( fminy * invCellSizeH ));
	maxCorner[0] = static_cast< int >( ceil( fmaxx * invCellSizeW ));
	maxCorner[1] = static_cast< int >( ceil( fmaxy * invCellSizeH ));
}

SquareMatrix< double, 2 > GetBarycentricMap(Point2D< double > tPos[3])
{
	SquareMatrix< double, 2 > parametrizationMap;
	parametrizationMap.coords[0][0] = tPos[1][0] - tPos[0][0];
	parametrizationMap.coords[0][1] = tPos[1][1] - tPos[0][1];
	parametrizationMap.coords[1][0] = tPos[2][0] - tPos[0][0];
	parametrizationMap.coords[1][1] = tPos[2][1] - tPos[0][1];
	return parametrizationMap.inverse();
}

Point2D< double > FindFoot(Point2D< double > const pntStart, Point2D< double > const pntEnd, Point2D< double > const pA)
{
	Point2D< double >  pFoot;
	double k = 0.0;
	if (pntStart[0] == pntEnd[0])
	{
		pFoot[0] = pntStart[0];
		pFoot[1] = pA[1];
		return pFoot;
	}
	k = (pntEnd[1] - pntStart[1]) * 1.0 / (pntEnd[0] - pntStart[0]);
	double A = k;
	double B = -1.0;
	double C = pntStart[1] - k * pntStart[0];

	pFoot[0] = (B * B * pA[0] - A * B * pA[1] - A * C) / (A * A + B * B);
	pFoot[1] = (A * A * pA[1] - A * B * pA[0] - B * C) / (A * A + B * B);

	return pFoot;
}

#endif