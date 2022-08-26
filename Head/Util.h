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

void GetTriangleIntegerBBox4(Point2D< double > tPos[4], const double invCellSizeW, const double invCellSizeH, int minCorner[2], int maxCorner[2])
{
	double fminx = std::min< double >(std::min< double >(tPos[0][0], tPos[1][0]), std::min< double >(tPos[2][0], tPos[3][0]));
	fminx = std::max< double >(fminx, 0.);
	double fminy = std::min< double >(std::min< double >(tPos[0][1], tPos[1][1]), std::min< double >(tPos[2][1], tPos[3][1]));
	fminy = std::max< double >(fminy, 0.);
	double fmaxx = std::max< double >(std::max< double >(tPos[0][0], tPos[1][0]), std::max< double >(tPos[2][0], tPos[3][0]));
	fmaxx = std::min< double >(fmaxx, 1.);
	double fmaxy = std::max< double >(std::max< double >(tPos[0][1], tPos[1][1]), std::max< double >(tPos[2][1], tPos[3][1]));
	fmaxy = std::min< double >(fmaxy, 1.);

	minCorner[0] = static_cast<int>(floor(fminx * invCellSizeW));
	minCorner[1] = static_cast<int>(floor(fminy * invCellSizeH));
	maxCorner[0] = static_cast<int>(ceil(fmaxx * invCellSizeW));
	maxCorner[1] = static_cast<int>(ceil(fmaxy * invCellSizeH));
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

Point3D< float > BilinearSample(Image<Point3D< float > >& img, Point2D< double >index)
{
	Point2D< double > nearNode = Point2D< double >(round(index[0]), round(index[1]));
	std::vector<Point2D< double > > cellCenters;
	cellCenters.push_back(Point2D< double >(nearNode[0] - 0.5, nearNode[1] - 0.5));
	cellCenters.push_back(Point2D< double >(nearNode[0] + 0.5, nearNode[1] - 0.5));
	cellCenters.push_back(Point2D< double >(nearNode[0] + 0.5, nearNode[1] + 0.5));
	cellCenters.push_back(Point2D< double >(nearNode[0] - 0.5, nearNode[1] + 0.5));

	std::vector<Point3D< float > > colors;
	colors.push_back(img(floor(cellCenters[0][0]), floor(cellCenters[0][1])));
	colors.push_back(img(floor(cellCenters[1][0]), floor(cellCenters[1][1])));
	colors.push_back(img(floor(cellCenters[2][0]), floor(cellCenters[2][1])));
	colors.push_back(img(floor(cellCenters[3][0]), floor(cellCenters[3][1])));

	Point3D< float > color1 = (float)(index[1] - cellCenters[0][1]) * colors[3] + (float)(1 - (index[1] - cellCenters[0][1])) * colors[0];
	Point3D< float > color2 = (float)(index[1] - cellCenters[0][1]) * colors[2] + (float)(1 - (index[1] - cellCenters[0][1])) * colors[1];

	Point3D< float > color3= (float)(index[0] - cellCenters[0][0]) * color2 + (float)(1 - (index[0] - cellCenters[0][0])) * color1;


	return color3;
}


bool InsideQuad(Point2D< double > quadPos[4], Point2D< double > p)
{
	
	Point2D< double > AB = quadPos[1] - quadPos[0];
	Point2D< double > BC = quadPos[2] - quadPos[1];
	Point2D< double > CD = quadPos[3] - quadPos[2];
	Point2D< double > DA = quadPos[0] - quadPos[3];

	int flag = -1;
	double direction0 = Point2D< double >::Cross(AB, BC);
	double direction1 = Point2D< double >::Cross(BC, CD);
	double direction2 = Point2D< double >::Cross(CD, DA);
	double direction3 = Point2D< double >::Cross(DA, AB);

	if (direction0 * direction1 * direction2 * direction3) {
		// convex polygon
		Point2D< double > AP = p - quadPos[0];
		Point2D< double > BP = p - quadPos[1];
		Point2D< double > CP = p - quadPos[2];
		Point2D< double > DP = p - quadPos[3];

		if (Point2D< double >::Cross(AB, AP) > 0) return false;
		if (Point2D< double >::Cross(BC, BP) > 0) return false;
		if (Point2D< double >::Cross(CD, CP) > 0) return false;
		if (Point2D< double >::Cross(DA, DP) > 0) return false;

		return true;
	}
	else {
		return true;
	}

	// todo Concave Polygon
}
#endif