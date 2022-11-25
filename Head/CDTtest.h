#pragma once

#include "CDT\CDT.h"

struct CustomPoint2D
{
	double data[2];

    CustomPoint2D(double x, double y) { data[0] = x, data[1] = y; }
};

struct CustomEdge
{
	std::pair<std::size_t, std::size_t> vertices;
};

void Test()
{
    // containers other than std::vector will work too

    std::vector<CDT::V2d<double> > points;
    //std::vector<CustomPoint2D> points;
    points.push_back(CDT::V2d<double>::make(0, 2));
    points.push_back(CDT::V2d<double>::make(-1, 1));
    points.push_back(CDT::V2d<double>::make(-1, -1));
    points.push_back(CDT::V2d<double>::make(1, -1));
    points.push_back(CDT::V2d<double>::make(1, 1));

    //std::vector<CustomEdge> edges = /*...*/;
    CDT::Triangulation<double> cdt;
    cdt.insertVertices(points);
    cdt.eraseSuperTriangle();
    int debug;
}
