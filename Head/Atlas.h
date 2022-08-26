#ifndef ATLAS_H
#define ATLAS_H

#include <Misha/Geometry.h>

class OppositeCoord
{
public:
	Point2D< double > center1, center2;
	Point2D< double > xAxis1, yAxis1, xAxis2, yAxis2;
};

class TextureNodeInfo
{
public:
	TextureNodeInfo(void) : tID(-1), ci(-1), cj(-1), chartID(-1), isInterior(false) {}
	TextureNodeInfo(int _tID, Point2D< double > _barycentricCoords, int _ci, int _cj, int _chartID, bool _isInterior) : tID(_tID), barycentricCoords(_barycentricCoords), ci(_ci), cj(_cj), chartID(_chartID), isInterior(_isInterior) {}

	int tID;
	Point2D< double > barycentricCoords;
	int ci, cj;
	int chartID;
	bool isInterior;
};

class BilinearElementIndex
{
protected:
	unsigned int v[4];
public:
	BilinearElementIndex(void) { v[0] = v[1] = v[2] = v[3] = 0; }
	BilinearElementIndex(unsigned int v0, unsigned int v1, unsigned int v2, unsigned int v3) { v[0] = v0, v[1] = v1, v[2] = v2, v[3] = v3; }
	unsigned int& operator[](unsigned int idx) { return v[idx]; }
	unsigned int  operator[](unsigned int idx) const { return v[idx]; }
};

class AtlasMesh
{
public:
	std::vector< Point2D< double > > vertices;
	std::vector< TriangleIndex > triangles;
	std::vector< int > triangleIndexInChart;
	std::vector< int > triangleChartIndex;
	std::vector< int > halfEdgeToEdgeIndex;
	std::vector< int > vertexMap;
	int numCharts;
};

class AtlasChart
{
public:
	Point2D< double > minCorner;
	Point2D< double > maxCorner;
	Point2D< double > gridOrigin;
	int originCoords[2];

	std::vector< TriangleIndex > triangles;
	std::vector< Point2D< double > > vertices;
	std::vector< int > boundaryHalfEdges;
	std::vector< int > atlasEdgeIndices;

	std::vector< int > meshVertexIndices;
	std::vector< int > meshTriangleIndices;
};

class GridChart
{
public:
	Point2D< double > corner;
	double cellSizeW;
	double cellSizeH;
	int width;
	int height;

	Image< int > nodeType;
	Image< int > cellType;

	Image< int > triangleID;
	Image< Point2D< double > > barycentricCoords;
};

class IndexedVector2D
{
public:
	IndexedVector2D(Point2D< double > p_p, int p_index, int p_vertex)
	{
		p = p_p;
		index = p_index;
		vertex = p_vertex;
	}
	Point2D< double > p;
	int index;
	int vertex;
};

// set order
class IndexedVector2DComparison
{
public:
	bool operator()(const IndexedVector2D& p1, const IndexedVector2D& p2) const
	{
		for (int i = 0; i < 2; i++)
		{
			if		(p1.p[i] < p2.p[i]) return true;
			else if (p2.p[i] < p1.p[i]) return false;
			else
			{
				if (p1.vertex < p2.vertex) return true;
				else if (p2.vertex < p1.vertex) return false;
			}
		}
		return false;
	}
};

#endif