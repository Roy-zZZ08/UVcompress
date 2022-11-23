#ifndef VALUE_DEFINE_H
#define VALUE_DEFINE_H

#include <iostream>

#include "TexturedMesh.h"
#include "Block.h"

vector<Block*> blocks; //作为搜索起始点的区域信息 
int ncols, nrows; //图片大小

TexturedMesh mesh;
vector< AtlasChart > atlasCharts;
Image< int > nodeType;
Image< int > cellType;
Image< int > travelID;
Image< int > triangleID;
Image< Point3D< float > > patchImg;
Image< Point2D< double > > barycentricCoords;

Mat SobelEdge; 

#endif