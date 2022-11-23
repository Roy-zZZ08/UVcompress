#ifndef VALUE_DEFINE_H
#define VALUE_DEFINE_H

#include <iostream>

#include "TexturedMesh.h"
#include "Block.h"

vector<Block*> blocks; //��Ϊ������ʼ���������Ϣ 
int ncols, nrows; //ͼƬ��С

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