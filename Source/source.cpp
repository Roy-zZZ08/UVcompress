#include <igl/cotmatrix.h>
#include <igl/readOBJ.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <opencv2/opencv.hpp>

#include "TexturedMesh.h"
#include "ChartConstruction.h"
#include "Atlas.h"
#include "init.h"

//-----klt---------
#include "klt.h"
#include "base.h"
#include "error.h"
#include "pnmio.h"

#include <iostream>
#include <thread>
#include <cmath>
#include <algorithm>
#include <Windows.h>

using namespace std;
using namespace cv;

TexturedMesh mesh;
vector< AtlasChart > atlasCharts;
Image< int > nodeType;
Image< int > cellType;
Image< int > travelID;
Image< int > triangleID;
Image< Point3D< float > > patchImg;
Image< Point2D< double > > barycentricCoords;



int main()
{
   // only support jpg
   //mesh.read("Resource/dogBone/model.obj", "Resource/dogBone/texture.jpg");

   //Initialize(mesh, mesh.texture.width(), mesh.texture.height(), nodeType, cellType, travelID, triangleID, barycentricCoords, patchImg, atlasCharts);

   InitKlt();

   //unsigned char* pixels = new unsigned char[_width * _height * 3];
   //for (int i = 0; i < _width; i++) for (int j = 0; j < _height; j++) for (int c = 0; c < 3; c++) 
   //    pixels[3 * (j * _width + i) + c] = (unsigned char)std::min< int >(255, std::max< int >(0, (int)((*this)(i, j)[c] * 255.f + 0.5f)));

   return 0;
}
