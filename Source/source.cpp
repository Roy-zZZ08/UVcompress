#include <igl/cotmatrix.h>
#include <igl/readOBJ.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <opencv2/opencv.hpp>

#include "TexturedMesh.h"
#include "ChartConstruction.h"
#include "Atlas.h"

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
Image< Point2D< double > > barycentricCoords;

int blocksize = 25;
int ncols, nrows;

uchar* patchInitImg(const string imgPath, int* ncols, int* nrows)
{
    uchar* ptr;
    Mat img = imread(imgPath, 1);
    if (img.empty()) {
        fprintf(stderr, "Can not load image %s\n", imgPath);
        return NULL;
    }

    *ncols = img.cols;
    *nrows = img.rows;
    ptr = (uchar*)malloc((*ncols) * (*nrows) * sizeof(char));
    if (ptr == NULL)
        KLTError("(imgRead) Memory not allocated");


}

int main()
{
   mesh.read("Resource/test2/test2.obj", "Resource/test2/test2.jpg");

   Initialize(mesh, mesh.texture.width(), mesh.texture.height(), nodeType, cellType, travelID, triangleID, barycentricCoords, atlasCharts);

   
   return 0;
}
