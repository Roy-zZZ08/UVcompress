#include <igl/cotmatrix.h>
#include <igl/readOBJ.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <opencv2/opencv.hpp>

#include "ValueDefine.h"

#include "ChartConstruction.h"
#include "CompressUV.h"
#include "Atlas.h"
#include "init.h"
#include "Remesh.h"

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

const char* meshName = "Resource/pack/model.obj";
const char* atlasName = "Resource/pack/texture.jpg";

int main()
{
	// only support jpg
	mesh.read(meshName, atlasName);

	Initialize(mesh, mesh.texture.width(), mesh.texture.height(), nodeType, cellType, travelID, triangleID, barycentricCoords, patchImg, atlasCharts);

	InitKlt();

	FeatureRemesh(meshName);
   
	CompressUV(meshName);

	return 0;
}
