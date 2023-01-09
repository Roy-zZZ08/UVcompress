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

const char* meshName = "tmp/outDelete5.obj";
const char* atlasName = "Resource/pack/texture3.jpg";


int main()
{
	// only support jpg
	/*mesh.read(meshName, atlasName);
	Initialize(mesh, mesh.texture.width(), mesh.texture.height(), nodeType, cellType, travelID, triangleID, barycentricCoords, patchImg, atlasCharts, oppositeHalfEdge);

	InitKlt();

	for (int i = 0; i < blocks.size(); i++) {
		FeatureRemesh(meshName, i);
		TexturedMesh newMesh;
		atlasCharts.clear();
		nodeType.clear();
		cellType.clear();
		travelID.clear();
		triangleID.clear();
		patchImg.clear();
		barycentricCoords.clear();
		oppositeHalfEdge.clear();
		newMesh.read(meshName, atlasName);
		Initialize(newMesh, mesh.texture.width(), newMesh.texture.height(), nodeType, cellType, travelID, triangleID, barycentricCoords, patchImg, atlasCharts, oppositeHalfEdge);
	}
	
	for (int i = 0; i < blocks.size(); i++) for (int j = 1; j < blocks[i]->finalMatchList.size(); j++) {
		AffineFeatureRemesh(i, j);
		TexturedMesh newMesh;
		atlasCharts.clear();
		nodeType.clear();
		cellType.clear();
		travelID.clear();
		triangleID.clear();
		patchImg.clear();
		barycentricCoords.clear();
		oppositeHalfEdge.clear();
		newMesh.read(meshName, atlasName);
		Initialize(newMesh, mesh.texture.width(), newMesh.texture.height(), nodeType, cellType, travelID, triangleID, barycentricCoords, patchImg, atlasCharts, oppositeHalfEdge);
	}*/
	
	
	CompressUV(meshName);

	return 0;
}
