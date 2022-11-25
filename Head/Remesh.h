#ifndef REMESH_H
#define REMESH_H

#include "ValueDefine.h"

#include <cmath>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include "CDT\CDT.h"

int  boundPointDis = 10; // 筛选边界点间的距离

//Eigen::MatrixXd V, TC, N;
//Eigen::MatrixXi F, FTC, FN;

class myTmpPoint {
public:
	int x;
	int y;

	int id;

	myTmpPoint(int _x, int _y, int _id = -1) :x(_x), y(_y), id(_id) { }
	bool operator<(const myTmpPoint& p) const {
		if (this->x == p.x) return this->y < p.y;
		else return this->x < p.x;
	}

};

class InsertVert {
public:

	// geometry position
	double x;
	double y;
	double z;

	// uv position
	double u;
	double v;

	int triId;

	InsertVert() :x(0), y(0), z(0), u(0), v(0), triId(-1) {}

	void ComputeGeoPos() {
		// 对于新插入的边界点，根据重心坐标计算其几何上的位置
		Point3D< double > tPos[3];
		for (int i = 0; i < 3; i++) {
			tPos[i][0] = V(F(triId, i), 0);
			tPos[i][1] = V(F(triId, i), 1);
			tPos[i][2] = V(F(triId, i), 2);
		}
		Point2D< double > barycentricCoord = barycentricCoords(u * ncols, v * nrows);
		Point3D< double > newPos;

		newPos = tPos[0] + barycentricCoord[0] * (tPos[1] - tPos[0]) + barycentricCoord[1] * (tPos[2] - tPos[0]);
		x = newPos[0];
		y = newPos[1];
		z = newPos[2];

	}
};

void FeatureRemesh(const char* meshName)
{
	igl::readOBJ(meshName, V, TC, N, F, FTC, FN);

	// test
	Mat raw = imread(imgPath + imgName + ".jpg", 1);
	nrows = raw.rows;
	ncols = raw.cols;

	for (int blockIndex = 0; blockIndex < blocks.size(); blockIndex++) {
		set<myTmpPoint> bondaryPoints;
		unordered_set<int> triangleList;
		vector<InsertVert> insertVerts;
		vector<Point2i> newBoundPoints;
		set<myTmpPoint>  neighborTriVerts;

		Mat featureMask = blocks[blockIndex]->featureMaskOuter;

		int blockHeight = blocks[blockIndex]->getSizeHeight();
		int blockWidth = blocks[blockIndex]->getSizeWidth();

		// 找到外围Mask覆盖的一阶or二阶领域三角面UV点

		for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
			for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
				if (blocks[blockIndex]->featureMaskOuter.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
					int tmpCol = col + blockWidth / 2 + blocks[blockIndex]->getStartWidth();
					int tmpRow = row + blockHeight / 2 + blocks[blockIndex]->getStartHeight();
					if (triangleID(tmpCol, raw.rows - tmpRow - 1) != -1) {
						triangleList.insert(triangleID(tmpCol, raw.rows - tmpRow - 1));
					}
				}
			}
		}

		for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
			for (int k = 0; k < 3; k++) {
				double nowUV[2];
				nowUV[0] = TC(FTC(*iter, k), 0);
				nowUV[1] = TC(FTC(*iter, k), 1);

				double nowPos[2]; //col row
				nowPos[0] = nowUV[0] * raw.cols;
				nowPos[1] = raw.rows - 1 - nowUV[1] * raw.rows;

				neighborTriVerts.insert(myTmpPoint(nowPos[1] - blocks[blockIndex]->getStartHeight(), nowPos[0] - blocks[blockIndex]->getStartWidth(), *iter * 3 + k)); // row col
				raw.at<Vec3b>(Point2i((int)nowPos[0], (int)nowPos[1])) = Vec3b(255, 0, 0);
			}
		}

		for (int i = 0; i < blockHeight; i++) {
			int jleft = 0, jright = blockWidth - 1;

			while (jleft < blockWidth	&& featureMask.at<uchar>(i, jleft)	!= 255) jleft++;
			while (jright >= 0			&& featureMask.at<uchar>(i, jright) != 255) jright--;

			if (jleft	!= blockWidth)	bondaryPoints.insert(myTmpPoint(i, jleft));
			if (jright	!= -1)			bondaryPoints.insert(myTmpPoint(i, jright));
		}
		for (int j = 0; j < blockWidth; j++) {
			int iup = 0, idown = blockHeight - 1;

			while (iup < blockHeight	&& featureMask.at<uchar>(iup, j)	!= 255) iup++;
			while (idown >= 0			&& featureMask.at<uchar>(idown, j)	!= 255) idown--;

			if (iup != blockHeight)		bondaryPoints.insert(myTmpPoint(iup, j));
			if (idown != -1)			bondaryPoints.insert(myTmpPoint(idown, j));
		}

		// 筛选边界点

		for (auto it = bondaryPoints.begin(); it != bondaryPoints.end(); it++) {

			int texelU = blocks[blockIndex]->getStartWidth() + (*it).y;
			int texelV = nrows - 1 - (blocks[blockIndex]->getStartHeight() + (*it).x);

			if (triangleID(texelU, texelV) == -1) continue; // 无几何定义

			bool flag = FALSE;
			for (auto it2 = neighborTriVerts.begin(); it2 != neighborTriVerts.end(); it2++) {
				if (pow((*it).x - (*it2).x, 2) + pow((*it).y - (*it2).y, 2) < boundPointDis) {
					flag = TRUE;
					break;
				}
			}
			if (flag) continue;

			// 第一个点默认进行插入
			if (it == bondaryPoints.begin()) {

				InsertVert newInsertVert;
				newInsertVert.u = texelU * 1.0 / ncols;
				newInsertVert.v = texelV * 1.0 / nrows;
				newInsertVert.triId = triangleID(texelU, texelV);
				newInsertVert.ComputeGeoPos();
				insertVerts.push_back(newInsertVert);

				newBoundPoints.push_back(Point2i((*it).x, (*it).y));
			}
			else {
				//  对于其他的候选点，要求与先前插入点存在一定距离并且在几何上有定义
				bool flag = TRUE;
				for (int i = 0; i < newBoundPoints.size(); i++) {
					if (pow((*it).x - newBoundPoints[i].x, 2) + pow((*it).y - newBoundPoints[i].y, 2) < boundPointDis * boundPointDis) {
						flag = FALSE;
						break;
					}
				}
				if (flag) {

					InsertVert newInsertVert;
					newInsertVert.u = texelU * 1.0 / ncols;
					newInsertVert.v = texelV * 1.0 / nrows;
					newInsertVert.triId = triangleID(texelU, texelV);
					newInsertVert.ComputeGeoPos();
					insertVerts.push_back(newInsertVert);


					newBoundPoints.push_back(Point2i((*it).x, (*it).y));
				}
			}
		}

		for (int i = 0; i < newBoundPoints.size(); i++) {
			Point2i boundaryRed = Point2i(blocks[blockIndex]->getStartWidth() + newBoundPoints[i].y, blocks[blockIndex]->getStartHeight() + newBoundPoints[i].x);
			raw.at<Vec3b>(boundaryRed) = Vec3b(0, 0, 255);
		}

		// 对于新插入的点，与一邻域点重新三角剖分
		std::vector<CDT::V2d<double> > points;

		for (auto it = neighborTriVerts.begin(); it != neighborTriVerts.end(); it++) {
			points.push_back(CDT::V2d<double>::make((*it).x, (*it).y));
		}
		for (int i = 0; i < newBoundPoints.size(); i++) {
			points.push_back(CDT::V2d<double>::make(newBoundPoints[i].x, newBoundPoints[i].y));
		}

		//std::vector<CustomEdge> edges = /*...*/;
		CDT::Triangulation<double> cdt;
		cdt.insertVertices(points);
		cdt.eraseSuperTriangle();
 		int debug=0;

		debug += 1;

		TexturedMesh newMesh;
		for (auto it = neighborTriVerts.begin(); it != neighborTriVerts.end(); it++) {
			newMesh.vertices.push_back(Point3D<double>(V(F((*it).id / 3, (*it).id % 3), 0), V(F((*it).id / 3, (*it).id % 3), 1), V(F((*it).id / 3, (*it).id % 3), 2)));
			newMesh.textureCoordinates.push_back(Point2D<double>(TC(FTC((*it).id / 3, (*it).id % 3), 0), TC(FTC((*it).id / 3, (*it).id % 3), 1)));
		}
		for (int i = 0; i < insertVerts.size(); i++) {
			newMesh.vertices.push_back(Point3D<double>(insertVerts[i].x, insertVerts[i].y, insertVerts[i].z));
			newMesh.textureCoordinates.push_back(Point2D<double>(insertVerts[i].u, insertVerts[i].v));
		}
		for (int i = 0; i < cdt.triangles.size(); i++) {
			newMesh.triangles.push_back(TriangleIndex(cdt.triangles[i].vertices[0], cdt.triangles[i].vertices[1], cdt.triangles[i].vertices[2]));	
		}

		Eigen::MatrixXd newV, newTC, newN;
		Eigen::MatrixXi newF, newFTC, newFN;

		newV.resize(newMesh.vertices.size(), 3);
		for (int i = 0; i < newMesh.vertices.size(); i++) for (int k = 0; k < 3; k++) {
			newV(i, k) = newMesh.vertices[i][k];
		}
		newTC.resize(newMesh.textureCoordinates.size(), 2);
		for (int i = 0; i < newMesh.textureCoordinates.size(); i++) for (int k = 0; k < 2; k++) {
			newTC(i, k) = newMesh.textureCoordinates[i][k];
		}
		newN.resize(1, 3);
		newN(0, 0) = 0.5; newN(0, 1) = 0.5; newN(0, 2) = 0.5;
		newF.resize(newMesh.triangles.size(), 3);
		for (int i = 0; i < newMesh.triangles.size(); i++) for (int k = 0; k < 3; k++) {
			newF(i, k) = newMesh.triangles[i][k];
		}
		newFTC.resize(newMesh.triangles.size(), 3);
		for (int i = 0; i < newMesh.triangles.size(); i++) for (int k = 0; k < 3; k++) {
			newFTC(i, k) = newMesh.triangles[i][k];
		}
		newFN.resize(newMesh.triangles.size(), 3);
		for (int i = 0; i < newMesh.triangles.size(); i++) for (int k = 0; k < 3; k++) {
			newFN(i, k) = 0;
		}
		igl::writeOBJ("tmp/outTestRemesh.obj", newV, newF, newN, newFN, newTC, newFTC);
	}

	imshow("raw", raw);
	imwrite(imgPath + imgName + "_remesh.png", raw);
	waitKey();

	/*int i = 0;
	double nowPos[3][3];
	double nowUV [3][2];

	for (int k = 0; k < 3; k++) {

		 geometry pos
		nowPos[k][0] = V(F(i, k), 0);
		nowPos[k][1] = V(F(i, k), 1);
		nowPos[k][2] = V(F(i, k), 2);

		 UV pos
		nowUV[k][0] = TC(FTC(i, k), 0);
		nowUV[k][1] = TC(FTC(i, k), 1);
	}

	 subdivision grid

	double newPos[3][3];
	double newUV[3][2];

	for (int k = 0; k < 3; k++) {

		 geometry pos
		newPos[k][0] = (nowPos[k][0] + nowPos[(k + 1) % 3][0]) / 2.0;
		newPos[k][1] = (nowPos[k][1] + nowPos[(k + 1) % 3][1]) / 2.0;
		newPos[k][2] = (nowPos[k][2] + nowPos[(k + 1) % 3][2]) / 2.0;

		 UV pos
		newUV[k][0] = (nowUV[k][0] + nowUV[(k + 1) % 3][0]) / 2.0;
		newUV[k][1] = (nowUV[k][1] + nowUV[(k + 1) % 3][1]) / 2.0;
	}

	int debug = 0;

	int nowRowV = V.rows();
	int nowRowTC = TC.rows();
	int nowRowF = F.rows();

	V.conservativeResize(nowRowV + 3, 3);
	TC.conservativeResize(nowRowTC + 3, 2);

	for (int k = 0; k < 3; k++) {
		V(nowRowV + k, 0) = newPos[k][0];
		V(nowRowV + k, 1) = newPos[k][1];
		V(nowRowV + k, 2) = newPos[k][2];

		TC(nowRowTC + k, 0) = newUV[k][0];
		TC(nowRowTC + k, 1) = newUV[k][1];
	}
	
	int vIndex0 = F(i, 0);
	int vIndex1 = F(i, 1);
	int vIndex2 = F(i, 2);

	int tcIndex0 = FTC(i, 0);
	int tcIndex1 = FTC(i, 1);
	int tcIndex2 = FTC(i, 2);

	F.conservativeResize(nowRowF + 3, 3);
	FN.conservativeResize(nowRowF + 3, 3);
	FTC.conservativeResize(nowRowF + 3, 3);

	F(i, 0) = vIndex0;
	F(i, 1) = nowRowV;
	F(i, 2) = nowRowV + 2;

	F(nowRowF, 0) = nowRowV;
	F(nowRowF, 1) = vIndex1;
	F(nowRowF, 2) = nowRowV + 1;

	F(nowRowF + 1, 0) = nowRowV + 2;
	F(nowRowF + 1, 1) = nowRowV + 1;
	F(nowRowF + 1, 2) = vIndex2;

	F(nowRowF + 2, 0) = nowRowV;
	F(nowRowF + 2, 1) = nowRowV + 1;
	F(nowRowF + 2, 2) = nowRowV + 2;

	FTC(i, 0) = tcIndex0;
	FTC(i, 1) = nowRowTC;
	FTC(i, 2) = nowRowTC + 2;

	FTC(nowRowF, 0) = nowRowTC;
	FTC(nowRowF, 1) = tcIndex1;
	FTC(nowRowF, 2) = nowRowTC + 1;

	FTC(nowRowF + 1, 0) = nowRowTC + 2;
	FTC(nowRowF + 1, 1) = nowRowTC + 1;
	FTC(nowRowF + 1, 2) = tcIndex2;

	FTC(nowRowF + 2, 0) = nowRowTC;
	FTC(nowRowF + 2, 1) = nowRowTC + 1;
	FTC(nowRowF + 2, 2) = nowRowTC + 2;

	FN(nowRowF, 0) = FN(nowRowF + 1, 0) = FN(nowRowF + 2, 0) = FN(i, 0);
	FN(nowRowF, 1) = FN(nowRowF + 1, 1) = FN(nowRowF + 2, 1) = FN(i, 1);
	FN(nowRowF, 2) = FN(nowRowF + 1, 2) = FN(nowRowF + 2, 2) = FN(i, 2);*/





	igl::writeOBJ("tmp/outTest0.obj", V, F, N, FN, TC, FTC);

	//					nowUV[0] = TC(FTC(*iter, k), 0);
	//					nowUV[1] = TC(FTC(*iter, k), 1);

	//int blockIndex = 0;
	//unordered_set<int>triangleList;

	//if (blocks[blockIndex]->finalMatchList.size() < 2) return;

	//for (int i = 0; i < blocks[blockIndex]->finalMatchList.size(); i++) {
	//	printf("\rCompress UV Region[%.2f%%]   ", i * 100.0 / blocks[blockIndex]->finalMatchList.size());

	//	triangleList.clear();
	//	Mat M = blocks[blockIndex]->finalMatchList[i].getMatrix();
	//	double* m = M.ptr<double>();

	//	int blockHeight = blocks[blockIndex]->getSizeHeight();
	//	int blockWidth = blocks[blockIndex]->getSizeWidth();

	//	// 找到经过仿射变换后的匹配特征所包含的三角形

	//	for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
	//		for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
	//			if (blocks[blockIndex]->featureMaskInner.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
	//				int tmpCol = (int)(m[0] * col + m[1] * row + m[2]);
	//				int tmpRow = (int)(m[3] * col + m[4] * row + m[5]);
	//				if (tmpCol < ncols && tmpCol >= 0 && tmpRow < nrows && tmpRow >= 0) {
	//					if (triangleID(tmpCol, nrows - tmpRow - 1) != -1) {
	//						triangleList.insert(triangleID(tmpCol, nrows - tmpRow - 1));
	//					}
	//				}
	//			}
	//		}
	//	}
	//	if (i == 0) {
	//		for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
	//			if (find(triangleVisit.begin(), triangleVisit.end(), *iter) == triangleVisit.end()) {
	//				triangleVisit.push_back(*iter);
	//			}
	//		}
	//	}
	//	else {
	//		double denominator = m[1] * m[3] - m[0] * m[4];
	//		for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
	//			if (find(triangleVisit.begin(), triangleVisit.end(), *iter) == triangleVisit.end()) {
	//				triangleVisit.push_back(*iter);

	//				// 计算逆向仿射变换修改每一个三角形的uv坐标
	//				for (int k = 0; k < 3; k++) {
	//					//std::cout << "TC:    " << std::endl << TC << std::endl;
	//					int nowRow = TC.rows();

	//					int test = FTC.rows();

	//					double nowUV[2];
	//					nowUV[0] = TC(FTC(*iter, k), 0);
	//					nowUV[1] = TC(FTC(*iter, k), 1);

	//					double nowPos[2]; //col row
	//					nowPos[0] = nowUV[0] * ncols;
	//					nowPos[1] = nrows - 1 - nowUV[1] * nrows;

	//					TC.conservativeResize(nowRow + 1, 2);

	//					double inverPos[2];
	//					inverPos[0] = (m[1] * (nowPos[1] - m[5]) - m[4] * (nowPos[0] - m[2])) / denominator + blockWidth / 2 + blocks[blockIndex]->getStartWidth(); //col
	//					inverPos[1] = (m[3] * (nowPos[0] - m[2]) - m[0] * (nowPos[1] - m[5])) / denominator + blockHeight / 2 + blocks[blockIndex]->getStartHeight(); //row

	//					double inverUV[2];
	//					inverUV[0] = inverPos[0] / ncols;
	//					inverUV[1] = (nrows - 1 - inverPos[1]) / nrows;

	//					TC(nowRow, 0) = inverUV[0];
	//					TC(nowRow, 1) = inverUV[1];

	//					FTC(*iter, k) = nowRow;
	//				}
	//			}
	//		}
	//	}

	//}
}

#endif