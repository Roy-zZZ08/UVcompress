#ifndef REMESH_H
#define REMESH_H

#include "ValueDefine.h"

#include <cmath>
#include <algorithm>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include "CDT\CDT.h"

#include "RemeshMesh.h"

int  boundPointDis = 10; // 筛选边界点间的距离

const char* tmpMeshName = "tmp/model.obj";
const char* tmpAtlasName = "Resource/pack/texture.jpg";


class myTmpPoint {
public:
	int x;
	int y;

	int id;
	int CDTid;

	myTmpPoint(int _x, int _y, int _id = -1) :x(_x), y(_y), id(_id) { CDTid = -1; }
	bool operator<(const myTmpPoint& p) const {
		if (this->x == p.x) return this->y < p.y;
		else return this->x < p.x;
	}
	bool operator==(const myTmpPoint& p) const {
		return this->x == p.x && this->y == p.y;
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

struct CustomEdge
{
	CustomEdge(std::size_t a, std::size_t b) { vertices = make_pair(a, b); }
	std::pair<std::size_t, std::size_t> vertices;
};


void FeatureRemesh(const char* meshName, int blockIndex)
{
	RemeshMesh rawMesh;
	rawMesh.read(tmpMeshName);

	igl::readOBJ(tmpMeshName, V, TC, N, F, FTC, FN);

	Mat raw = imread(imgPath + imgName + ".jpg", 1);
	nrows = raw.rows;
	ncols = raw.cols;

	unordered_set<int> triangleList; // 包含外围特征的三角形

	set<myTmpPoint> bondaryPoints;
	vector<myTmpPoint> neighborTriVerts;

	vector<InsertVert> insertVerts;
	vector<Point2i> newBoundPoints;

	vector<CustomEdge> edges;
	vector<int> edgeToCDTidx(3 * F.size());

	Mat featureMask = blocks[blockIndex]->featureMaskOuter;

	int blockHeight = blocks[blockIndex]->getSizeHeight();
	int blockWidth = blocks[blockIndex]->getSizeWidth();

	// 找到外围Mask覆盖的一阶or二阶领域三角面UV点

	for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
		for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
			if (featureMask.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
				int tmpCol = col + blockWidth / 2 + blocks[blockIndex]->getStartWidth();
				int tmpRow = row + blockHeight / 2 + blocks[blockIndex]->getStartHeight();
				if (triangleID(tmpCol, raw.rows - tmpRow - 1) != -1) {
					triangleList.insert(triangleID(tmpCol, raw.rows - tmpRow - 1));
				}
			}
		}
	}

	int CDTcount = 0;
	for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
		vector<myTmpPoint> tmpVerts;
		for (int k = 0; k < 3; k++) {
			double nowUV[2];
			nowUV[0] = TC(FTC(*iter, k), 0);
			nowUV[1] = TC(FTC(*iter, k), 1);

			double nowPos[2]; //col row
			nowPos[0] = nowUV[0] * raw.cols;
			nowPos[1] = raw.rows - 1 - nowUV[1] * raw.rows;

			tmpVerts.push_back(myTmpPoint(nowPos[1] - blocks[blockIndex]->getStartHeight(), nowPos[0] - blocks[blockIndex]->getStartWidth(), *iter * 3 + k)); // row col
		}

		// 插入后当前三角形在CDT中的index为 nowSize\nowSize+1\nowSize+2
		// 判断该三角形的三条边是否是mask边界
		for (int k = 0; k < 3; k++) {
			int nowEdge = *iter * 3 + k;
			int oppositeTri = oppositeHalfEdge[nowEdge] / 3;
			if (triangleList.find(oppositeTri) == triangleList.end()) {
				// 找到边界的话，就将组成边界的点加入
				int edgeIndex1 = -1, edgeIndex2 = -1;
				auto it = find(neighborTriVerts.begin(), neighborTriVerts.end(), tmpVerts[k]);
				if (it == neighborTriVerts.end()) {
					tmpVerts[k].CDTid = CDTcount++;
					neighborTriVerts.push_back(tmpVerts[k]);
					edgeIndex1 = tmpVerts[k].CDTid;
				}
				else {
					edgeIndex1 = it->CDTid;
				}
				it = find(neighborTriVerts.begin(), neighborTriVerts.end(), tmpVerts[(k + 1) % 3]);
				if (it == neighborTriVerts.end()) {
					tmpVerts[(k + 1) % 3].CDTid = CDTcount++;
					neighborTriVerts.push_back(tmpVerts[(k + 1) % 3]);
					edgeIndex2 = tmpVerts[(k + 1) % 3].CDTid;
				}
				else {
					edgeIndex2 = it->CDTid;
				}
				edges.push_back(CustomEdge(edgeIndex1, edgeIndex2));
			}
		}

		// 需要在原始网格中删除这部分三角面
		rawMesh.deleteTri[*iter] = 1;
		rawMesh.deleteNum++;

	}
	//rawMesh.save("tmp/deleteMesh.obj");

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

	blocks[blockIndex]->newBoundPoints = newBoundPoints;

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
	cdt.insertEdges(
		edges.begin(),
		edges.end(),
		[](const CustomEdge& e) { return e.vertices.first; },
		[](const CustomEdge& e) { return e.vertices.second; }
	);

	cdt.eraseOuterTriangles();

	int rawMeshVnum = rawMesh.vertices.size();
	int rawMeshTCnum = rawMesh.textureCoordinates.size();

	for (int i = 0; i < insertVerts.size(); i++) {
		rawMesh.vertices.push_back(Point3D<double>(insertVerts[i].x, insertVerts[i].y, insertVerts[i].z));
		rawMesh.textureCoordinates.push_back(Point2D<double>(insertVerts[i].u, insertVerts[i].v));
	}
	for (int i = 0; i < cdt.triangles.size(); i++) {
		int newFIdx[3];
		int newFTCIdx[3];
		for (int k = 0; k < 3; k++) {
			// belong to neighborTriVerts
			if (cdt.triangles[i].vertices[k] < neighborTriVerts.size()) {
				int rawId = neighborTriVerts[cdt.triangles[i].vertices[k]].id;
				newFIdx[k] = F(rawId / 3, rawId % 3);
				newFTCIdx[k] = FTC(rawId / 3, rawId % 3);
			}
			else { // belong to new insert verts
				newFIdx[k] = cdt.triangles[i].vertices[k] + rawMeshVnum - neighborTriVerts.size();
				newFTCIdx[k] = cdt.triangles[i].vertices[k] + rawMeshTCnum - neighborTriVerts.size();
			}
		}
		rawMesh.triangles.push_back(TriangleIndex(newFIdx[0], newFIdx[1], newFIdx[2]));
		rawMesh.coordinateIndexs.push_back(TriangleIndex(newFTCIdx[0], newFTCIdx[1], newFTCIdx[2]));
	}

	rawMesh.updateNormals();
	rawMesh.save(tmpMeshName);


	//imshow("raw", raw);
	//imwrite(imgPath + imgName + "_remesh.png", raw);
	//waitKey();
}

void AffineFeatureRemesh(int blockIndex, int matchIndex)
{
	RemeshMesh rawMesh;
	rawMesh.read(tmpMeshName);
	igl::readOBJ(tmpMeshName, V, TC, N, F, FTC, FN);

	Mat raw = imread(imgPath + imgName + ".jpg", 1);
	nrows = raw.rows;
	ncols = raw.cols;

	// do affine map feature remesh

	unordered_set<int> triangleList;
	set<myTmpPoint> bondaryPoints;
	vector<myTmpPoint> neighborTriVerts;
	vector<InsertVert> insertVerts;
	vector<Point2i> newBoundPoints;
	vector<CustomEdge> edges;
	vector<int> edgeToCDTidx(3 * F.size());

	Mat featureMask = blocks[blockIndex]->featureMaskOuter;

	Mat M = blocks[blockIndex]->finalMatchList[matchIndex].getMatrix();
	double* m = M.ptr<double>();

	int blockHeight = blocks[blockIndex]->getSizeHeight();
	int blockWidth = blocks[blockIndex]->getSizeWidth();

	// 找到经过仿射变换后的匹配特征所包含的三角形

	for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
		for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
			if (featureMask.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
				int tmpCol = (int)(m[0] * col + m[1] * row + m[2]);
				int tmpRow = (int)(m[3] * col + m[4] * row + m[5]);
				if (tmpCol < ncols && tmpCol >= 0 && tmpRow < nrows && tmpRow >= 0) {
					if (triangleID(tmpCol, nrows - tmpRow - 1) != -1) {
						triangleList.insert(triangleID(tmpCol, nrows - tmpRow - 1));
					}
				}
			}
		}
	}

	int CDTcount = 0;
	Point2i initCenter(blockHeight / 2, blockWidth / 2);
	Point2i affineCenter(m[5], m[2]); // row col

	for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
		vector<myTmpPoint> tmpVerts;
		for (int k = 0; k < 3; k++) {
			double nowUV[2];
			nowUV[0] = TC(FTC(*iter, k), 0);
			nowUV[1] = TC(FTC(*iter, k), 1);

			double nowPos[2]; //col row
			nowPos[0] = nowUV[0] * raw.cols;
			nowPos[1] = raw.rows - 1 - nowUV[1] * raw.rows;

			// 以仿射变换后的坐标中心为参照点（区别于 init feature remesh）
			tmpVerts.push_back(myTmpPoint(nowPos[1] - affineCenter.x, nowPos[0] - affineCenter.y, *iter * 3 + k)); // row col
		}

		for (int k = 0; k < 3; k++) {
			int nowEdge = *iter * 3 + k;
			int oppositeTri = oppositeHalfEdge[nowEdge] / 3;
			if (triangleList.find(oppositeTri) == triangleList.end()) {

				int edgeIndex1 = -1, edgeIndex2 = -1;
				auto it = find(neighborTriVerts.begin(), neighborTriVerts.end(), tmpVerts[k]);
				if (it == neighborTriVerts.end()) {
					tmpVerts[k].CDTid = CDTcount++;
					neighborTriVerts.push_back(tmpVerts[k]);
					edgeIndex1 = tmpVerts[k].CDTid;
				}
				else {
					edgeIndex1 = it->CDTid;
				}
				it = find(neighborTriVerts.begin(), neighborTriVerts.end(), tmpVerts[(k + 1) % 3]);
				if (it == neighborTriVerts.end()) {
					tmpVerts[(k + 1) % 3].CDTid = CDTcount++;
					neighborTriVerts.push_back(tmpVerts[(k + 1) % 3]);
					edgeIndex2 = tmpVerts[(k + 1) % 3].CDTid;
				}
				else {
					edgeIndex2 = it->CDTid;
				}
				edges.push_back(CustomEdge(edgeIndex1, edgeIndex2));
			}
		}

		rawMesh.deleteTri[*iter] = 1;
		rawMesh.deleteNum++;
	}

	for (int i = 0; i < blocks[blockIndex]->newBoundPoints.size(); i++) {

		// 边界点经过仿射变换后相对于仿射变换窗口中心的坐标
		Point2i refTmpPoint(blocks[blockIndex]->newBoundPoints[i].x - blockHeight / 2, blocks[blockIndex]->newBoundPoints[i].y - blockWidth / 2);
		
		Point2i refBoundPoint;
		refBoundPoint.x = (int)(m[3] * refTmpPoint.y + m[4] * refTmpPoint.x);
		refBoundPoint.y = (int)(m[0] * refTmpPoint.y + m[1] * refTmpPoint.x);
		
		newBoundPoints.push_back(refBoundPoint); // row col

		int texelU = affineCenter.y + refBoundPoint.y;
		int texelV = nrows - 1 - (affineCenter.x + refBoundPoint.x);
		InsertVert newInsertVert;
		newInsertVert.u = texelU * 1.0 / ncols;
		newInsertVert.v = texelV * 1.0 / nrows;
		newInsertVert.triId = triangleID(texelU, texelV);
		if (newInsertVert.triId != -1) {
			newInsertVert.ComputeGeoPos();
			insertVerts.push_back(newInsertVert);
		}
	}

	std::vector<CDT::V2d<double> > points;

	for (auto it = neighborTriVerts.begin(); it != neighborTriVerts.end(); it++) {
		points.push_back(CDT::V2d<double>::make((*it).x, (*it).y));
	}
	for (int i = 0; i < newBoundPoints.size(); i++) {
		points.push_back(CDT::V2d<double>::make(newBoundPoints[i].x, newBoundPoints[i].y));
	}

	CDT::Triangulation<double> cdt;
	cdt.insertVertices(points);
	cdt.insertEdges(
		edges.begin(),
		edges.end(),
		[](const CustomEdge& e) { return e.vertices.first; },
		[](const CustomEdge& e) { return e.vertices.second; }
	);

	cdt.eraseOuterTriangles();

	int rawMeshVnum = rawMesh.vertices.size();
	int rawMeshTCnum = rawMesh.textureCoordinates.size();

	for (int i = 0; i < insertVerts.size(); i++) {
		rawMesh.vertices.push_back(Point3D<double>(insertVerts[i].x, insertVerts[i].y, insertVerts[i].z));
		rawMesh.textureCoordinates.push_back(Point2D<double>(insertVerts[i].u, insertVerts[i].v));
	}
	for (int i = 0; i < cdt.triangles.size(); i++) {
		int newFIdx[3];
		int newFTCIdx[3];
		for (int k = 0; k < 3; k++) {
			// belong to neighborTriVerts
			if (cdt.triangles[i].vertices[k] < neighborTriVerts.size()) {
				int rawId = neighborTriVerts[cdt.triangles[i].vertices[k]].id;
				newFIdx[k] = F(rawId / 3, rawId % 3);
				newFTCIdx[k] = FTC(rawId / 3, rawId % 3);
			}
			else { // belong to new insert verts
				newFIdx[k] = cdt.triangles[i].vertices[k] + rawMeshVnum - neighborTriVerts.size();
				newFTCIdx[k] = cdt.triangles[i].vertices[k] + rawMeshTCnum - neighborTriVerts.size();
			}
		}
		rawMesh.triangles.push_back(TriangleIndex(newFIdx[0], newFIdx[1], newFIdx[2]));
		rawMesh.coordinateIndexs.push_back(TriangleIndex(newFTCIdx[0], newFTCIdx[1], newFTCIdx[2]));
	}

	rawMesh.updateNormals();
	rawMesh.save(tmpMeshName);

}

#endif