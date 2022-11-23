#ifndef REMESH_H
#define REMESH_H

#include "ValueDefine.h"

#include <cmath>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

int  boundPointDis = 10; // 筛选边界点间的距离

//Eigen::MatrixXd V, TC, N;
//Eigen::MatrixXi F, FTC, FN;

class myTmpPoint {
public:
	int x;
	int y;

	myTmpPoint(int _x, int _y) :x(_x), y(_y) { }
	bool operator<(const myTmpPoint& p) const {
		if (this->x == p.x) return this->y < p.y;
		else return this->x < p.x;
	}

};

void FeatureRemesh(const char* meshName)
{
	igl::readOBJ(meshName, V, TC, N, F, FTC, FN);

	// test
	Mat raw = imread(imgPath + imgName + ".jpg", 1);

	for (int blockIndex = 0; blockIndex < blocks.size(); blockIndex++) {
		set<myTmpPoint> bondaryPoints;
		unordered_set<int>triangleList;

		Mat featureMask = blocks[blockIndex]->featureMaskOuter;

		int blockHeight = blocks[blockIndex]->getSizeHeight();
		int blockWidth = blocks[blockIndex]->getSizeWidth();

		// 找到外围Mask覆盖的一阶or二阶领域三角面UV点

		for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
			for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
				if (blocks[blockIndex]->featureMaskOuter.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
					if (triangleID(col, nrows - row - 1) != -1) {
						triangleList.insert(triangleID(col, nrows - row - 1));
					}
				}
			}
		}

		for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {

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
		vector<Point2i> newBoundPoints;
		for (auto it = bondaryPoints.begin(); it != bondaryPoints.end(); it++) {
			if (it == bondaryPoints.begin()) {
				newBoundPoints.push_back(Point2i((*it).x, (*it).y));
			}
			else {
				bool flag = TRUE;
				for (int i = 0; i < newBoundPoints.size(); i++) {
					if (pow((*it).x - newBoundPoints[i].x, 2) + pow((*it).y - newBoundPoints[i].y, 2) < boundPointDis * boundPointDis) {
						flag = FALSE;
						break;
					}
				}
				if (flag) newBoundPoints.push_back(Point2i((*it).x, (*it).y));
			}
		}

		for (int i = 0; i < newBoundPoints.size(); i++) {
			Point2i boundaryRed = Point2i(blocks[blockIndex]->getStartWidth() + newBoundPoints[i].y, blocks[blockIndex]->getStartHeight() + newBoundPoints[i].x);
			raw.at<Vec3b>(boundaryRed) = Vec3b(0, 0, 255);
		}
	}

	imshow("raw", raw);
	waitKey();

	int i = 0;
	double nowPos[3][3];
	double nowUV [3][2];

	for (int k = 0; k < 3; k++) {

		// geometry pos
		nowPos[k][0] = V(F(i, k), 0);
		nowPos[k][1] = V(F(i, k), 1);
		nowPos[k][2] = V(F(i, k), 2);

		// UV pos
		nowUV[k][0] = TC(FTC(i, k), 0);
		nowUV[k][1] = TC(FTC(i, k), 1);
	}

	// subdivision grid

	double newPos[3][3];
	double newUV[3][2];

	for (int k = 0; k < 3; k++) {

		// geometry pos
		newPos[k][0] = (nowPos[k][0] + nowPos[(k + 1) % 3][0]) / 2.0;
		newPos[k][1] = (nowPos[k][1] + nowPos[(k + 1) % 3][1]) / 2.0;
		newPos[k][2] = (nowPos[k][2] + nowPos[(k + 1) % 3][2]) / 2.0;

		// UV pos
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
	FN(nowRowF, 2) = FN(nowRowF + 1, 2) = FN(nowRowF + 2, 2) = FN(i, 2);





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