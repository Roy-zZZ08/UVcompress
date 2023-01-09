#ifndef COMPRESS_UV_H
#define COMPRESS_UV_H

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include "init.h"
#include "ValueDefine.h"
#include "RemeshMesh.h"

#include <unordered_set>
#include <set>
#include <algorithm>

//int blockIndex = 0; // 当前压缩的blockIndex 测试用
vector<int> triangleVisit; // record compressed triangle

unordered_set<int> deleteTriIndex;

Eigen::MatrixXd V, TC, N;
Eigen::MatrixXi F, FTC, FN;

// for test
void DeleteFeature(RemeshMesh& rawMesh, int blockIndex) {

	if (blocks[blockIndex]->finalMatchList.size() < 2) return;

	for (int i = 1; i < blocks[blockIndex]->finalMatchList.size(); i++) {

		Mat M = blocks[blockIndex]->finalMatchList[i].getMatrix();
		double* m = M.ptr<double>();

		int blockHeight = blocks[blockIndex]->getSizeHeight();
		int blockWidth = blocks[blockIndex]->getSizeWidth();

		// 找到经过仿射变换后的匹配特征所包含的三角形

		for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
			for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
				if (blocks[blockIndex]->featureMaskInner.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
					int tmpCol = (int)(m[0] * col + m[1] * row + m[2]);
					int tmpRow = (int)(m[3] * col + m[4] * row + m[5]);
					if (tmpCol < ncols && tmpCol >= 0 && tmpRow < nrows && tmpRow >= 0) {
						if (triangleID(tmpCol, nrows - tmpRow - 1) != -1) {
							deleteTriIndex.insert(triangleID(tmpCol, nrows - tmpRow - 1));
						}
					}
				}
			}
		}

	}
}

void CompressFeature(int blockIndex) 
{
	unordered_set<int>triangleList;

	if (blocks[blockIndex]->finalMatchList.size() < 2) return;

	for (int i = 0; i < blocks[blockIndex]->finalMatchList.size(); i++) {
		printf("\rCompress UV Region[%.2f%%]   ", i * 100.0 / blocks[blockIndex]->finalMatchList.size());

		triangleList.clear();
		Mat M = blocks[blockIndex]->finalMatchList[i].getMatrix();
		double* m = M.ptr<double>();

		int blockHeight = blocks[blockIndex]->getSizeHeight();
		int blockWidth = blocks[blockIndex]->getSizeWidth();

		// 找到经过仿射变换后的匹配特征所包含的三角形

		for (int row = -blockHeight / 2; row < blockHeight / 2; row++) {
			for (int col = -blockWidth / 2; col < blockWidth / 2; col++) {
				if (blocks[blockIndex]->featureMaskInner.at<uchar>(row + blockHeight / 2, col + blockWidth / 2) == 255) {
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
		if (i == 0) {
			for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
				if (find(triangleVisit.begin(), triangleVisit.end(), *iter) == triangleVisit.end()) {
					triangleVisit.push_back(*iter);
				}
			}
		}
		else {
			double denominator = m[1] * m[3] - m[0] * m[4];
			for (auto iter = triangleList.begin(); iter != triangleList.end(); iter++) {
				if (find(triangleVisit.begin(), triangleVisit.end(), *iter) == triangleVisit.end()) {
					triangleVisit.push_back(*iter);

					// 计算逆向仿射变换修改每一个三角形的uv坐标
					for (int k = 0; k < 3; k++) {
						//std::cout << "TC:    " << std::endl << TC << std::endl;
						int nowRow = TC.rows();

						int test = FTC.rows();

						double nowUV[2];
						nowUV[0] = TC(FTC(*iter, k), 0);
						nowUV[1] = TC(FTC(*iter, k), 1);

						double nowPos[2]; //col row
						nowPos[0] = nowUV[0] * ncols;
						nowPos[1] = nrows - 1 - nowUV[1] * nrows;

						TC.conservativeResize(nowRow + 1, 2);

						double inverPos[2];
						inverPos[0] = (m[1] * (nowPos[1] - m[5]) - m[4] * (nowPos[0] - m[2])) / denominator + blockWidth / 2 + blocks[blockIndex]->getStartWidth(); //col
						inverPos[1] = (m[3] * (nowPos[0] - m[2]) - m[0] * (nowPos[1] - m[5])) / denominator + blockHeight / 2 + blocks[blockIndex]->getStartHeight(); //row

						double inverUV[2];
						inverUV[0] = inverPos[0] / ncols;
						inverUV[1] = (nrows - 1 - inverPos[1]) / nrows;

						TC(nowRow, 0) = inverUV[0];
						TC(nowRow, 1) = inverUV[1];

						FTC(*iter, k) = nowRow;
					}
				}
			}
		}
			
	}
}

void DeletePure(RemeshMesh& rawMesh)
{
	Mat src;
	src = imread(imgPath + imgName + ".jpg", 1);
	//imgTest.at<Vec3b>(tmpRow, tmpCol)
	// 检测纯色区域
	Mat pureTriRegion = Mat::zeros(src.size(), src.type());
	nrows = pureTriRegion.rows;

	// 计算每个三角形覆盖哪些像素
	vector< vector<Point2i> > triPixelIndex;
	vector<int> pureTriIndex; // 纯色三角形Index
	vector<Vec3b> pureTriAvgColor; // 纯色三角形平均颜色

	triPixelIndex.resize(F.rows());
	for (int row = 0; row < pureTriRegion.rows; row++) {
		for (int col = 0; col < pureTriRegion.cols; col++) {

			if (triangleID(col, nrows - row - 1) != -1) {
				triPixelIndex[triangleID(col, nrows - row - 1)].push_back(Point2i(col, row));
			}
		}
	}

	// 判断三角形像素是否包含边界
	for (int i = 0; i < F.rows(); i++) {
		bool nearEdge = FALSE;
		for (int j = 0; j < triPixelIndex[i].size(); j++) {
			if (SobelEdge.at<uchar>(triPixelIndex[i][j]) == 255) {
				nearEdge = TRUE;
				break;
			}
		}
		if (!nearEdge && triPixelIndex[i].size() > 0 && find(triangleVisit.begin(), triangleVisit.end(), i) == triangleVisit.end()) pureTriIndex.push_back(i);
	}

	// 分直方图统计颜色出现频率

	int hisInterval = 15;
	int hisNum = 256 / hisInterval + 1;
	vector<vector<int> >* colorCount = new vector<vector<int> >(hisNum * hisNum * hisNum);

	pureTriAvgColor.resize(pureTriIndex.size());
	for (int i = 0; i < pureTriIndex.size(); i++) {
		pureTriAvgColor[i] = src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][0]);
		for (int j = 1; j < triPixelIndex[pureTriIndex[i]].size(); j++) {
			Vec3b debug = src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][j]);
			pureTriAvgColor[i] = pureTriAvgColor[i] / 2.0 + src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][j]) / 2.0;
		}
		//cout << "index: " << pureTriIndex[i] << " avgColor: " << pureTriAvgColor[i] << endl;
		int cx, cy, cz;
		cx = pureTriAvgColor[i][0] / hisInterval;
		cy = pureTriAvgColor[i][1] / hisInterval;
		cz = pureTriAvgColor[i][2] / hisInterval;
		(*colorCount)[cx * hisNum * hisNum + cy * hisNum + cz].push_back(pureTriIndex[i]);
	}

	// 对聚类后的纯色三角形，进行压缩
	// 同一颜色区间内压缩为中值三角形
	unordered_set<int> tmpDeleteTriIndex; // 候选的删除项（如果一邻域内没有别的三角形需要删除就保留该三角形）
	for (int i = 0; i < colorCount->size(); i++) {
		if ((*colorCount)[i].size() > 1) {
			int refTriIndex = (*colorCount)[i][(*colorCount)[i].size() / 2];
			for (int j = 0; j < (*colorCount)[i].size(); j++) {
				for (int k = 0; k < 3; k++) {
					tmpDeleteTriIndex.insert((*colorCount)[i][j]);
				}
			}
		}
	}

	// BFS find compress region
	vector<bool> visit(rawMesh.triangles.size(), false);
	queue<int> tmpQueue;
	for (auto it = tmpDeleteTriIndex.begin(); it != tmpDeleteTriIndex.end(); it++) {
		if (visit[*it] == false) {
			visit[*it] = true;
			unordered_set<int> tmpRegion;
			tmpQueue.push(*it);
			while (!tmpQueue.empty()) {
				int nowIndex = tmpQueue.front();
				tmpQueue.pop();
				tmpRegion.insert(nowIndex);
				visit[nowIndex] = true;
				for (int k = 0; k < 3; k++) {
					int oppositeTri = oppositeHalfEdge[nowIndex * 3 + k] / 3;
					if (visit[oppositeTri] == false) {
						if (tmpDeleteTriIndex.find(oppositeTri) != tmpDeleteTriIndex.end() 
							|| deleteTriIndex.find(oppositeTri) != deleteTriIndex.end()) {
							tmpQueue.push(oppositeTri);
						}
					}
				}
			}
			if (tmpRegion.size() > 20) {
				for (auto it2 = tmpRegion.begin(); it2 != tmpRegion.end(); it2++) {
					deleteTriIndex.insert(*it2);
				}
			}
			
		}
	}

	//for (auto it = tmpDeleteTriIndex.begin(); it != tmpDeleteTriIndex.end(); it++) {
	//	bool flag = false;
	//	for (int k = 0; k < 3; k++) {
	//		int oppositeTri = oppositeHalfEdge[(*it) * 3 + k] / 3;
	//		if (tmpDeleteTriIndex.find(oppositeTri) != tmpDeleteTriIndex.end() || deleteTriIndex.find(oppositeTri) != deleteTriIndex.end()) {
	//			flag = true;
	//			break;
	//		}
	//	}
	//	if (flag) deleteTriIndex.insert(*it);
	//}
	

}

void CompresPure()
{
	Mat src;
	src = imread(imgPath + imgName + ".jpg", 1);
	//imgTest.at<Vec3b>(tmpRow, tmpCol)
	// 检测纯色区域
	Mat pureTriRegion = Mat::zeros(src.size(), src.type());
	nrows = pureTriRegion.rows;

	// 计算每个三角形覆盖哪些像素
	vector< vector<Point2i> > triPixelIndex;
	vector<int> pureTriIndex; // 纯色三角形Index
	vector<Vec3b> pureTriAvgColor; // 纯色三角形平均颜色

	triPixelIndex.resize(F.rows());
	for (int row = 0; row < pureTriRegion.rows; row++) {
		for (int col = 0; col < pureTriRegion.cols; col++) {

			if (triangleID(col, nrows - row - 1) != -1) {
				triPixelIndex[triangleID(col, nrows - row - 1)].push_back(Point2i(col, row));
			}
		}
	}

	// 判断三角形像素是否包含边界
	for (int i = 0; i < F.rows(); i++) {
		bool nearEdge = FALSE;
		for (int j = 0; j < triPixelIndex[i].size(); j++) {
			if (SobelEdge.at<uchar>(triPixelIndex[i][j]) == 255) {
				nearEdge = TRUE;
				break;
			}
		}
		if (!nearEdge && triPixelIndex[i].size()>0 && find(triangleVisit.begin(), triangleVisit.end(), i) == triangleVisit.end()) pureTriIndex.push_back(i);
	}

	// 分直方图统计颜色出现频率

	int hisInterval = 15;
	int hisNum = 256 / hisInterval + 1;
	vector<vector<int> >* colorCount = new vector<vector<int> >(hisNum * hisNum * hisNum);

	pureTriAvgColor.resize(pureTriIndex.size());
	for (int i = 0; i < pureTriIndex.size(); i++) {
		pureTriAvgColor[i] = src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][0]);
		for (int j = 1; j < triPixelIndex[pureTriIndex[i]].size(); j++) {
			Vec3b debug = src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][j]);
			pureTriAvgColor[i] = pureTriAvgColor[i] / 2.0 + src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][j]) / 2.0;
		}
		//cout << "index: " << pureTriIndex[i] << " avgColor: " << pureTriAvgColor[i] << endl;
		int cx, cy, cz;
		cx = pureTriAvgColor[i][0] / hisInterval;
		cy = pureTriAvgColor[i][1] / hisInterval;
		cz = pureTriAvgColor[i][2] / hisInterval;
		(*colorCount)[cx * hisNum * hisNum + cy * hisNum + cz].push_back(pureTriIndex[i]);
	}

	// 对聚类后的纯色三角形，进行压缩
	// 同一颜色区间内压缩为中值三角形
	for (int i = 0; i < colorCount->size(); i++) {
		if ((*colorCount)[i].size() > 1) {
			int refTriIndex = (*colorCount)[i][(*colorCount)[i].size() / 2];
			for (int j = 0; j < (*colorCount)[i].size(); j++) {
				for (int k = 0; k < 3; k++) {
					FTC((*colorCount)[i][j], k) = FTC(refTriIndex, k);
				}
			}
		}
	}

	for (int i = 0; i < pureTriIndex.size(); i++) {
		for (int j = 0; j < triPixelIndex[pureTriIndex[i]].size(); j++) {
			pureTriRegion.at<Vec3b>(triPixelIndex[pureTriIndex[i]][j]) = src.at<Vec3b>(triPixelIndex[pureTriIndex[i]][j]);
		}
	}

	imwrite(imgPath + imgName + "_pureTriRegion.png", pureTriRegion);

}

void CompressUV(const char* meshName) {
	//直接压缩掉的面
	RemeshMesh rawMesh;
	rawMesh.read(meshName);

	rawMesh.saveClean("tmp/outClean.obj");

	igl::readOBJ(meshName, V, TC, N, F, FTC, FN);

	// 1.压缩特征块区域
	for (int i = 0; i < blocks.size(); i++) {
		//if (i != 6 && i != 12)
		CompressFeature(i);
		DeleteFeature(rawMesh, i);
	}
	
	// 2.压缩非特征纯色区域

	DeletePure(rawMesh);
	CompresPure();


	rawMesh.deleteNum = deleteTriIndex.size();
	for (auto it = deleteTriIndex.begin(); it != deleteTriIndex.end(); it++) {
		rawMesh.deleteTri[*it] = 1;
	}

	
	rawMesh.save("tmp/outDelete.obj");
	//std::cout << "FTC:    " << std::endl << FTC << std::endl;
	igl::writeOBJ("tmp/outTest.obj", V, F, N, FN, TC, FTC);

}


#endif