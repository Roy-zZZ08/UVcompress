#ifndef  INIT_H
#define  INIT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <queue>
#include <thread>
#include <cmath>
#include <algorithm>
#include <Windows.h>
#include <bitset>
#include "Block.h"

//   klt
#include "error.h"
#include "base.h"
#include "pnmio.h"
#include "klt.h"

using namespace std;
using namespace cv;

int threadNum = 16;
int blockSize;
int blockSizeWidth, blockSizeHeight;
int ncols, nrows;


KLT_FeatureList testFl[16] = { nullptr };

float NMSth; //相似性区域的非极大值抑制阈值
float simiThread = 0.6; 
float equalBlockThread = 1300;
int minBlockSize = 10;
int edgeThread = 40; //Sobel算子计算后认为可以作为边缘的阈值
int matchNum = 0;
vector<Block*> blocks; //作为搜索起始点的区域信息 

mutex mtx; // protect img
mutex mtxSeed;

const string imgPath = ".\\tmp\\";
string imgName = "bunny";

//===========================================================================
// Sobel边缘算子，计算得到8位单通道边缘强度信息图
//===========================================================================

Mat sobelEdge(Mat src)
{
    Mat dst, gray;
    dst = Mat::zeros(src.size(), src.type());

    Mat kernel = (Mat_<int>(2, 2) << 0, 1, -1, 0);
    filter2D(src, dst, -1, kernel, cv::Point(-1, -1), 0.0);
    cvtColor(src, gray, COLOR_BGR2GRAY);
    Mat xgrad, ygrad;
    Sobel(gray, xgrad, CV_16S, 1, 0, 3);
    Sobel(gray, ygrad, CV_16S, 0, 1, 3);
    convertScaleAbs(xgrad, xgrad);
    convertScaleAbs(ygrad, ygrad);

    addWeighted(xgrad, 0.5, ygrad, 0.5, 0, dst);

    //===================debug===========
    imwrite(imgPath + imgName + "_sobel.png", dst);
    waitKey();
    //===================debug===========

    return dst;
}

//===========================================================================
//  由预处理的Sobel信息，泛洪得到候选起始区域
//  过滤重复区域，并计算每个特征的ID Mask
//===========================================================================

void InitBlocks(Mat src) 
{
    Mat Sobel = sobelEdge(src);
    vector<pair<Point2i, Point2i> > searchReigon; // startPoint & width & height
    vector<vector<Point2i> > regionBoundary; // 候选区域的边缘信息

    // 泛洪计算候选不连通边缘区域
    Mat visitMap = Mat::zeros(Sobel.size(), Sobel.type());
    for (int row = 0; row < Sobel.rows; row++) {
        for (int col = 0; col < Sobel.cols; col++) {
            if ((int)visitMap.at<uchar>(row, col) == 0) {
                visitMap.at<uchar>(row, col) = 1;
                if ((int)Sobel.at<uchar>(row, col) > edgeThread) {

                    Point2i startPoint = Point2i(row, col);
                    Point2i endPoint = Point2i(row, col);
                    queue<Point2i> tmpPoints;
                    vector<Point2i> tmpBound;
                    tmpPoints.push(startPoint);
                    tmpBound.push_back(startPoint);
                    while (!tmpPoints.empty()) {
                        Point2i nowPoint = tmpPoints.front();
                        visitMap.at<uchar>(nowPoint.x, nowPoint.y) = 1;
                        tmpPoints.pop();
                        if (nowPoint.x < startPoint.x)
                            startPoint.x = nowPoint.x;
                        if (nowPoint.y < startPoint.y)
                            startPoint.y = nowPoint.y;
                        if (nowPoint.x > endPoint.x)
                            endPoint.x = nowPoint.x;
                        if (nowPoint.y > endPoint.y)
                            endPoint.y = nowPoint.y;
                        if (nowPoint.x - 1 >= 0 && nowPoint.y - 1 >= 0 && (int)visitMap.at<uchar>(nowPoint.x - 1, nowPoint.y - 1) == 0 && (int)Sobel.at<uchar>(nowPoint.x - 1, nowPoint.y - 1) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x - 1, nowPoint.y - 1));
                            visitMap.at<uchar>(nowPoint.x - 1, nowPoint.y - 1) = 1;
                        }
                        if (nowPoint.x - 1 >= 0 && (int)visitMap.at<uchar>(nowPoint.x - 1, nowPoint.y) == 0 && (int)Sobel.at<uchar>(nowPoint.x - 1, nowPoint.y) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x - 1, nowPoint.y));
                            tmpBound.push_back(Point2i(nowPoint.x - 1, nowPoint.y));
                            visitMap.at<uchar>(nowPoint.x - 1, nowPoint.y) = 1;
                        }
                        if (nowPoint.x - 1 >= 0 && nowPoint.y + 1 < Sobel.cols && (int)visitMap.at<uchar>(nowPoint.x - 1, nowPoint.y + 1) == 0 && (int)Sobel.at<uchar>(nowPoint.x - 1, nowPoint.y + 1) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x - 1, nowPoint.y + 1));
                            tmpBound.push_back(Point2i(nowPoint.x - 1, nowPoint.y + 1));
                            visitMap.at<uchar>(nowPoint.x - 1, nowPoint.y + 1) = 1;
                        }
                        if (nowPoint.x + 1 < Sobel.rows && nowPoint.y - 1 >= 0 && (int)visitMap.at<uchar>(nowPoint.x + 1, nowPoint.y - 1) == 0 && (int)Sobel.at<uchar>(nowPoint.x + 1, nowPoint.y - 1) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x + 1, nowPoint.y - 1));
                            tmpBound.push_back(Point2i(nowPoint.x + 1, nowPoint.y - 1));
                            visitMap.at<uchar>(nowPoint.x + 1, nowPoint.y - 1) = 1;
                        }
                        if (nowPoint.x + 1 < Sobel.rows && (int)visitMap.at<uchar>(nowPoint.x + 1, nowPoint.y) == 0 && (int)Sobel.at<uchar>(nowPoint.x + 1, nowPoint.y) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x + 1, nowPoint.y));
                            tmpBound.push_back(Point2i(nowPoint.x + 1, nowPoint.y));
                            visitMap.at<uchar>(nowPoint.x + 1, nowPoint.y) = 1;
                        }
                        if (nowPoint.x + 1 < Sobel.rows && nowPoint.y + 1 < Sobel.cols && (int)visitMap.at<uchar>(nowPoint.x + 1, nowPoint.y + 1) == 0 && (int)Sobel.at<uchar>(nowPoint.x + 1, nowPoint.y + 1) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x + 1, nowPoint.y + 1));
                            tmpBound.push_back(Point2i(nowPoint.x + 1, nowPoint.y + 1));
                            visitMap.at<uchar>(nowPoint.x + 1, nowPoint.y + 1) = 1;
                        }
                        if (nowPoint.y - 1 >= 0 && (int)visitMap.at<uchar>(nowPoint.x, nowPoint.y - 1) == 0 && (int)Sobel.at<uchar>(nowPoint.x, nowPoint.y - 1) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x, nowPoint.y - 1));
                            tmpBound.push_back(Point2i(nowPoint.x, nowPoint.y - 1));
                            visitMap.at<uchar>(nowPoint.x, nowPoint.y - 1) = 1;
                        }
                        if (nowPoint.y + 1 < Sobel.cols && (int)visitMap.at<uchar>(nowPoint.x, nowPoint.y + 1) == 0 && (int)Sobel.at<uchar>(nowPoint.x, nowPoint.y + 1) > edgeThread) {

                            tmpPoints.push(Point2i(nowPoint.x, nowPoint.y + 1));
                            tmpBound.push_back(Point2i(nowPoint.x, nowPoint.y + 1));
                            visitMap.at<uchar>(nowPoint.x, nowPoint.y + 1) = 1;
                        }
                    }
                    int boundaryPad = 2;
                    if (startPoint.x > boundaryPad && startPoint.y > boundaryPad && endPoint.x < Sobel.rows - boundaryPad && endPoint.y < Sobel.cols - boundaryPad) {
                        Point2i tmpBlockSize = endPoint - startPoint;
                        if (tmpBlockSize.x > minBlockSize && tmpBlockSize.y > minBlockSize && tmpBlockSize.x < Sobel.rows / 4 && tmpBlockSize.y < Sobel.cols / 4) {
                            Point2i kltRegionSize = Point2i((endPoint.x - startPoint.x + 2) / 2 * 2, ((endPoint.y - startPoint.y + 2) / 2 * 2));
                            searchReigon.push_back(make_pair(startPoint, kltRegionSize));
                            regionBoundary.push_back(tmpBound);
                        }
                    }
                }
            }
        }
    }

    // filter the regions
    vector<vector<Point2i> > finalBoundary; //保存有效区域的边界信息

    int blocksIndex = 0;
    blocks.push_back(new Block(blocksIndex++, searchReigon[0].second.x, searchReigon[0].second.y, searchReigon[0].first.x, searchReigon[0].first.y));
    finalBoundary.push_back(regionBoundary[0]);
    blocks[0]->computeColorHistogram(src);
    for (int i = 1; i < searchReigon.size(); i++) {
        Block* tmpBlock = new Block(-1, searchReigon[i].second.x, searchReigon[i].second.y, searchReigon[i].first.x, searchReigon[i].first.y);
        tmpBlock->computeColorHistogram(src);
        bool flag = false;
        for (int j = 0; j < blocks.size(); j++) {
            int compare_method = 3; // CV_COMP_BHATTACHARYYA
            if (abs(blocks[j]->getSizeWidth() - tmpBlock->getSizeWidth()) < blocks[j]->getSizeWidth() / 3 && abs(blocks[j]->getSizeHeight() - tmpBlock->getSizeHeight()) < blocks[j]->getSizeHeight() / 3) {
                double simi = compareHist(blocks[j]->getHist(), tmpBlock->getHist(), compare_method);
                //cout << "simi: block " << j << " region " << i << " :" << simi << endl;
                if (simi < simiThread) {
                    flag = true;
                    break;
                }
            }
        }
        if (!flag) {
            tmpBlock->index = blocksIndex++;
            blocks.push_back(tmpBlock);
            finalBoundary.push_back(regionBoundary[i]);
        }
    }
    
    Mat result1, result2;
    src.copyTo(result1);
    src.copyTo(result2);

    for (int i = 0; i < searchReigon.size(); i++) {
        Rect testRect;
        testRect.x = searchReigon[i].first.y;
        testRect.y = searchReigon[i].first.x;
        testRect.width = searchReigon[i].second.y;
        testRect.height = searchReigon[i].second.x;
        cv::rectangle(result1, testRect, cv::Scalar(255, 0, 0));
    }
    //===================debug===========
    cv::imshow("result1", result1);
    imwrite(imgPath + imgName + "_start0.png", result1);
    waitKey();
    //===================debug===========
    
    // 计算 feature mask
    for (int index = 0; index < blocks.size(); index++) {
        Mat imgBlock(blocks[index]->getSizeHeight(), blocks[index]->getSizeWidth(), CV_8UC3);
        for (int i = 0; i < imgBlock.rows; i++) {
            for (int j = 0; j < imgBlock.cols; j++) {
                imgBlock.at<Vec3b>(i, j) = src.at<Vec3b>(blocks[index]->getStartHeight() + i, blocks[index]->getStartWidth() + j);
            }
        }
        Mat imgidBlock = Mat::zeros(blocks[index]->getSizeHeight(), blocks[index]->getSizeWidth(), Sobel.type());
        Mat idBlock = Mat::zeros(blocks[index]->getSizeHeight(), blocks[index]->getSizeWidth(), Sobel.type());
        for (int i = 0; i < finalBoundary[index].size(); i++) {
            int terminal = 0;
            imgidBlock.at<uchar>(finalBoundary[index][i].x - blocks[index]->getStartHeight(), finalBoundary[index][i].y - blocks[index]->getStartWidth()) = 255;
        }
        for (int i = 0; i < idBlock.rows; i++) {
            for (int j = 0; j < idBlock.cols; j++) {
                idBlock.at<uchar>(i, j) = 255;
            }
        }
        // 使用四个方向扫描线计算ID map
        for (int i = 0; i < idBlock.rows; i++) {
            int jleft = 0, jright = idBlock.cols - 1;
            while (jleft < idBlock.cols && imgidBlock.at<uchar>(i, jleft) != 255) idBlock.at<uchar>(i, jleft++) = 0;
            while (jright >= 0 && imgidBlock.at<uchar>(i, jright) != 255) idBlock.at<uchar>(i, jright--) = 0;
        }
        for (int j = 0; j < idBlock.cols; j++) {
            if (j == 57)
                int debug = 0;
            int iup = 0, idown = idBlock.rows - 1;
            while (iup < idBlock.rows && imgidBlock.at<uchar>(iup, j) != 255) idBlock.at<uchar>(iup++, j) = 0;
            while (idown >= 0 && imgidBlock.at<uchar>(idown, j) != 255) idBlock.at<uchar>(idown--, j) = 0;
        }
        imshow("idBlock" + index, idBlock);
        waitKey();
        blocks[index]->featureID = idBlock;

        //===================debug===========
        Rect testRect;
        testRect.x = blocks[index]->getStartWidth();
        testRect.y = blocks[index]->getStartHeight();
        testRect.width = blocks[index]->getSizeWidth();
        testRect.height = blocks[index]->getSizeHeight();
        cv::rectangle(result2, testRect, cv::Scalar(0, 0, 255));
        //===================debug===========
    }

    //===================debug===========
    cv::imshow("result2", result2);
    imwrite(imgPath + imgName + "_start.png", result2);
    waitKey();
    //===================debug===========
}

//===========================================================================
// 对于一个起始搜索区域，遍历种子点找到所有候选的klt计算起点
//===========================================================================

void FindInitMatch(int start, int end, int blockIndex, vector<Block*> seedBlocks) {

    // color histogram simi
    for (int index = start; index < end; index++) {

        printf("\rcompute block simi[%.2f%%]         ", (index - start) * 100.0 / (end - start));

        int compare_method = 3; // CV_COMP_BHATTACHARYYA
        double simi = compareHist(blocks[blockIndex]->getHist(), seedBlocks[index]->getHist(), compare_method);
        //cout << i << " method: " << compare_method << " simi:" << simi << endl;

        //Correlation ( CV_COMP_CORREL )

        if (simi < simiThread) {
            //cout << i << " simi:" << simi << endl;
            //float testTheta = guessTheta(blocks[index]->getHog(), seedBlocks[i]->getHog());
            //cout << "index " << i << " theta " << testTheta << endl;
            for (int theta = 0; theta < 360; theta += 60) {
                float scale = seedBlocks[index]->getScale();
                Point2f move = Point2f(seedBlocks[index]->getStartWidth() * scale - blocks[blockIndex]->getStartWidth(),
                    seedBlocks[index]->getStartHeight() * scale - blocks[blockIndex]->getStartHeight());
                mtxSeed.lock();
                blocks[blockIndex]->addInitMatch(move, theta, scale);
                matchNum++;
                mtxSeed.unlock();
            }
        }
    }
}

//===========================================================================
// 对每一个搜索区域，生成对应的查找种子点
//===========================================================================

void GenerateSeed(Mat src, int blockIndex)
{
    vector<Block*> seedBlocks;
    // generate pyramid
    Mat pyramid1, pyramid2, pyramid3;
    resize(src, pyramid1, Size(src.cols / 1.5, src.rows / 1.5), (0, 0), (0, 0));
    resize(pyramid1, pyramid2, Size(pyramid1.cols / 1.5, pyramid1.rows / 1.5), (0, 0), (0, 0));
    resize(pyramid2, pyramid3, Size(pyramid2.cols / 1.5, pyramid2.rows / 1.5), (0, 0), (0, 0));

    // generate seedPoints
    blockSizeHeight = blocks[blockIndex]->getSizeHeight();
    blockSizeWidth = blocks[blockIndex]->getSizeWidth();

    int seedBlockIndex = 0;
    for (int row = 0; row + blockSizeHeight <= src.rows; row += blockSizeHeight / 4) {
        for (int col = 0; col + blockSizeWidth <= src.cols; col += blockSizeWidth / 4) {
            printf("\rgenerate seedPoints[%.2f%%]   ", row * 100.0 / src.rows);
            Block* tmpBlock = new Block(seedBlockIndex++, blockSizeHeight, blockSizeWidth, row, col);
            tmpBlock->computeColorHistogram(src);
            seedBlocks.push_back(tmpBlock);
        }
    }

    for (int row = 0; row + blockSizeHeight <= pyramid1.rows; row += blockSizeHeight / 4) {
        for (int col = 0; col + blockSizeWidth <= pyramid1.cols; col += blockSizeWidth / 4) {
            printf("\rgenerate seedPoints pyramid1[%.2f%%]   ", row * 100.0 / pyramid1.rows);
            Block* tmpBlock = new Block(seedBlockIndex++, blockSizeHeight, blockSizeWidth, row, col, 1.5);
            tmpBlock->computeColorHistogram(pyramid1);
            seedBlocks.push_back(tmpBlock);
        }
    }

    for (int row = 0; row + blockSizeHeight <= pyramid2.rows; row += blockSizeHeight / 4) {
        for (int col = 0; col + blockSizeWidth <= pyramid2.cols; col += blockSizeWidth / 4) {
            printf("\rgenerate seedPoints pyramid2[%.2f%%]   ", row * 100.0 / pyramid2.rows);
            Block* tmpBlock = new Block(seedBlockIndex++, blockSizeHeight, blockSizeWidth, row, col, 1.5 * 1.5);
            tmpBlock->computeColorHistogram(pyramid2);
            seedBlocks.push_back(tmpBlock);
        }
    }

    for (int row = 0; row + blockSizeHeight <= pyramid3.rows; row += blockSizeHeight / 4) {
        for (int col = 0; col + blockSizeWidth <= pyramid3.cols; col += blockSizeWidth / 4) {
            printf("\rgenerate seedPoints pyramid3[%.2f%%]   ", row * 100.0 / pyramid3.rows);
            Block* tmpBlock = new Block(seedBlockIndex++, blockSizeHeight, blockSizeWidth, row, col, 1.5 * 1.5 * 1.5);
            tmpBlock->computeColorHistogram(pyramid3);
            seedBlocks.push_back(tmpBlock);
        }
    }
    cout << endl;

    blocks[blockIndex]->addInitMatch(Point2f(0.0, 0.0), 0, 1);
    // accelerate using thread
    vector<thread> t(threadNum);
    for (int i = 0; i < threadNum; i++) {
        t[i] = thread(FindInitMatch, i * seedBlocks.size() / threadNum, (i + 1) * seedBlocks.size() / threadNum, blockIndex, seedBlocks);
    }
    for (int i = 0; i < threadNum; i++) {
        t[i].join();
    }

    for (auto it = seedBlocks.begin(); it != seedBlocks.end(); it++) {
        if (*it != NULL) {
            delete* it;
            *it = NULL;
        }
    }
    seedBlocks.clear();
}

//===========================================================================
//  图片转为灰度Uchar数组，以供klt输入
//===========================================================================

uchar* ImgToGrayUchar(Mat src, int* ncols, int* nrows)
{
    uchar* ptr;
    *ncols = src.cols;
    *nrows = src.rows;
    ptr = (uchar*)malloc((*ncols) * (*nrows) * sizeof(char));
    if (ptr == NULL)
        KLTError("(imgRead) Memory not allocated");

    Mat imgGray;
    cvtColor(src, imgGray, COLOR_BGR2GRAY);

    uchar* tmpptr = ptr;
    for (int i = 0; i < *nrows; i++)
    {
        for (int j = 0; j < *ncols; j++)
        {
            *tmpptr = imgGray.at<uchar>(i, j);
            tmpptr++;
        }
    }
    return ptr;
}

//===========================================================================
// 对于初始化好的区域匹配结果，进行klt匹配优化其仿射匹配结果
//===========================================================================

void FindingSimi(int start, int end, uchar* img, int blockIndex, int threadIndex)
{
    KLT_TrackingContext tc;

    int tmpMatchNum = end - start;

    tc = KLTCreateTrackingContext(blockSizeHeight, blockSizeWidth);

    testFl[threadIndex] = initialAffineTrack(blocks[blockIndex], tmpMatchNum, start, end);
    myTrackAffine(tc, img, ncols, nrows, testFl[threadIndex]);

    return;

}

//===========================================================================
// 对于klt匹配计算后的结果，进行非极大值抑制保留最佳匹配效果
//===========================================================================

bool myCompare(pair<pair<float, float>, pair<float, int> > a, pair<pair<float, float>, pair<float, int> > b)
{
    return a.second.first < b.second.first;
}

void ApplyNMS(vector<vector<pair<pair<float, float>, pair<float, int> > > >& NMSlist,
    vector<vector<pair<pair<float, float>, pair<float, int> > > >& affineList, int blockIndex)
{
    // {aff_x aff_y error featureIndex}
    int eachThreadCount = blocks[blockIndex]->initMatchList.size() / threadNum;
    for (int threadIndex = 0; threadIndex < threadNum; threadIndex++) {
        for (int i = 0; i < testFl[threadIndex]->nFeatures; i++) {
            if (testFl[threadIndex]->feature[i]->val == KLT_TRACKED) {
                // Init NMS 
                // should be the center of the aff map
                NMSlist[testFl[threadIndex]->feature[i]->block_index].push_back(
                    make_pair(make_pair(testFl[threadIndex]->feature[i]->affineCenterX,
                        testFl[threadIndex]->feature[i]->affineCenterY),
                        make_pair(testFl[threadIndex]->feature[i]->error, i + threadIndex * eachThreadCount)));
            }
        }
    }


    //Apply NMS for each Block's matchlist
    int i = blockIndex;
    // sort by error
    sort(NMSlist[i].begin(), NMSlist[i].end(), myCompare);
    affineList[i].push_back(NMSlist[i][0]);
    for (int index = 1; index < NMSlist[i].size(); index++) {
        float minDist = 1e9;
        for (int j = 0; j < affineList[i].size(); j++) {
            // compute the distance between two affine region
            float tmpDist = pow(affineList[i][j].first.first - NMSlist[i][index].first.first, 2) +
                pow(affineList[i][j].first.second - NMSlist[i][index].first.second, 2);
            if (tmpDist < minDist)
                minDist = tmpDist;
        }
        if (minDist > NMSth) {
            affineList[i].push_back(NMSlist[i][index]);
        }
    }

    for (int j = 0; j < affineList[i].size(); j++) {
        int featureIndex = affineList[i][j].second.second;
        int tmpThreadIndex = featureIndex / eachThreadCount;
        if (tmpThreadIndex == threadNum)
            tmpThreadIndex--;
        int tmpFeatureIndex = featureIndex - tmpThreadIndex * eachThreadCount;
        Mat M = Mat::zeros(cv::Size(2, 3), CV_64F);
        double* m = M.ptr<double>();

        m[0] = testFl[tmpThreadIndex]->feature[tmpFeatureIndex]->aff_Axx;
        m[1] = testFl[tmpThreadIndex]->feature[tmpFeatureIndex]->aff_Axy;
        m[2] = testFl[tmpThreadIndex]->feature[tmpFeatureIndex]->aff_x;
        m[3] = testFl[tmpThreadIndex]->feature[tmpFeatureIndex]->aff_Ayx;
        m[4] = testFl[tmpThreadIndex]->feature[tmpFeatureIndex]->aff_Ayy;
        m[5] = testFl[tmpThreadIndex]->feature[tmpFeatureIndex]->aff_y;
        blocks[i]->finalMatchList.push_back(Match(M));
    }

}

//===========================================================================
// KLT 处理起点
//===========================================================================

void InitKlt() 
{
    // 读入图片
    Mat src;
    src = imread(imgPath + imgName + ".jpg", 1);

    InitBlocks(src);
    
    int blockIndex = 1; //当前搜索的区域索引
    GenerateSeed(src, blockIndex);
   
    uchar* initImg = ImgToGrayUchar(src, &ncols, &nrows);

    //==============================
    // 多线程进行klt匹配计算

    vector<thread> t(threadNum);
    int eachThreadCount = blocks[blockIndex]->initMatchList.size() / threadNum;
    for (int i = 0; i < threadNum - 1; i++) {
        t[i] = thread(FindingSimi, i * eachThreadCount, (i + 1) * eachThreadCount, initImg, blockIndex, i);
    }
    t[threadNum - 1] = thread(FindingSimi, (threadNum - 1) * eachThreadCount, blocks[blockIndex]->initMatchList.size(), initImg, blockIndex, threadNum - 1);
    for (int i = 0; i < threadNum; i++) {
        t[i].join();
    }
    //==============================

    //==============================
    // 非极大值抑制
    NMSth = blockSizeHeight * blockSizeWidth / 8.0;
    // {aff_x aff_y error featureIndex}
    vector<vector<pair<pair<float, float>, pair<float, int> > > >NMSlist(blocks.size()), affineList(blocks.size());
    ApplyNMS(ref(NMSlist), ref(affineList), blockIndex);

    /*for (int i = 0; i < blocks.size(); i++) {
        if (blocks[i]->equalBlock != -1) {
            for (vector<Match>::iterator it = blocks[blocks[i]->equalBlock]->finalMatchList.begin(); it != blocks[blocks[i]->equalBlock]->finalMatchList.end(); it++)
                blocks[i]->finalMatchList.push_back(*it);
        }
    }*/

    // Reconstructed Test
    Mat img1 = imread(imgPath + imgName + ".jpg", 1);
    Mat imgTest(nrows, ncols, CV_8UC3);
    namedWindow("Test");

    for (int i = 0; i < blocks[blockIndex]->finalMatchList.size(); i++) {
        Mat M = blocks[blockIndex]->finalMatchList[i].getMatrix();
        double* m = M.ptr<double>();
        for (int row = -blockSizeHeight / 2; row < blockSizeHeight / 2; row++) {
            for (int col = -blockSizeWidth / 2; col < blockSizeWidth / 2; col++) {
                int tmpCol = (int)(m[0] * col + m[1] * row + m[2]);
                int tmpRow = (int)(m[3] * col + m[4] * row + m[5]);
                if (tmpCol < ncols && tmpCol >= 0 && tmpRow < nrows && tmpRow >= 0) {
                    //imgTest.at<Vec3b>(tmpRow, tmpCol) = img1.at<Vec3b>(row + blocks[blockIndex]->getStartHeight()+ blockSize / 2, col + blocks[blockIndex]->getStartWidth() + blockSize / 2);
                    imgTest.at<Vec3b>(tmpRow, tmpCol) = img1.at<Vec3b>(tmpRow, tmpCol);
                }
            }
        }
        imshow("image", imgTest);
        waitKey();
    }
    
    imwrite(imgPath + imgName + "_test.png", imgTest);
    imshow("image", imgTest);
    waitKey();


    //CreatingCharts();

}

float computeDiff(int index1, int index2, const Mat& img)
{
    float diff = 0;
    int height1 = blocks[index1]->getStartHeight();
    int height2 = blocks[index2]->getStartHeight();
    int width1 = blocks[index1]->getStartWidth();
    int width2 = blocks[index2]->getStartWidth();

    for (int i = 0; i < blockSizeHeight; i++) {
        for (int j = 0; j < blockSizeWidth; j++) {
            // BGR
            float t10 = float(img.at<Vec3b>(height1 + i, width1 + j)[0]);
            float t11 = float(img.at<Vec3b>(height1 + i, width1 + j)[1]);
            float t12 = float(img.at<Vec3b>(height1 + i, width1 + j)[2]);
            float t20 = float(img.at<Vec3b>(height2 + i, width2 + j)[0]);
            float t21 = float(img.at<Vec3b>(height2 + i, width2 + j)[1]);
            float t22 = float(img.at<Vec3b>(height2 + i, width2 + j)[2]);

            diff += abs(t10 - t20);
            diff += abs(t11 - t21);
            diff += abs(t12 - t22);
        }
    }
    return diff;
}

Mat img;
Rect rect;
int startHeight, startWidth;

void showimage()
{
    Mat result;
    img.copyTo(result);
    cv::rectangle(result, rect, cv::Scalar(0, 0, 255));
    cv::imshow("imgRead", result);
}

void onMouse(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        startWidth = x;
        startHeight = y;
        rect.x = x;
        rect.y = y;
        rect.width = 1;
        rect.height = 1;
        break;
    case cv::EVENT_MOUSEMOVE:
        if (flags & cv::EVENT_FLAG_LBUTTON) {
            rect = Rect(cv::Point(rect.x, rect.y), cv::Point(x, y));
            blockSizeWidth = (x - rect.x) / 2 * 2;
            blockSizeHeight = (y - rect.y) / 2 * 2;

            printf("\rblockSizeWidth: %d  blockSizeHeight: %d   ", blockSizeWidth, blockSizeHeight);
            //cout << "x: " << x << " y: " << y << endl;
            showimage();
        }
        break;
    case cv::EVENT_LBUTTONUP:
        if (rect.width > 1 && rect.height > 1) {
            showimage();
        }
    default:
        break;
    }
}





#endif // ! INIT_H
