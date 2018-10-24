#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class PNPSolver{
public:
    PNPSolver();
    ~PNPSolver();

public:
    void setCameraMatrix(float a, float b, float c, float d, float e, float f, float g, float h, float i);
    void setDistortionCoef(float a, float b, float c, float d, float e);

    void pushPoints3D(double x, double y, double z);
    void clearPoints3D();

    void pushPoints2D(const Point2d points);
    void clearPoints2D();

    //默认使用迭代法
    void solvePnP(bool useExtrinsicGuess = false, int flags = 0);
    Point3d getTvec();

    void showParams();

private:
    //相机参数矩阵
    Mat camera_matrix;
    //相机畸变矩阵
    Mat distortion_coef;
    //世界坐标系
    vector<Point3d> Points3D;
    //相机坐标系二维点
    vector<Point2d> Points2D;

    Mat tvec;
    Mat rvec;
};

#endif