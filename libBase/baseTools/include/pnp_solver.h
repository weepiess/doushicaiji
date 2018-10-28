////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2019      tool for pnp
///ALL RIGHTS RESERVED
///@file:pnp_solver.h
///@brief: 封装pnp解算函数。
///@vesion 1.0
///@author: wyx
///@email: 1418555317@qq.com
///@date: 2018.10.27
///修订历史: 
////////////////////////////////////////////////////////////////////////////////
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
    //设置相机参数矩阵
    void setCameraMatrix(float a, float b, float c, float d, float e, float f, float g, float h, float i);
    //设置相机畸变矩阵
    void setDistortionCoef(float a, float b, float c, float d, float e);

    //压入三维世界坐标点
    void pushPoints3D(double x, double y, double z);
    void clearPoints3D();

    //压入二维点
    //注意压入的顺序需要和 pushPoints3D() 一致
    void pushPoints2D(const Point2d points);
    void clearPoints2D();

    //默认使用迭代法
    //SOLVEPNP_ITERATIVE速度较慢，并且三维坐标的四个点必须共面
    //SOLVEPNP_P3P和SOLVEPNP_EPNP不要求点共面，P3P适用于四个点的情况，EPNP适用于五个点及以上的情况
    void solvePnP(bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE);
    Point3d getTvec();

    //debug用于输出所有的参数
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