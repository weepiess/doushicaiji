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
    void setCameraMatrix(float fx, float fy, float cx, float cy);
    /** 设置相机畸变矩阵
     * 四个参数分别为x,y径向和切向畸变
     */
    void setDistortionCoef(float rdx, float rdy, float tdx, float tdy);

    /** 压入三维世界坐标点, 默认以传入正方体的中心作为坐标原点，并且以xy平面的顺时针方向
     * @param: is_z_overlook, 决定z的顺序，true则物体向下倾斜
     */
    void pushPoints3D(double x_length, double y_length, double z_length = 0, bool is_z_overlook = false);
    void clearPoints3D();

    /** 压入二维点注意
     * 压入的顺序需要和 pushPoints3D() 一致
     */
    void pushPoints2D(const Point2d points);
    void clearPoints2D();

    /** 默认使用迭代法
     * SOLVEPNP_ITERATIVE速度较慢，并且三维坐标的四个点必须共面
     * SOLVEPNP_P3P和SOLVEPNP_EPNP不要求点共面，P3P适用于四个点的情况，EPNP适用于五个点及以上的情况
     */
    void solvePnP(bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE);
    Point3d getTvec();

    //debug用于输出所有的参数
    void showParams();

private:
    //相机参数矩阵
    cv::Mat camera_matrix;
    //相机畸变矩阵
    cv::Mat distortion_coef;
    //世界坐标系
    vector<Point3d> Points3D;
    //相机坐标系二维点
    vector<Point2d> Points2D;

    cv::Mat tvec;
    cv::Mat rvec;
};

#endif