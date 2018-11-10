#ifndef KALMAN_FILTER_BY_OPENCV_H
#define KALMAN_FILTER_BY_OPENCV_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class SimpleKalmanFilter{
public:
    SimpleKalmanFilter();
    ~SimpleKalmanFilter();

public:
    //初始化与KalmanFilter相同，给出了一些初始状态时的默认值
    void init(int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F, float processNoiseCov = 1e-5, 
                float measurementNoiseCov = 1e-1, float statePostCov = 0.1);
public:
    //opencv自带的卡尔曼类,需要预测时直接调用该变量的函数，不对其做封装，减少数据复制和赋值
    KalmanFilter kFilter;

    //观测矩阵
    cv::Mat measurement;
};

#endif