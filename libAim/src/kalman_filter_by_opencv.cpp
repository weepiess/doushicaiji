#include "kalman_filter_by_opencv.h"

SimpleKalmanFilter::SimpleKalmanFilter(){}
SimpleKalmanFilter::~SimpleKalmanFilter(){}

void SimpleKalmanFilter::init(int dynamParams, int measureParams, int controlParams, int type, 
                                float processNoiseCov, float measurementNoiseCov, float statePostCov){
    //对Kalman初始化
    kFilter.init(dynamParams, measureParams, controlParams, type);
    //对各项参数初始化
    setIdentity(kFilter.measurementMatrix);
    setIdentity(kFilter.processNoiseCov, Scalar::all(processNoiseCov));
    setIdentity(kFilter.measurementNoiseCov, Scalar::all(measurementNoiseCov));
    setIdentity(kFilter.errorCovPost, Scalar::all(1));
    //假设初始状态的状态误差是个高斯分布，满足均值为0，方差为给定值
    randn(kFilter.statePost, Scalar::all(0), Scalar::all(statePostCov));
    //初始化观测矩阵
    measurement = Mat::zeros(measureParams, 1, CV_32F);
}