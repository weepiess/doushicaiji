#include "kalman_filter_by_opencv.h"
Kalman_filter::Kalman_filter(){}
Kalman_filter::~Kalman_filter(){}

void Kalman_filter::init( int measureParams, double Noise ,int controlParams)
{
    Kf.init(4,measureParams,controlParams);
    Kf.transitionMatrix = transitionMatrix;
    //Kf.transitionMatrix=transitionMatrix;
    Kf.measurementMatrix=measurementMatrix;
    Kf.measurementNoiseCov=measurementNoiseCov;
    setIdentity(Kf.processNoiseCov, Scalar::all(Noise)); 
    //setIdentity(Kf.measurementNoiseCov, Scalar::all(NoisevCov));  
    setIdentity(Kf.errorCovPost, Scalar::all(1)); 
    randn(Kf.statePost, Scalar::all(0), Scalar::all(0.1)); 
}

Mat Kalman_filter::predict()
{
    Kf.statePost=statePost;
    return Kf.predict();
}

void Kalman_filter::correct(Mat measurement)
{
    Kf.correct(measurement);
}

