#include "kalman_filter_by_opencv.h"

Kalman_filter::Kalman_filter(){}
Kalman_filter::~Kalman_filter(){}

void Kalman_filter::init( int measureParams, double Noise ,int controlParams){
    Kf.init(6,measureParams,controlParams);
    Kf.transitionMatrix = transitionMatrix;
    Kf.measurementMatrix = measurementMatrix;
    Kf.measurementNoiseCov = measurementNoiseCov;
    setIdentity(Kf.processNoiseCov, Scalar::all(Noise)); 
    setIdentity(Kf.errorCovPost, Scalar::all(1)); 
    
}

Mat Kalman_filter::predict(){
    Kf.statePost=statePost;
    return Kf.predict();
}

void Kalman_filter::correct(Mat measurement){
    Kf.correct(measurement);
}