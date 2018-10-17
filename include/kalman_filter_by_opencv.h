#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;
class Kalman_filter 
{
    public:
        Kalman_filter();
        ~Kalman_filter();
    public:
        //int dynamParams;
        int measureParams;
        int controlParams;
        //Mat transitionMatrix;
        Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)  
        Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))  
        Mat transitionMatrix;   //!< state transition matrix (A)  
        Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)  
        Mat measurementMatrix;  //!< measurement matrix (H)  
        Mat processNoiseCov;    //!< process noise covariance matrix (Q)  
        Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)  
        Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  
        Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)  
        Mat errorCovPost;   
    public:
        void init( int measureParams,double Noise,float dt,int controlParams);
        
        Mat predict();
        void correct(Mat measurement);
    private:
        KalmanFilter Kf;
                

};
/*example of using kalman_filter_by_opencv CV modle
Kalman_filter kf;
Mat measurement = Mat::zeros(4, 1, CV_32F); 
kf.transitionMatrix=(Mat_<float>(4, 4) <<   
            1,0,dt,0,   
            0,1,0,dt,   
            0,0,1,0,   
            0,0,0,1 );
kf.measurementMatrix=(Mat_<float>(4, 4) <<   
            1,0,0,0,   
            0,1,0,0,   
            0,0,1,0,   
            0,0,0,1 );
kf.measurementNoiseCov(Mat_<float>(4, 4) <<   
            2000,0,0,0,   
            0,2000,0,0,   
            0,0,10000,0,   
            0,0,0,10000 );
kf.init(4,1e-5,1e-1,1/50);
measurement.at<float>(0)= (float)x;  
measurement.at<float>(1) = (float)y;  
measurement.at<float>(2)= (float)vx;  
measurement.at<float>(3) = (float)vy;  
psoestate=kf.predict();
kf.correct(measurement)
*/
/*example of using kalman_filter_by_opencv CV modle
Kalman_filter kf;
Mat measurement = Mat::zeros(4, 1, CV_32F); 
kf.measurementMatrix=(Mat_<float>(4, 4) <<   
            1,0,0,0,0,0,   
            0,1,0,0,0,0,   
            0,0,1,0,0,0   
            0,0,0,1 );
kf.measurementNoiseCov(Mat_<float>(4, 4) <<   
            2000,0,0,0,   
            0,2000,0,0,   
            0,0,10000,0,   
            0,0,0,10000 );
kf.init(6,1e-1,1/50);
measurement.at<float>(0)= (float)x;  
measurement.at<float>(1) = (float)y;  
measurement.at<float>(2)= (float)vx;  
measurement.at<float>(3) = (float)vy;  
psoestate=kf.predict();
kf.correct(measurement)
*/




