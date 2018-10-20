#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "time.h"
#include "kalman_filter_by_opencv.h"
#include "usb_capture_with_opencv.h"

using namespace cv;
using namespace std;
class AutoAim{
public:
    AutoAim();
    AutoAim(int width, int height);
    ~AutoAim();

    enum Color{
        red, blue
    };

    bool checkBorder();
    void setImage(Mat &src, Mat &mask , Color enemyColor);
    void findLamp(Mat &mask, vector<RotatedRect> &lamps);
    void findBestArmor(Mat &src, vector<RotatedRect> &lamps, Point &bestCenter, vector<Point2f> &posAndSpeed,Mat &best_lamps, clock_t &start);
    bool resizeROI(Rect &origin, Rect &current);
    void test();
    Rect rectROI;
    Kalman_filter kf;
    Point2f aim(Mat &src, int is_red,int is_predict,double time_delay);

private:
    const static float max_offset_angle;
    Point2f lastPoint;
    int IMG_WIDTH;
    int IMG_HEIGHT;
    int resizeCount;
    bool hasROI;
    Mat camera_matrix;
    Mat distortion_coef;
    Mat best_lamps = Mat::zeros(8, 1, CV_32F); 
    Mat temp = Mat::zeros(8, 1, CV_32F);
    Mat measurement = Mat::zeros(4, 1, CV_32F);
    Mat rvec;
    Mat tvec;
    vector<Point3d> Points3D;

    Point bestCenter;
    
    vector<RotatedRect> lamps;
    vector<Point2f> posAndSpeed;
    int camera_is_open;
    Point2f calPitchAndYaw(float x, float y, float z);
};                                                                                                                                  

#endif
