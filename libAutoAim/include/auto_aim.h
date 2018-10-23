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

    bool checkBorder();
    void setImage(Mat &src, Mat &mask , int enemyColor);
    void findLamp(Mat &mask, vector<RotatedRect> &lamps);
    void findBestArmor(vector<RotatedRect> &lamps, Point &bestCenter, vector<Point2f> &posAndSpeed,Mat &best_lamps, clock_t &start);
    bool resizeROI(Rect &origin, Rect &current);
    double gravityKiller(double z_distance,double y_distance,double bullet_speed,float current_pitch);
    Point2f aim(Mat &src, int color, float currPitch, float currYaw, int is_predict,double time_delay);

public:
    Rect rectROI;
    Kalman_filter kf;
    const static int color_red = 0;
    const static int color_blue = 1;

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
    Point2f calPitchAndYaw(float x, float y, float z, float currPitch, float currYaw);
};                                                                                                                                  

#endif
