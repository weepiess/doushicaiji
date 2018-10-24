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
    Point2f aim(Mat &src, int color,int is_predict,double time_delay);
    void change_roi(int &x, int &y, int &width, int &hight);

public:
    Rect rectROI;
    Kalman_filter kf;
    const static int color_red = 0;
    const static int color_blue = 1;

private:
    int key=0;
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
    int rectx,recty,rectwidth,recthight;
    Point bestCenter;
    Point Armorsize;
    vector<RotatedRect> lamps;
    vector<Point2f> posAndSpeed;
    int camera_is_open;
    Point2f calPitchAndYaw(float x, float y, float z);
};                                                                                                                                  

#endif
