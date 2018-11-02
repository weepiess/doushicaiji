#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "base_aim.h"
#include "time.h"
#include "kalman_filter_by_opencv.h"
#include "usb_capture_with_opencv.h"

using namespace cv;
using namespace std;

class AutoAim: public BaseAim{
public:
    AutoAim();
    ~AutoAim();

public:
    //aim()的形式不固定，但是返回值必须是AimResult类型
    AimResult aim(Mat &src, Point2f &pitYaw);
    void set_parameters(int angle,int inside_angle, int height, int width);
    void init(int width, int height, float dt_);

private:
    bool setImage(Mat &src);
    void resetROI();
    void findLamp_rect(Mat &img, vector<RotatedRect> &pre_armor_lamps); //搜索所有可能灯条
    void match_lamps(Mat &img, vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps); //匹配灯条
    void select_armor(vector<RotatedRect> real_armor_lamps); //锁定装甲板

private:
    int param_diff_angle;
    int param_inside_angle;
    int param_diff_height;
    int param_diff_width;

    Point3d last_tvec;
    int resizeCount;
    RotatedRect best_lamps[2];
    Mat measurement = Mat::zeros(6, 1, CV_32F);
    float dt;
    Point bestCenter;
    Mat mask;

    Rect rectROI;
    Kalman_filter kf;
};                                                                                                      

#endif
