#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include <base_aim.h>
#include "time.h"
#include "kalman_filter_by_opencv.h"
#include "usb_capture_with_opencv.h"

using namespace cv;
using namespace std;

class AutoAim: public BaseAim{
public:
    AutoAim();
    AutoAim(int width, int height,float dt_);
    ~AutoAim();

public:
    typedef struct {
        Point2f point[4];
        int angle;
        int height;
        int width;
        int x;
        int y;
    }Armor_lamps;

    BaseAim::AimResult aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw);

private:
    bool setImage(Mat &src);
    float cal_angle(Point2f point[4]);
    void resetROI();
    void findLamp_rect(Mat &img, vector<Armor_lamps> &pre_armor_lamps); //搜索所有可能灯条
    void match_lamps(Mat &img, vector<Armor_lamps> &pre_armor_lamps, vector<Armor_lamps> &real_armor_lamps); //匹配灯条
    void select_armor(vector<Armor_lamps> real_armor_lamps, Mat &best_lamps); //锁定装甲板

private:
    int param_diff_angle;
    int param_inside_angle;
    int param_diff_height;
    int param_diff_width;

    Point3d last_tvec;
    int resizeCount;
    Mat best_lamps = Mat::zeros(8, 1, CV_32F); 
    Mat temp = Mat::zeros(8, 1, CV_32F);
    Mat measurement = Mat::zeros(6, 1, CV_32F);
    Mat mask;
    float dt;
    Point bestCenter;
    Point Armorsize;
    
    vector<RotatedRect> lamps;

    bool camera_is_open;

    Rect rectROI;
    Kalman_filter kf;
};                                                                                                      

#endif
