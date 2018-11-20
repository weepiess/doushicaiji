#ifndef INFANTRY_AUTO_AIM_H
#define INFANTRY_AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "base_aim.h"
#include "kalman_filter_by_opencv.h"
#include "usb_capture_with_opencv.h"

using namespace cv;
using namespace std;

class InfantryAutoAim: public BaseAim{
public:
    InfantryAutoAim();
    ~InfantryAutoAim();

public:
    //aim()的形式不固定，但是返回值必须是AimResult类型
    AimResult aim(cv::Mat &src, Point2f &pitYaw);
    void init(unsigned char color);

private:
    bool setImage(cv::Mat &src);
    void findLamp_rect(vector<cv::RotatedRect> &pre_armor_lamps); //搜索所有可能灯条
    void match_lamps(vector<cv::RotatedRect> &pre_armor_lamps, vector<cv::RotatedRect> &real_armor_lamps); //匹配灯条
    void select_armor(vector<cv::RotatedRect> real_armor_lamps); //锁定装甲板

    void initParams(unsigned char color);
    void resetROI();
    bool isGlobalSearch();
    bool rejudgeByTvec(double x, double y, double z);
    Point2d cal_x_y(cv::RotatedRect &rect, int is_up);

private:
    //ROI
    Rect rectROI;
    int resizeCount;
    cv::Mat mask;

    //灯条识别
    int frameCount; //每120帧做一次全局搜索，防止对方一个贴脸一个很远，结果一直锁定远处的
    Point3d lastValiableTvec;
    int farEnsureCount; //近处和远处的确认帧不同，往远处跳代表了命中率会降低，所以需要很大的确认帧数，近处跳代表会提升命中率，有较少的确认帧数
    int nearEnsureCount;
    int landspaceEnsureCount;
    cv::RotatedRect best_lamps[2];
    Point bestCenter;

    //卡尔曼滤波
    SimpleKalmanFilter simpleKF;
};                                                                                                      

#endif
