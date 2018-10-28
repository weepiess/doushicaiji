#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "time.h"
#include "kalman_filter_by_opencv.h"
#include "usb_capture_with_opencv.h"
#include "pnp_solver.h"
#include "math_tools.h"

using namespace cv;
using namespace std;

class AutoAim{
public:
    AutoAim();
    AutoAim(int width, int height,float dt_);
    ~AutoAim();

    Point2f aim(Mat src,vector<int> &best_lamps , float currPitch, float currYaw, int is_predict, double time_delay);

public:
    typedef struct 
    {
        Point2f point[4];
        int angle;
        int height;
        int width;
        int x;
        int y;
    }Armor_lamps;
    const static int color_red = 0;
    const static int color_blue = 1;
    void set_parameters(int angel,int inside_anlge, int height, int width);
    void setImage(Mat &src, int enemyColor);                                                      //图像预处理
    void findLamp_rect(Mat &img,vector<Armor_lamps> &pre_armor_lamps);                                     //搜索所有可能灯条
    void match_lamps(Mat &img,vector<Armor_lamps> &pre_armor_lamps,vector<Armor_lamps> &real_armor_lamps); //匹配灯条
    void select_armor(vector<Armor_lamps> real_armor_lamps,vector<int> &best_lamps);                      //锁定装甲板

private:
    
    int param_diff_angle;
    int param_inside_angle;
    int param_diff_height;
    int param_diff_width;
    int  cal_angle(Point2f point[4]);
    bool checkBorder();
    bool resizeROI(Rect &origin, Rect &current);
    void change_roi(int &x, int &y, int &width, int &hight);
    Point2f calPitchAndYaw(float x, float y, float z, float currPitch, float currYaw);

private:

    const static float max_offset_angle;
    PNPSolver pnpSolver;
    Point3d last_tvec;
    int IMG_WIDTH;
    int IMG_HEIGHT;
    int resizeCount;
    bool hasROI;
    //Mat best_lamps = Mat::zeros(8, 1, CV_32F); 
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
