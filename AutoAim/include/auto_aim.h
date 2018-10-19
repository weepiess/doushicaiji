#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "time.h"
#include "kalman_filter_by_opencv.h"

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
    Point2f calPitchAndYaw(float x, float y, float z);
    Rect rectROI;
    Kalman_filter kf;

private:
    const static float max_offset_angle;
    Point2f lastPoint;
    int IMG_WIDTH;
    int IMG_HEIGHT;
    int resizeCount;
    bool hasROI;
};                                                                                                                                  

#endif
