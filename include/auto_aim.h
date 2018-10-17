#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "time.h"

using namespace cv;
using namespace std;

class AutoAim{
public:
    AutoAim();
    ~AutoAim();

    void setImage(Mat &src, Mat &mask);
    void findLamp(Mat &mask, vector<RotatedRect> &lamps,Mat &src);
    void findLamp(Mat &src, Mat &mask, vector<RotatedRect> &lamps);
    void findBestArmor(vector<RotatedRect> &lamps, Point &bestCenter, vector<Point2f> &posAndSpeed, clock_t &start);

private:
    const static float max_offset_angle;
    Point lastFitPoint;
    int lostFrameCount;
    int ensureFrameCount;
};

#endif