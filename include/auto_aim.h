#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class AutoAim{
public:
    AutoAim();
    ~AutoAim();

    void setImage(Mat &src, Mat &mask);
    void findLamp(Mat &src, Mat &mask, vector<RotatedRect> &lamps);
    void findBestArmor(vector<RotatedRect> &lamps, Point &bestCenter);

private:
    const static float max_offset_angle;
    Point lastFitPoint;
    int lostFrameCount;
    int ensureFrameCount;
};

#endif