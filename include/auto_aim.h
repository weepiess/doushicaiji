#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
// #include <opencv2/video/tracking.hpp>
// #include <opencv2/core/utility.hpp>


using namespace cv;
using namespace std;
#define red      1
#define blue     0
class AutoAim{
public:
    AutoAim();
    ~AutoAim();

    void setImage(Mat &src, Mat &mask);
    void findLamp(Mat &src, Mat &mask, vector<RotatedRect> &lamps);
    void findBestArmor(vector<RotatedRect> &lamps, Point &bestCenter);
    bool resize(Rect &origin, Rect &current);

private:
    const static float max_offset_angle;
    Point lastFitPoint;
    int lostFrameCount;
    int ensureFrameCount;
};

#endif