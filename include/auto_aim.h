#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "time.h"

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

    void setImage(Mat &src, Mat &mask , Color enemyColor);
    void findLamp(Mat &mask, vector<RotatedRect> &lamps);
    void findBestArmor(vector<RotatedRect> &lamps, Point &bestCenter);
    bool resizeROI(Rect &origin, Rect &current);
    Rect rectROI;

private:
    const static float max_offset_angle;
    int IMG_WIDTH;
    int IMG_HEIGHT;
    int resizeCount;
    bool hasROI;
};

#endif