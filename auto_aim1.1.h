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
    void findLamp(Mat &mask, vector<RotatedRect> &lamps, Mat &src);
    void decide(Mat &src,vector<RotatedRect> &dec, vector<RotatedRect> &memory);

public:
    const static float max_offset_angle;
};

#endif