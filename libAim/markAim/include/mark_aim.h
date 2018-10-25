#ifndef MARK_AIM_H
#define MARK_AIM_H

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class MarkAim{
public:
    MarkAim();
    ~MarkAim();

public:
    void setImage(Mat &image, Mat &mask);
};

#endif