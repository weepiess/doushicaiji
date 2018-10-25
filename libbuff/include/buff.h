#ifndef BUFF_H
#define BUFF_H

#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

class Buff{
public:
Buff();
~Buff();

void setImage(Mat &image, Mat &mask);
void findcontours(Mat &src, Mat &image);
void anglesolover(Point &center);

private:
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Rect rect;
    Rect rect2;
    Point pt;
    vector<Point> rects;
    vector<Point> match;
    Rect ROI;
    double minVal = -1;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;
};                                                                                                                                  

#endif