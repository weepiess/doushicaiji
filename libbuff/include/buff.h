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
void Match(Mat &src, Mat &image);
void findLED(Mat &src, Mat &image);
void hashcal(Mat &src,Mat &image);
void findbigbuff(Mat &src,Mat &image);

private:
    /*
    For matchtemplete
    
    double minVal = -1;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;
    */
    /*
    For findLED
    大小符均可使用
    */
    vector<Rect> roisaver;
    Rect LEDroi;
    Rect ROI;
    Mat roi;
    int min_y=0x3f3f3f3f;
    int max_x=0;
    int min_x=0x3f3f3f3f;
    vector<Point> position;
    float max=0;
    float max2=0;
};                                                                                                                                  

#endif