#ifndef CAM_H
#define CAM_H
#define DEBUG
#include <stdlib.h>
#include <iostream>
#include <string>
#include "OpenNI.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace openni;

class cam{

    public:
    #ifdef DEBUG
    #endif // DEBUG
    cam();
    ~cam();
    void Camera_Init(Device& device);


    private:

    Mat region;
    const int nFrames = 9; 


    void filter(Mat &Pointvalue);
    void PixelFilter(Mat &image);
    void framfilter(Mat &src0,Mat &src1,Mat &src2,Mat &src3,Mat &src4, Mat &image, int size);
    void contourfinder(Mat &src,Mat &image);
    void stagefinder(Mat &src,Mat &image);




};
#endif