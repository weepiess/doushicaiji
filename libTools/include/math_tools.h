#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class MathTools{

public:
    //pitch轴发射角消除重力
    static double gravityKiller(double z_distance, double y_distance, double bullet_speed, double current_pitch);
    
};

#endif
