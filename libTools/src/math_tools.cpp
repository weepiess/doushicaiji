#include "math_tools.h"

double MathTools::gravityKiller(double z_distance,double y_distance,double bullet_speed,float current_pitch){
    constexpr double GRAVITY = 9.7913;
    double alpha = current_pitch*CV_PI/180;
    double x = -y_distance*sin(alpha)+z_distance*cos(alpha);
    double y = z_distance*sin(alpha)+y_distance*cos(alpha);
    double v = bullet_speed;
    double m = 2*((y*GRAVITY+v*v)-
            sqrt((y*GRAVITY+v*v)*(y*GRAVITY+v*v)-
                    GRAVITY*GRAVITY*(y_distance*y_distance+
                            z_distance*z_distance)))/
            (GRAVITY*GRAVITY);
    double time = sqrt(m);
    double beta = asin((2*y-GRAVITY*time*time)/(2*v*time))*180/CV_PI;
    double offset = beta-current_pitch;

    return offset;
}