#include <iostream>
#include "auto_aim.h"
#include "usb_capture_with_thread.h"
#define red 1
#define blue 0
int main(int agrc, char *argv[]){
    UsbCaptureWithThread CAP;
    string camera_path;
    int wideth;
    int height;
    CAP.init(camera_path,wideth,height);
    AutoAim autoAim(wideth,height);
    while(CAP.isOpen()){
       clock_t start = clock();
       Mat src;
       Point2f angle;
       CAP.getImg(src);
       angle=autoAim.aim(src,red,0,0);
       clock_t finish = clock();
       double time_delay=start-finish;
       CAP.getImg(src);
       angle=autoAim.aim(src,red,1,time_delay);
    }
    return 0;
}
