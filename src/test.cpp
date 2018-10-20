#include <iostream>
#include "auto_aim.h"
#include "usb_capture_with_thread.h"
#define red 1
#define blue 0
int main(int agrc, char *argv[]){
    /*
    UsbCaptureWithThread CAP;
    string camera_path;
    int wideth;
    int height;
    CAP.init(camera_path,wideth,height);
    AutoAim autoAim(wideth,height);
    while(CAP.isOpen()){
       Mat src;
       Point2f angle;
       CAP.getImg(src);
       angle=autoAim.aim(src,red,0);
       CAP.getImg(src);
       angle=autoAim.aim(src,red,1);
    }*/
    
    autoAim.test();
    return 0;
}
