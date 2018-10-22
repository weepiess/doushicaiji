#include <iostream>
#include "auto_aim.h"
#include "usb_capture_with_thread.h"
#include "serial_interface.h"

int main(int agrc, char *argv[]){
    //UsbCaptureWithThread CAP;
    string camera_path;
    int width = 1280;
    int height = 720;
    //CAP.init("/dev/video0",width,height);
    VideoCapture cap("/dev/xRMAimVideo");
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    AutoAim autoAim(width,height);
    SerialInterface sInterface;
    sInterface.init("/dev/ttyUSB0");
    Mat src;
    while(cap.isOpened() && sInterface.isOpen()){
cout<<"hhhhhhhhh!"<<endl;
       //clock_t start = clock();
       Point2f angle;
       //if(CAP.getImg(src)!=0) break;
       cap>>src;
       angle=autoAim.aim(src,AutoAim::color_red,0,0);
       cout<<angle<<endl;
       if(angle.x==180 && angle.y==180)
		continue;
       sInterface.YunTaiDeltaSet(angle.x, angle.y);
//	getchar();
       //clock_t finish = clock();
       //double time_delay=start-finish;
       //char c = waitKey(1);
       //if(c == 27) break;
       //CAP.getImg(src);
       //angle=autoAim.aim(src,autoAim.red,1,time_delay);
    }
    return 0;
}
