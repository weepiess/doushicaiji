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
    //VideoCapture cap("/dev/xRMAimVideo");
    //VideoCapture cap(1);
    //cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    //cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    
    AutoAim autoAim(1280,720,1/50);
    
    autoAim.set_parameters(5,60,30,20);
    //SerialInterface sInterface;
    //sInterface.init("/dev/ttyUSB0");
    //SerialPacket recvPacket;
    float pitch=1; 
    float yaw=1;
    Mat src=imread("../res/pic-final/my_photo-194.jpg");
    //imshow("s",src);
    //waitKey(0);
    
    //while(cap.isOpened()){
       //clock_t start = clock();
       Point2f angle;
       vector<AutoAim::Armor_lamps> pre_armor_lamps;
       vector<AutoAim::Armor_lamps> real_armor_lamps;
       vector<int> best_lamps; //= Mat::zeros(8, 1, CV_32F); 
       //if(CAP.getImg(src)!=0) break;
       //cap>>src;
       //sInterface.getAbsYunTaiDelta();
       //while(sInterface.dataRecv(recvPacket)!=0){}
       //pitch = recvPacket.getFloatInBuffer(2);
       //yaw = recvPacket.getFloatInBuffer(6);
       
       autoAim.setImage(src,autoAim.color_red);
       
       autoAim.findLamp_rect(src,pre_armor_lamps);
       
       autoAim.match_lamps(src,pre_armor_lamps,real_armor_lamps);
       
       autoAim.select_armor(real_armor_lamps,best_lamps);
       angle=autoAim.aim(src,best_lamps,pitch,yaw,0,0);
       cout<<angle<<endl;

        imshow("src",src);
        waitKey(0);
       //if(angle.x==180 && angle.y==180) continue;
       //sInterface.YunTaiDeltaSet(angle.x, angle.y);
       //clock_t finish = clock();
       //double time_delay=start-finish;
       //char c = waitKey(1);
       //if(c == 27) break;
       //CAP.getImg(src);
       //angle=autoAim.aim(src,autoAim.red,1,time_delay);
    //}
    return 0;
}
