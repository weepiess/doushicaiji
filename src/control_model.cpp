////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Model Code for robot
///ALL RIGHTS RESERVED
///@file:control_model.cpp
///@brief: robot 控制模型，包含对所有应用的管理，创建应用，并改变应用，以及监听操作手
/// 的指令，并处理。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "image_tool.h"
#include "control_model.h"
#include "basic_tool.h"
#include "serial_port_debug.h"
#include "usb_capture_with_thread.h"
#include <stdlib.h>
using namespace cv;

ControlModel::ControlModel(){}

ControlModel::~ControlModel(){}

void ControlModel::init(RobotModel* robotModel){
    pRobotModel=robotModel;
    autoAim.init(1280, 720);
    hCamera = mVision.init();
    mVision.startPlay(hCamera);
    //配置文件
    cv::FileStorage f("../res/main_config.yaml", cv::FileStorage::READ);
    f["enemy_is_red"] >> mEnemyIsRed;//自瞄敌方颜色
    f.release();
    usleep(1000000);//等待1s，等摄像头稳定
    //初始模式初始化
    mSetMode=ROBOT_MODE_AUTOAIM;
}

//串口数据接收处理入口
void ControlModel::serialListenDataProcess(SerialPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    //debug
    //SerialPortDebug::testSerialPortListenPrint(recvPacket);
    unsigned char CMD= recvPacket.getCMD();
   // cout<<"CMD:"<<(int)CMD<<edl;
    if(CMD_SERIAL_ABS_YUNTAI_DELTA==CMD){
        //底层数据更新,pitch/yaw
         pRobotModel->mcuDataUpdate(recvPacket.getFloatInBuffer(2),recvPacket.getFloatInBuffer(6));
         autoAim.setPitchAndYaw(recvPacket.getFloatInBuffer(2), recvPacket.getFloatInBuffer(6));
    } else if(CMD_SERIAL_MINIPC_SHUTDOWN==CMD){
        //关机命令
        cout << "shutdown!!!!!!!!!!" << endl;
        system("shutdown -h now");
    }
}

void ControlModel::processFSM(){
    //模式切换预处理
    if(mSetMode!=pRobotModel->getCurrentMode()){
        pRobotModel->setCurrentMode(mSetMode);
        switch (mSetMode){
            case ROBOT_MODE_AUTOAIM:{
                autoAim.set_parameters(3, 70, 0, 0);
                autoAim.setEnemyColor(BaseAim::color_red);
                autoAim.openPredict();
                break;
            }
        }
    }
 
    //模式运行
    switch (pRobotModel->getCurrentMode()){
        case ROBOT_MODE_AUTOAIM:{
            ImageTool::timeInit();
            autoAim.setPitchAndYaw(0, 0);
            cv::Mat src;
            //UsbCaptureWithThread *cap = pRobotModel->getpUsbCapture();
            if(mVision.getImg(src)!=0) break;
            Point2f angle;
            //报错invalid use of non-static member function，应该是在函数中不能去调用其他指针的函数
            //AimResult result = autoAim.aim(src, pRobotModel->getCurrentPitch, pRobotModel->getCurrentYaw, py);
            autoAim.aim(src, angle);
            string s = "src";
            imshow(s, src);
            waitKey(1);
            break;
        }
        default:
            cout<<"[aiProcess]mode error"<<endl;
            break;
    }

}