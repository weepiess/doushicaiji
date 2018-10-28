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
using namespace cv;

ControlModel::ControlModel(){
}

ControlModel::~ControlModel() {


}

void ControlModel::init(RobotModel* robotModel){
    pRobotModel=robotModel;
    //配置文件
    cv::FileStorage f("../res/main_config.yaml", cv::FileStorage::READ);
    f["enemy_is_red"] >> mEnemyIsRed;//自瞄敌方颜色
    f.release();
    usleep(1000000);//等待1s，等摄像头稳定
    //初始模式初始化
    mSetMode=ROBOT_MODE_ZERO;
}

//串口数据接收处理入口
void ControlModel::serialListenDataProcess(SerialPacket recvPacket) {
//复杂自定义数据包，需要自定义解析单独处理
    //debug
    //SerialPortDebug::testSerialPortListenPrint(recvPacket);
    unsigned char CMD=recvPacket.getCMD();
   // cout<<"CMD:"<<(int)CMD<<endl;
    if(CMD_SERIAL_DATA_UPDATE==CMD){
        //底层数据更新,pitch/yaw
         pRobotModel->mcuDataUpdate(recvPacket.getFloatInBuffer(2),recvPacket.getFloatInBuffer(6));
    } else if(CMD_SERIAL_MINIPC_SHUTDOWN==CMD){
        //关机命令
        cout << "shutdown!!!!!!!!!!" << endl;
        system("shutdown -h now");
    } else{

    }
}



void ControlModel::processFSM(){
    //模式切换预处理
    if(mSetMode!=pRobotModel->getCurrentMode()){
        pRobotModel->setCurrentMode(mSetMode);
        switch (mSetMode){
            case ROBOT_MODE_ZERO:
                //模式0运行前控制代码
                break;
            case ROBOT_MODE_ONE:
                //模式1运行前控制代码
                break;
            default:
                break;
        }
    }
 

    //模式运行
    switch (pRobotModel->getCurrentMode()){
        case ROBOT_MODE_ZERO:
            //模式0运行控制代码
            break;
        case ROBOT_MODE_ONE:
            //模式1运行控制代码
            break;
        default:
            cout<<"[aiProcess]mode error"<<endl;
            break;
    }

}
