////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_port_debug.cpp
///@brief: 机器人串口调试模块
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-5-19
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "serial_port_debug.h"

using namespace std;

SerialPortDebug::SerialPortDebug() {

}

SerialPortDebug::~SerialPortDebug() {

}

void SerialPortDebug::init(SerialInterface *serialInterface) {
    pSerialInterface=serialInterface;
}

void SerialPortDebug::testSerialPort() {
    cout<<"start to test serial port,press 'c' to continue,press 'q' to quit!"<<endl;
    char c=getchar();
    cout<<"云台增量测试，a:pitch5;   s:yaw10;  d:pitch-5"<<endl;
    cout<<"云台增量测试(慢速)，z:yaw10;   x:yaw10,yaw10"<<endl;
    cout<<"f绝对角:pitch0,yaw0;    g:pitch-10,yaw10"<<endl;
    cout<<"发射子弹测试，h:shoot"<<endl;
    cout<<"以下指令请无线调试："<<endl;
    cout<<"底盘随机模式：j"<<endl;
    cout<<"底盘停止：n"<<endl;
    cout<<"云台全局扫描模式,k"<<endl;
    cout<<"云台局部扫描模式,m"<<endl;
    cout<<"底盘测试，l,to 250cm"<<endl;
    cout<<"please input..."<<endl;

    while(c!='q') {
        c = getchar();
        switch (c){
            case 'a':
                cout<<"YunTaiDeltaSet(5,0),test...."<<endl;
                pSerialInterface->YunTaiDeltaSet(5,0);
                cout<<"please input..."<<endl;
                break;
            case 's':
                cout<<"YunTaiDeltaSet(0,10),test...."<<endl;
                pSerialInterface->YunTaiDeltaSet(0,10);
                cout<<"please input..."<<endl;
                break;
            case 'd':
                cout<<"YunTaiDeltaSet(-5,0),test...."<<endl;
                pSerialInterface->YunTaiDeltaSet(-5,10);
                cout<<"please input..."<<endl;
                break;
            case 'q':
                cout<<"quit"<<endl;
                break;
        }

    }
    cout<<"test end!"<<endl;
}

void SerialPortDebug::testSerialPortListenPrint(SerialPacket recvPacket) {
    unsigned CMD=recvPacket.getCMD();
    if(CMD==CMD_SERIAL_ABS_YUNTAI_DELTA){
        float pitch=recvPacket.getFloatInBuffer(2);
        float yaw=recvPacket.getFloatInBuffer(6);
        short int location = recvPacket.getShortIntInBuffer(10);
        cout << "data update,pitch:" <<pitch<<"yaw:"<<yaw<<"location:"<<location << endl;
    }else{

    }
}
