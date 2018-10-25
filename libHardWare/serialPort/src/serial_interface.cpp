////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_interface.cpp
///@brief: 机器人控制基本接口源文件，包含对车底盘及云台的基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "serial_interface.h"

SerialInterface::SerialInterface(){

};


SerialInterface::~SerialInterface(void) {

};

int SerialInterface::init(std::string devPath){
    if(mMcuSerialPort.init(devPath)){
        return 0;
    }else{
        return -1;
    }
}

bool SerialInterface::isOpen(){
    return mMcuSerialPort.isOpen();
}

int SerialInterface::dataSend(SerialPacket sendPacket){
    if(mMcuSerialPort.Send(sendPacket.buffer,16)==16){
        return 0;
    }
    return -1;

}
int SerialInterface::dataRecv(SerialPacket &recvPacket){

    if(mMcuSerialPort.Recv(recvPacket.buffer,16)==16) {
        if(recvPacket.unPacking()==0){
            return 0;
        }
    }
    return -1;
}


/*********************控制接口**************************/

void SerialInterface::chassisStopSet() {
    SerialPacket sendPacket;
    sendPacket.creatPacket(CMD_SERIAL_CHASSIS_STOP_SET);
    dataSend(sendPacket);
}



void SerialInterface::YunTaiDeltaSet(float pitch,float yaw){
    SerialPacket sendPacket;
    sendPacket.creatPacket(CMD_SERIAL_YUNTAI_DELTA_SET);
    sendPacket.setFloatInBuffer(pitch,2);
    sendPacket.setFloatInBuffer(yaw,6);
    sendPacket.setUncharInBuffer(0x00,10);
    dataSend(sendPacket);
}


void SerialInterface::getAbsYunTaiDelta(){
    SerialPacket recvPacket;
    recvPacket.creatPacket(CMD_SERIAL_ABS_YUNTAI_DELTA);
    dataSend(recvPacket);
}


void SerialInterface::YunTaiShoot(unsigned char num){
    SerialPacket sendPacket;
    sendPacket.creatPacket(CMD_SERIAL_SHOOT);
    sendPacket.setUncharInBuffer(num,2);
    dataSend(sendPacket);
}


