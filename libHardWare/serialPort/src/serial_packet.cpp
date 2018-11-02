////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for Robot
///ALL RIGHTS RESERVED
///@file:serial_packet.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-5-17
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include <cstring>
#include "serial_packet.h"

SerialPacket::SerialPacket(){
    clearPakcet();
    mInitflag= false;
}

SerialPacket::~SerialPacket(){

}

void SerialPacket::creatPacket(unsigned char CMD){
    buffer[0]=0xff;//帧头
    buffer[mLen-1]=0x0d;//帧尾
    mCMD=CMD;
    buffer[1]=mCMD;//命令
    mInitflag= true;
}

unsigned char  SerialPacket::getCMD(){
    return  mCMD;
}
/**************自定义装载数据***********************/
void SerialPacket::setUncharInBuffer(unsigned char data,int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-1){ //越界检查
        buffer[locationInBuffer]=data;
    }
}

void SerialPacket::setShortIntInBuffer(short int data,int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-2){//越界检查
        memcpy(buffer+locationInBuffer, &data,2);
    }
}

void SerialPacket::setIntInBuffer(int data,int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-4){//越界检查
        memcpy(buffer+locationInBuffer, &data,4);
    }

}

void SerialPacket::setFloatInBuffer(float data,int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-4){//越界检查
        memcpy(buffer+locationInBuffer, &data,4);
    }
}
/**************自定义解析数据***********************/
unsigned char SerialPacket::getUncharInBuffer(int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-1){ //越界检查
        return buffer[locationInBuffer];
    }
}


short int SerialPacket::getShortIntInBuffer(int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-2){ //越界检查
        short int data;
        memcpy(&data,buffer+locationInBuffer, 2);
        return data;
    }

}

int SerialPacket::getIntInBuffer(int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-4){ //越界检查
        int data;
        memcpy(&data,buffer+locationInBuffer, 4);
        return data;
    }
}

float SerialPacket::getFloatInBuffer(int locationInBuffer){
    if(locationInBuffer>0&&locationInBuffer<mLen-4){ //越界检查
        float data;
        memcpy(&data,buffer+locationInBuffer, 4);
        return data;
    }
}

//数据帧解析，包含数据帧检查
int SerialPacket::unPacking(){
    //检查帧头，帧尾
    if(buffer[0]==0xff&&buffer[mLen-1]==0x0d){
        mCMD=buffer[1];//解析得到命令
        mInitflag=true;
        return 0;
    }
    return 1;

}
//数据帧清空
void SerialPacket::clearPakcet(){
    memset(buffer, 0, sizeof(char)*mLen);  //每个字节都用0填充
    mInitflag= false;
}

