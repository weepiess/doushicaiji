////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      SerialPort Code for Robot
///ALL RIGHTS RESERVED
///@file:serial_packet.h
///@brief: 串口通信数据包封装，包含数据包打包以及解析等相关接口。
///@vesion 1.0(版本号)
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-5-17
///修订历史：
////////////////////////////////////////////////////////////////////////////////


#ifndef RMSERIALPORTDEMO_SERIAL_PACKET_H
#define RMSERIALPORTDEMO_SERIAL_PACKET_H


class SerialPacket{
public:
    SerialPacket();
    ~SerialPacket();

public:
    //数据包缓存buffer
    unsigned char buffer[16];
private:

    int mLen=16;//定义数据包缓存buffer长度
    bool mInitflag;//初始化标志位
    unsigned char mCMD;

public:
    /** 构造数据包,填充为buffer二进制数据
    *  @param:  unsigned char CMD，发送的命令，（一个数据帧至少包含一个命令）
    *  @return: void
    */
    void creatPacket(unsigned char CMD);
    unsigned char  getCMD();
    /***************自定义装载数据***************/
    void setUncharInBuffer(unsigned char data,int locationInBuffer);//unchar类型数据
    void setShortIntInBuffer(short int data,int locationInBuffer);//short int类型数据
    void setIntInBuffer(int data,int locationInBuffer);//int类型数据
    void setFloatInBuffer(float data,int locationInBuffer);//float类型数据
    /*******自定义解析数据************/
    unsigned char getUncharInBuffer(int locationInBuffer);//unchar类型数据
    int getIntInBuffer(int locationInBuffer);//short int类型数据
    short int getShortIntInBuffer(int locationInBuffer);//int类型数据
    float getFloatInBuffer(int locationInBuffer);//float类型数据
    /** 解析数据帧，包含数据帧检查（解析CMD）
    *  @param:  void
    *  @return: int，错误号，0代表无错误
    */
    int unPacking();
    //buffer清空
    void clearPakcet();

};


#endif //RMSERIALPORTDEMO_SERIAL_PACKET_H
