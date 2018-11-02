////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_port_debug.h
///@brief: 串口调试模块。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-5-19
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#ifndef RMDEMO_SERIALPORT_DEBUG_H
#define RMDEMO_SERIALPORT_DEBUG_H

#include "serial_interface.h"
#include "serial_packet.h"

class SerialPortDebug{
public:
    SerialPortDebug();
    ~SerialPortDebug();
public:
    void init(SerialInterface *serialInterface);
    void testSerialPort();
    static void testSerialPortListenPrint(SerialPacket recvPacket);

private:
    SerialInterface *pSerialInterface;
};

#endif
