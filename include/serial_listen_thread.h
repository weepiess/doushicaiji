////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Thread Code for robot
///ALL RIGHTS RESERVED
///@file:serial_listen_thread.h
///@brief: 串口监听线程控制。除了少量的数据校验，不包括数据处理，
/// 数据处理通过机器人控制模型（ControlModel）中的接口实现。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////


#ifndef RMDEMO_SERIAL_LISTEN_THREAD_H
#define RMDEMO_SERIAL_LISTEN_THREAD_H

#include "base_thread.h"
#include "control_model.h"

class SerialListenThread:public BaseThread{
public:
    SerialListenThread();
    ~SerialListenThread();

public:
    void init(RobotModel *robotModel,ControlModel *controlModel);
    void run();

private:
    RobotModel *pRobotModel;
    ControlModel *pControlModel;
    bool mExitFlag;

};



#endif //RMDEMO_SERIAL_LISTEN_THREAD_H
