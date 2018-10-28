////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Tool Code for robot
///ALL RIGHTS RESERVED
///@file:basic_tool.h
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-13
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#ifndef SENTRYDEMO_BASIC_TOOL_H
#define SENTRYDEMO_BASIC_TOOL_H
#include <sys/time.h>
class BasicTool{
public:
    static struct timeval start;
    static struct timeval startInitGet();
    //获取当前时间距离程序启动时间的时间差
    static int currentTimeMsGet();
};

#endif //SENTRYDEMO_BASIC_TOOL_H
