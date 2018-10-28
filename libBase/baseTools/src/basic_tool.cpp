////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Tool Code for robot
///ALL RIGHTS RESERVED
///@file:basic_tool.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-13
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#include "basic_tool.h"
#include "iostream"

struct timeval BasicTool::start=startInitGet();

struct timeval BasicTool::startInitGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv;
}


int BasicTool::currentTimeMsGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (int)((tv.tv_sec-start.tv_sec)*1000+(tv.tv_usec-start.tv_usec)/1000);
}