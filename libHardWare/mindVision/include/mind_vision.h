#ifndef MIND_VISION_H
#define MIND_VISION_H

#include "CameraApi.h" //相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

class MindVision{
public:
    MindVision();
    ~MindVision();

public:
    int init();
    int startPlay(int hCamera_);
    int setResolution(int width, int height);
    int getImg(Mat &src);

private:
    unsigned char           * g_pRgbBuffer;       //处理后数据缓存区
    int                     iCameraCounts = 1;    //找到的相机的个数
    int                     iStatus=-1;           //状态返回值
    int                     hCamera=-1;           //相机句柄
    tSdkCameraDevInfo       tCameraEnumList;      //设备列表信息
    tSdkCameraCapbility     tCapability;          //设备描述信息
    tSdkFrameHead           sFrameInfo;           //每一帧的头指针
    BYTE*			        pbyBuffer;            //数据缓冲区
    IplImage                *iplImage = NULL;     //图像的矩阵存储形式     
    int                     channel=3;            //通道数
    tSdkImageResolution     tResolution;          //像素
};

#endif