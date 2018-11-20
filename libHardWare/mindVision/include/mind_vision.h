#ifndef MIND_VISION_H
#define MIND_VISION_H

#include "CameraApi.h" //相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>

#define CAMERA_PARAMS_ADJUST

using namespace cv;
using namespace std;

class MindVision{
public:
    MindVision();
    ~MindVision();

public:
    /** 该公司相机的初始化，使用之前必须调用此函数得到可以使用的相机句柄
     * @return: int, 返回可用相机的句柄
     */
    int init();

    /** 根据传入的相机句柄，初始化对应的相机
     * @param: hCamera_, 相机句柄
     * @param: color, 敌方颜色
     * @return: int, 错误码
     */
    int startPlay(int hCamera_, unsigned char color);

    /** 设置相机的分辨率
     * @param: width, 横向像素
     * @param: height, 纵向像素
     * @return: int, 错误码
     */
    int setResolution(int width, int height);

    /** 得到图像
     * @param: src, 得到的图像
     * @return: int, 错误码
     */
    int getImg(Mat &src);

    /** 根据传入的字符调整不同的参数
     * @param: c, 键盘输入的字符
     * @return: void
     */
    void adjustParams(unsigned char c);

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

    #ifdef CAMERA_PARAMS_ADJUST
    /** 当前的操作是作用在哪个相机元素上
     * "f"：frameSpeed  "g"：gamma  "e"：exposureTime  "c"：contrast  "a":aeState
     */
    unsigned char currOperation;

    /** 用于调整相机参数的临时变量*/
    int index, gamma, exposureTime, contrast, aeState, resolution_width, resolution_height;
    #endif
};

#endif