////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      UsbCapture Code for robot
///ALL RIGHTS RESERVED
///@file:usb_capture_with_thread.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-7-25
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "usb_capture_with_opencv.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"

using namespace std;
using namespace cv;


UsbCaptureWithOpenCV::UsbCaptureWithOpenCV(){
    mIsInit= false;
    mIsOpen=false;
    mIsChangeFormat= false;
}

UsbCaptureWithOpenCV::~UsbCaptureWithOpenCV(){

}

int UsbCaptureWithOpenCV::init(){
    if(mIsOpen){
        return 0;
    }
/***1.open device*****/
    if(mCap.open(mVideoPath)){
        mCap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        mCap.set(cv::CAP_PROP_FRAME_WIDTH, mCaptureWidth);
        mCap.set(cv::CAP_PROP_FRAME_HEIGHT, mCaptureHeight);
        mIsOpen=true;
        return 0;
    }
    return -1;


}



int UsbCaptureWithOpenCV::init(std::string videoPath, int width, int height) {
    if(mIsOpen){
        return -1;
    }
    mVideoPath=videoPath;
    mCaptureWidth=width;
    mCaptureHeight=height;
    mIsInit= true;
    mIsCapture=true;
    int ret=init();
    //启动线程
    start();
    return ret;
}

bool UsbCaptureWithOpenCV::isOpen() {
    return mIsOpen;
}


int UsbCaptureWithOpenCV::changeVideoFormat(int width, int height){
    if((width!=mCaptureWidth)||(height!=mCaptureHeight)){
        mCaptureWidth=width;
        mCaptureHeight=height;
        mIsChangeFormat=true;
    }
    return 0;
}


int UsbCaptureWithOpenCV::GetImgRaw() {
    int ret=-1;
    if(mCap.isOpened()){
        if(mCap.read(mImgTmp)){
            ret=0;
        } else{
            ret=-2;
        }
    } else{
        ret= -1;
    }
    if(ret!=0){
        cout<<"cap-"<<mVideoPath<<"-error,reconnecting!"<<endl;
        //重连摄像头
        mIsOpen=false;
        if(mCap.isOpened()){
            mCap.release();
        }
        if(init()!=0){
            usleep(500000);//500ms
        }
        return ret;
    }
    pthread_mutex_lock(&imgMutex);
    mImgTmp.copyTo(mImg);//写入mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    mIsUpdate=true;
    return 0;
}

int UsbCaptureWithOpenCV::getImg(Mat &img){
    if(!mIsUpdate){
        //等待100ms
        int timeCounter=0;
        while(!mIsUpdate&&timeCounter<100){
            usleep(1000);//1ms等待
            timeCounter++;
        }
        if(!mIsUpdate){
            return -3;//更新超时
        }
    }
    pthread_mutex_lock(&imgMutex);
    mImg.copyTo(img);//读mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    if(!img.empty()&&(img.cols==mCaptureWidth)&&(img.rows==mCaptureHeight)){
        mIsUpdate= false;
        return 0;
    } else{
        return -1;
    }
}




void UsbCaptureWithOpenCV::run() {
    while(true){
        if(mIsChangeFormat){
            //更改图像分辨率
            mIsOpen=false;
            if(mCap.isOpened()){
                mCap.set(cv::CAP_PROP_FRAME_WIDTH, mCaptureWidth);
                mCap.set(cv::CAP_PROP_FRAME_HEIGHT, mCaptureHeight);
                mIsChangeFormat= false;
            }
        }
        //获取图片
        if(mIsCapture){
            GetImgRaw();
        } else{
            usleep(30000);//30ms
        }
    }
}

void UsbCaptureWithOpenCV::setCaptureState(bool isCapture) {
    mIsCapture=isCapture;
}
