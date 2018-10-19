////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      UsbCapture Code for robot
///ALL RIGHTS RESERVED
///@file:usb_capture.cpp
///@brief: 基于v4l2的封装相机操作。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "usb_capture.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"

using namespace std;
using namespace cv;


static int xioctl(int fd, int request, void *arg)
{
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

UsbCapture::UsbCapture(){
    mVideofd=-1;
    mIsInit= false;
    mIsOpen=false;
    mIsMjpeg=true;
}

UsbCapture::~UsbCapture(){
    if(mVideofd!=-1){
        close(mVideofd);
    }
}

int UsbCapture::init(){
    if(mIsOpen){
        return 0;
    }
/***1.open device*****/
    mVideofd = open(mVideoPath.c_str(), O_RDWR);
    if (mVideofd == -1)
    {
        printf("Opening video device error\n");
        return 1;
    }
/***2.set format*****/
    if(setVideoFormat(mCaptureWidth,mCaptureHeight,mIsMjpeg)!=0){
        return 2;
    }
/***3.mmap******/
    if(init_mmap()!=0){
        return 3;
    }
    refreshVideoFormat();
    mIsOpen=true;
    return 0;
}



int UsbCapture::init(std::string videoPath, int width, int height, bool mjpeg) {
    mVideoPath=videoPath;
    mCaptureWidth=width;
    mCaptureHeight=height;
    mIsMjpeg=mjpeg;
    mIsInit= true;
    return init();
}

bool UsbCapture::isOpen() {
    return mIsOpen;
}

int UsbCapture::setVideoFormat(int width, int height, bool MJPEG){
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    if (MJPEG == true){
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        mImgFormat=V4L2_PIX_FMT_MJPEG;
    }else{
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        mImgFormat=V4L2_PIX_FMT_YUYV;
    }
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (-1 == xioctl(mVideofd, VIDIOC_S_FMT, &fmt)){
        printf("Setting Pixel Format\n");
        return 1;
    }
    return 0;
}

int UsbCapture::init_mmap()
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(mVideofd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(mVideofd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 2;
    }

    mBuffer = (uint8_t *)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, mVideofd, buf.m.offset);
    mBufferLen=buf.length;
   // printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    //printf("Image Length: %d\n", buf.bytesused);

    return 0;
}

int UsbCapture::refreshVideoFormat(){
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(mVideofd, VIDIOC_G_FMT, &fmt)) {
        perror("Querying Pixel Format\n");
        return 1;
    }
    mCaptureWidth = fmt.fmt.pix.width;
    mCaptureHeight = fmt.fmt.pix.height;
    mImgFormat = fmt.fmt.pix.pixelformat;
    return 0;
}

void UsbCapture::cvtRaw2Mat(const void * data, cv::Mat & image){
    if (mImgFormat == V4L2_PIX_FMT_MJPEG){
        Mat src(mCaptureHeight, mCaptureWidth, CV_8UC3, (void*) data);
        image = imdecode(src, 1);
    }
    else if(mImgFormat == V4L2_PIX_FMT_YUYV){
        Mat yuyv(mCaptureHeight, mCaptureWidth, CV_8UC2, (void*) data);
        cvtColor(yuyv, image, CV_YUV2BGR_YUYV);
    }
}

int UsbCapture::getImg(Mat &img){
    //struct timeval start,end;
    //gettimeofday(&start,NULL);
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(mVideofd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        //设备松动重连
        mIsOpen=false;
        close(mVideofd);
        init();
        return 2;
    }

    if(-1 == xioctl(mVideofd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 3;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(mVideofd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(mVideofd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return 4;
    }
    if(-1 == xioctl(mVideofd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return 5;
    }
    //gettimeofday(&end,NULL);

    //cout<<"Running time: "<<(end.tv_sec-start.tv_sec)*1000+((double)(end.tv_usec-start.tv_usec))/1000<<"ms"<<endl;

    cvtRaw2Mat((void*)mBuffer,img);
    return 0;
}

UsbCapture& UsbCapture::operator >> (Mat& img)
{
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(mVideofd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        //设备松动重连
        mIsOpen=false;
        close(mVideofd);
        init();
        return *this;
    }

    if(-1 == xioctl(mVideofd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return *this;
    }
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(mVideofd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(mVideofd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return *this;
    }

    if(-1 == xioctl(mVideofd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return *this;
    }
    cvtRaw2Mat((void*)mBuffer,img);

    return *this;

}



int UsbCapture::setSaturation(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_SATURATION;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Saturation error\n");
        return -1;
    }
    return 0;
}

int UsbCapture::setWhiteBalance(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set White Balance error\n");
        return -1;
    }
    return 0;
}
int UsbCapture::setBrightness(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_BRIGHTNESS;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Brightness error\n");
        return -1;
    }
    return 0;
}
int UsbCapture::setGamma(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_GAMMA;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Gamma error\n");
        return -1;
    }
    return 0;
}

int UsbCapture::setContrast(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_CONTRAST;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Contrast error\n");
        return -1;
    }
    return 0;
}


int UsbCapture::setExposureTime(int t){
    if(!mIsOpen){
        return 1;
    }


    struct v4l2_control control_s;
    control_s.id = V4L2_CID_EXPOSURE_AUTO;
    control_s.value = V4L2_EXPOSURE_MANUAL;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Close MANUAL Exposure error\n");
        return 3;
    }
    control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control_s.value = t;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Exposure Time error\n");
        return 1;
    }
    return 0;
}


int UsbCapture::changeVideoFormat(int width, int height, bool mjpeg){
    if(!mIsOpen){
        return 4;
    }
    mIsOpen=false;
    enum   v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(mVideofd, VIDIOC_STREAMOFF, &type) < 0){
        perror("VIDIOC_STREAMOFF");
        return 5;
    }
    close(mVideofd);
    munmap(mBuffer, mBufferLen);
    mCaptureWidth=width;
    mCaptureHeight=height;
    mIsMjpeg=mjpeg;
    return init();

}
int UsbCapture::infoPrint(){
    struct v4l2_capability caps = {};
    if (-1 == xioctl(mVideofd, VIDIOC_QUERYCAP, &caps))
    {
        perror("Querying Capabilities");
        return 1;
    }

    printf( "Driver Caps:\n"
                    "  Driver: \"%s\"\n"
                    "  Card: \"%s\"\n"
                    "  Bus: \"%s\"\n"
                    "  Version: %d.%d\n"
                    "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);

    struct v4l2_cropcap cropcap = {0};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl (mVideofd, VIDIOC_CROPCAP, &cropcap))
    {
        perror("Querying Cropping Capabilities");
        return 1;
    }
    printf( "Camera Cropping:\n"
                    "  Bounds: %dx%d+%d+%d\n"
                    "  Default: %dx%d+%d+%d\n"
                    "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    int support_grbg10 = 0;

    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(mVideofd, VIDIOC_ENUM_FMT, &fmtdesc))
    {
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
            support_grbg10 = 1;
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(mVideofd, VIDIOC_G_FMT, &fmt)) {
        perror("Querying Pixel Format\n");
        return 1;
    }
    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    printf( "Selected Camera Mode:\n"
                    "  Width: %d\n"
                    "  Height: %d\n"
                    "  PixFmt: %s\n"
                    "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);

    struct v4l2_streamparm streamparm = {0};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(mVideofd, VIDIOC_G_PARM, &streamparm)) {
        perror("Querying Frame Rate\n");
        return 1;
    }
    printf( "Frame Rate:  %f\n====================\n",
            (float)streamparm.parm.capture.timeperframe.denominator /
            (float)streamparm.parm.capture.timeperframe.numerator);

    struct v4l2_control control_g;

    //曝光时间
    control_g.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Exposure error\n");
    }else{
        printf("EXPOSURE:%d\n",control_g.value);
    }
    //色彩饱和度
    control_g.id = V4L2_CID_SATURATION ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Saturation error\n");
    }else{
        printf("Saturation:%d\n",control_g.value);
    }
    //对比度
    control_g.id = V4L2_CID_CONTRAST ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Contrast error\n");
    }else{
        printf("Contrast:%d\n",control_g.value);
    }
    //增益
    control_g.id = V4L2_CID_GAIN;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get GAIN error\n");
    }else{
        printf("GAIN:%d\n",control_g.value);
    }
    //伽玛值
    control_g.id = V4L2_CID_GAMMA;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Gamma error\n");
    }else{
        printf("Gamma:%d\n",control_g.value);
    }
    //亮度
    control_g.id = V4L2_CID_BRIGHTNESS;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Brightness error\n");
    }else{
        printf("Brightness:%d\n",control_g.value);
    }
    //白平衡
    control_g.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get White Balance error\n");
    }else{
        printf("White Balance:%d\n",control_g.value);
    }
    //色调
    control_g.id =V4L2_CID_HUE ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get HUE error\n");
    }else{
        printf("HUE:%d\n",control_g.value);
    }
    //逆光补偿
    control_g.id =V4L2_CID_BACKLIGHT_COMPENSATION ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get BACKLIGHT_COMPENSATION error\n");
    }else{
        printf("BACKLIGHT_COMPENSATION:%d\n",control_g.value);
    }
    //清晰度
    control_g.id =V4L2_CID_SHARPNESS ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get SHARPNESS error\n");
    }else{
        printf("SHARPNESS:%d\n",control_g.value);
    }

    return 0;

}
