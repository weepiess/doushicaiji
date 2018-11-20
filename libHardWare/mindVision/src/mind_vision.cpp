#include "mind_vision.h"

#define CAMERA_PARAMS_ADJUST

MindVision::MindVision(){}

MindVision::~MindVision(){
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
}

int MindVision::init(){
    CameraSdkInit(1);

    int index = 1;
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

    #ifdef DEBUG
	printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);  
    #endif

    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&index);

    //初始化失败
    #ifdef DEBUG
	printf("state = %d\n", iStatus);
    #endif

    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }

    return index;
}

int MindVision::startPlay(int hCamera_, unsigned char color){
    hCamera = hCamera_;

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    cv::FileStorage f;
    if(color == 0){ //红色
        f = FileStorage("../libHardWare/mindVision/res/light_red_config.yaml", cv::FileStorage::READ);
    } else {
        f = FileStorage("../libHardWare/mindVision/res/light_blue_config.yaml", cv::FileStorage::READ);
    }
    int width, height, frameSpeed, gamma, exposureTime, contrast, aeState;
    f["frame_speed"] >> frameSpeed;
    f["gamma"] >> gamma;
    f["exposure_time"] >> exposureTime;
    f["contrast"] >> contrast;
    f["ae_state"] >> aeState;
    f["resolution_width"] >> width;
    f["resolution_height"] >> height;
    f.release();

    #ifdef CAMERA_PARAMS_ADJUST
    this->index = frameSpeed;
    this->gamma = gamma;
    this->exposureTime = exposureTime;
    this->contrast = contrast;
    this->aeState = aeState;
    this->resolution_width = width;
    this->resolution_height = height; 
    #endif

    //设置分辨率，默认是1280*720
    setResolution(width, height);

    #ifdef DEBUG
    CameraGetImageResolution(hCamera, &tResolution);
    printf("%d %d", tResolution.iWidth, tResolution.iHeight);
    #endif

    //帧率
    CameraSetFrameSpeed(hCamera, frameSpeed); //1
    //gamma值
    CameraSetGamma(hCamera, gamma); //20
    //曝光时间
    CameraSetExposureTime(hCamera, exposureTime); //1000
    //对比度
    CameraSetContrast(hCamera, contrast); //150
    //是否自动曝光
    CameraSetAeState(hCamera, aeState==0?FALSE:TRUE); //FALSE
    //白平衡
    //CameraSetWbMode(hCamera, TRUE);
    //色温
    //CameraSetPresetClrTemp(hCamera, 0);
    //色温增益
    //CameraSetGain(hCamera, 1, 0, 0);

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    return 0;
}

int MindVision::setResolution(int width, int height){
    tResolution.iIndex = 0xFF;
    tResolution.iWidth = width;
    tResolution.iHeight = height;
    CameraSetImageResolution(hCamera, &tResolution);
}

int MindVision::getImg(Mat &src){
    //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
	//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
	CameraReleaseImageBuffer(hCamera,pbyBuffer);

    if(CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS){
		CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
		if (iplImage){
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
        cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        src = cv::cvarrToMat(iplImage);//这里只是进行指针转换，将IplImage转换成Mat类型
        return 0;
	}
    return -1;
}

void MindVision::adjustParams(unsigned char c){
    if(c=='f' || c=='g' || c=='c' || c=='a' || c=='e'){
        currOperation = c;
        return;
    }
    if(!(c == 'n' || c == 'm' || c == 's')) return;
    cout<<c<<endl;
    //保存参数
    if(c=='s'){
        cv::FileStorage f("../libHardWare/mindVision/res/light_red_config.yaml", cv::FileStorage::WRITE);
        f << "frame_speed" << index;
        f << "gamma" << gamma;
        f << "exposure_time" << exposureTime;
        f << "contrast" << contrast;
        f << "ae_state" << aeState;
        f << "resolution_width" << resolution_width;
        f << "resolution_height" << resolution_height;
        f.release();
        return;
    }
    int status = -1;
    switch(currOperation){
        case 'f':{
            if(c == 'n' && index < 2) ++index;
            if(c == 'm' && index > 0) --index;
            CameraSetFrameSpeed(hCamera, index);
            break;
        }
        case 'g':{
            if(c == 'n') gamma+=2;
            if(c == 'm' && gamma>=2) gamma-=2;
            CameraSetGamma(hCamera, gamma);
            break;
        }
        case 'c':{
            if(c == 'n')  contrast+=2;
            if(c == 'm' && contrast>=2) contrast-=2;
            CameraSetContrast(hCamera, contrast);
            break;
        }
        case 'a':{
            if(c == 'n') aeState = 1;
            if(c == 'm') aeState = 0;
            CameraSetAeState(hCamera, aeState);
            break;
        }
        case 'e':{
            if(c == 'n') exposureTime+=100;
            if(c == 'm' && exposureTime>=100) exposureTime-=100;
            CameraSetExposureTime(hCamera, exposureTime);
            break;
        }
    }
}