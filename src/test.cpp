#include <iostream>
#include "OpenNI.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "cam.h"
//#include "serial_interface.h"
using namespace openni;
using namespace std;
using namespace cv;

int main(int agrc, char *argv[]){
   
    /*openni::Device device;
    Status result = OpenNI::initialize();
    cout<<result<<endl;
    result = device.open( openni::ANY_DEVICE );
    cout<<result<<endl;
    cam Cam;
    Cam.Camera_Init(device);*/

   //cam Cam;
	Mat i_before(480, 640, CV_8UC4);				
	Mat i_after(480, 640, CV_8UC4);				
	Mat i_result(480, 640, CV_16UC1);				
    Mat src;
    Mat src2;
    Status result = STATUS_OK;
    VideoFrameRef oniDepthImg;
    VideoFrameRef oniColorImg;
    VideoFrameRef  frameDepth;
    Mat results(480, 640,CV_8U);
    Mat cvDepthImg;
    Mat cvBGRImg;
    Mat cvFusionImg;
    char key=0;
    int count=0;
    result = OpenNI::initialize();
    Array<DeviceInfo> aDeviceList;  
    OpenNI::enumerateDevices( &aDeviceList );   
    
    cout << "         " << aDeviceList.getSize() << endl;  
    
    for( int i = 0; i < aDeviceList.getSize(); ++ i )  
    {    
        cout << "设备 " << i << endl;    
        const DeviceInfo& rDevInfo = aDeviceList[i];      
        cout << "URI: " << rDevInfo.getUri() << endl;     

    }
    //CheckOpenNIError( result, "initialize context" );

    Device device;
    result = device.open( openni::ANY_DEVICE );
    cout<<result<<endl;

    return 0;
}
