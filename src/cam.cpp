#include <stdlib.h>
#include <iostream>
#include <string>
#include "OpenNI.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;
using namespace openni;

void CheckOpenNIError( Status result, string status )
{
    if( result != STATUS_OK )
        cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}
int main( int argc, char** argv )
{
    Status result = STATUS_OK;
    //OpenNI2 image
    VideoFrameRef oniDepthImg;
    VideoFrameRef oniColorImg;
    //OpenCV image
    Mat cvDepthImg;
    Mat cvBGRImg;
    Mat cvFusionImg;
    namedWindow("depth",WINDOW_NORMAL);
    namedWindow("image",WINDOW_NORMAL);
    //namedWindow("fusion",WINDOW_NORMAL);
    char key=0;
    result = OpenNI::initialize();
    CheckOpenNIError( result, "initialize context" );
    //打开device
    Device device;
    result = device.open( openni::ANY_DEVICE );
    // create depth stream
    VideoStream oniDepthStream;
    result = oniDepthStream.create( device, openni::SENSOR_DEPTH );
    // set depth video mode
    VideoMode modeDepth;
    modeDepth.setResolution( 1280, 720 );
    modeDepth.setFps( 60 );
    modeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
    oniDepthStream.setVideoMode(modeDepth);
    // start depth stream
    result = oniDepthStream.start();
    // create color stream
    VideoStream oniColorStream;
    result = oniColorStream.create( device, openni::SENSOR_COLOR );
    // set color video mode
    VideoMode modeColor;
    modeColor.setResolution( 1280, 720 );
    modeColor.setFps( 60 );
    modeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );
    oniColorStream.setVideoMode( modeColor);
    // set depth and color imge registration mode
    if( device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }
    // start color stream
    result = oniColorStream.start();
    while( key!=27 )
    {
        if( oniColorStream.readFrame( &oniColorImg ) == STATUS_OK )
        {
            // convert data into OpenCV type
            cv::Mat cvRGBImg( oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData() );
            cvtColor( cvRGBImg, cvBGRImg, CV_RGB2BGR );
            imshow( "image", cvBGRImg );
        }
        if( oniDepthStream.readFrame( &oniDepthImg ) == STATUS_OK )
        {
            cv::Mat cvRawImg16U( oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData() );
            cvRawImg16U.convertTo( cvDepthImg, CV_8U, 255.0/(oniDepthStream.getMaxPixelValue()));
            // convert depth image GRAY to BGR
            cvtColor(cvDepthImg,cvFusionImg,CV_GRAY2BGR);
            imshow( "depth", cvDepthImg );
        }
        //addWeighted(cvBGRImg,0.5,cvFusionImg,0.5,0,cvFusionImg);
        //imshow("fusion",cvFusionImg);
        key = waitKey(20);
    }
    destroyWindow("depth");
    destroyWindow("image");
    oniDepthStream.destroy();
    oniColorStream.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
