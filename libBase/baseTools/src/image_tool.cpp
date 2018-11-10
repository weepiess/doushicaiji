////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Tool Code for robot
///ALL RIGHTS RESERVED
///@file:image_tool.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-3
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "image_tool.h"

using namespace std;
using namespace cv;

//绘制旋转矩形
void ImageTool::drawRotatedRect(cv::Mat &img,cv::RotatedRect r){
    RNG rng(12345);
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    Point2f rect_points[4];
    r.points( rect_points );
    for( int j = 0; j < 4; j++ )
        line( img, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
}
//绘制四边形
void ImageTool::draw4Point4f(cv::Mat &img, cv::Point2f *point2fs) {
    RNG rng(12345);
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for( int j = 0; j < 4; j++ )
        line( img, point2fs[j], point2fs[(j+1)%4], color, 1, 8 );
}

float ImageTool::calc2PointDistance(cv::Point2f point1, cv::Point2f point2) {
    return sqrt(((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)));
}

float ImageTool::calc2PointApproDistance(cv::Point2f point1, cv::Point2f point2){
    return abs(point1.x - point2.x) + abs(point1.y - point2.y);
}

float ImageTool::calc2PointAngle(cv::Point2f point1, cv::Point2f point2) {
    if(point1.x == point2.x){
        return 90;
    } else{
        double k;
        k= -(point1.y - point2.y) / (point1.x - point2.x);//符号取反，图像坐标系和实际坐标系不统一
        return  (float)(atan(k)*180/CV_PI);
    }

}

float ImageTool::calcTriangleInnerAngle(cv::Point2f vertexPoint, cv::Point2f point1, cv::Point2f point2) {
    float a,b,c;//求角C
    float angleC;
    a=calc2PointDistance(vertexPoint,point1);
    b=calc2PointDistance(vertexPoint,point2);
    c=calc2PointDistance(point1,point2);
    angleC=static_cast<float>(acos((a * a + b * b - c * c) / (2 * a * b)) / CV_PI * 180);
    return angleC;

}

double ImageTool::gravityKiller(double z_distance, double y_distance, double bullet_speed, double current_pitch){
    constexpr double GRAVITY = 9.7913;
    double alpha = current_pitch*CV_PI/180;
    double x = -y_distance*sin(alpha)+z_distance*cos(alpha);
    double y = z_distance*sin(alpha)+y_distance*cos(alpha);
    double v = bullet_speed;
    double m = 2*((y*GRAVITY+v*v)-sqrt((y*GRAVITY+v*v)*(y*GRAVITY+v*v)-GRAVITY*GRAVITY*(y_distance*y_distance+z_distance*z_distance)))/(GRAVITY*GRAVITY);
    double time = sqrt(m);
    double beta = asin((2*y-GRAVITY*time*time)/(2*v*time))*180/CV_PI;
    double offset = beta-current_pitch;
    return offset;
}

struct timeval ImageTool::start={0};
struct timeval ImageTool::end={0};
clock_t ImageTool::currentClock=clock();

void ImageTool::timeInit() {
    gettimeofday(&start,NULL);
}

void ImageTool::timeCout() {
    gettimeofday(&end,NULL);
    cout<<"Running time: "<<(end.tv_sec-start.tv_sec)*1000+((double)(end.tv_usec-start.tv_usec))/1000<<"ms"<<endl;
}

double ImageTool::timeCount(){
    gettimeofday(&end, NULL);
    return (end.tv_sec-start.tv_sec)*1000+((double)(end.tv_usec-start.tv_usec))/1000;
}

double ImageTool::timeGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (tv.tv_sec*1000-tv.tv_usec/1000);
}

void ImageTool::timeInitClock() {
    currentClock=clock();
}

void ImageTool::timeCoutClock() {
    clock_t ck=clock();
    cout<<"Running time(clock): "<<(double)(ck-currentClock)/CLOCKS_PER_SEC*1000<<endl;
}

void ImageTool::showRGBSplitImg(cv::Mat img){
    Mat bgr[3];
    Mat img_b,img_g,img_r;
    split(img, bgr); //将三个通道的像素值分离
    img_b = bgr[0];
    img_g = bgr[1];
    img_r = bgr[2];
    imshow("img",img);
    imshow("b",img_b);
    imshow("g",img_g);
    imshow("r",img_r);

    //r阈值处理
    Mat imgR_th;
    threshold( img_r, imgR_th,123, 255,CV_THRESH_BINARY );
    imshow("r_th",imgR_th);

    //r-b处理
    Mat imgRsB;
    imgRsB=(img_r-img_b)*10;
    imshow("imgR-B",imgRsB);
    //r-g处理
    Mat imgRsG;
    imgRsG=(img_r-img_g)*10;
    imshow("imgR-G",imgRsG);
    waitKey(0);

}

void ImageTool::showHSVInRangeImg(cv::Mat img) {
    Mat imgHSV;
    cvtColor(img, imgHSV, CV_BGR2HSV);

    int iLowH = 25;
    int iHighH = 67;
    int iLowS = 42;
    int iHighS = 180;

    int iLowV = 0;
    int iHighV = 255;
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    //
    Mat imgThre;
    while(1){
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThre);
        imshow("hsv",imgHSV);
        imshow("hsvInRange",imgThre);
        waitKey(1);
    }

    waitKey(0);
}