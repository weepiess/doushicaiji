#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

//const Scalar lowb = Scalar(100, 150, 100);
//const Scalar upperb = Scalar(124, 255, 255);

const char* WIN_DST = "win_dst";
const String fileName = "/home/wyx/图片/pic/my_photo-1.jpg";

//int min_light_gray = 210;
//int br_threshold = 110;

/*
void threshold_src(Mat &src, Mat &green_channels, Mat &enemy_color_img){
    int total_pixels = src.rows * src.cols;
    const uchar *ptr_src = src.data;
    const uchar *ptr_src_end = src.data + total_pixels*3;

    green_channels.create(src.size(), CV_8UC1);
    enemy_color_img = Mat(src.size(), CV_8UC1, Scalar(0));
    uchar *ptr_g = green_channels.data, *ptr_enemy_color = enemy_color_img.data;

    for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_enemy_color){
        uchar b = *ptr_src;
        uchar g = *(++ptr_src);
        uchar r = *(++ptr_src);
        *(ptr_g) = g;
        if(b-r > br_threshold){
            *(ptr_enemy_color) = 255;
        }
    }
}
*/

/*
int main(int agrc, char **argv){
    Mat src = imread(fileName);
    Mat blur_src, dst, hsv_src, thre_img;
    GaussianBlur(src, blur_src, Size(3,3), 2, 2);
    
    //Mat green_channels, enemy_color_img;
    //threshold_src(blur_src, green_channels, enemy_color_img);

    //HSV颜色空间，效果还行，对远处的线识别的较好，但是识别的物体不太好处理
    //cvtColor(blur_src, hsv_src, COLOR_BGR2HSV);
    //inRange(hsv_src, lowb, upperb, dst);

    //RGB颜色空间，可以识别出两条明显的线，但是有杂点，element的Size需要根据远近调节
    //近处需要Size较大，远处需要Size较小
    vector<Mat> channels;
    split(blur_src, channels);
    Mat blue_channel = channels[0];
    Mat red_channel = channels[2];
    Mat diff = blue_channel - red_channel;

    Mat blackHat_img;
    Mat element = getStructuringElement(MORPH_RECT, Size(12,12));
    morphologyEx(diff, blackHat_img, MORPH_BLACKHAT, element);

    //自适应阈值化，可以识别出灯条，但是杂点很多，需要调后面两个参数
    adaptiveThreshold(blackHat_img, thre_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 9, 5);
    //Mat element1 = getStructuringElement(MORPH_RECT, Size(3,3));
    //morphologyEx(thre_img, dst, MORPH_CLOSE, element1);

    namedWindow(WIN_DST, WINDOW_NORMAL);
    imshow(WIN_DST, thre_img);
    waitKey(0);
    return 0;
}*/