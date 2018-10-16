#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <time.h>

const String fileName = "/home/wyx/图片/1/my_photo-55.jpg";

const float AutoAim::max_offset_angle = 30;

AutoAim::AutoAim(){}

AutoAim::~AutoAim(){}

//排序得到的椭圆，使得角度近似、高度近似的点邻近
bool cmp(RotatedRect &x, RotatedRect &y){
   if(x.angle!=y.angle) return x.angle>y.angle;
   if(x.size.height!=y.size.height) return x.size.height<y.size.height;
   return x.size.width>y.size.width;
}

void AutoAim::setImage(Mat &img, Mat &mask){
    clock_t start, finish;
    start = clock();
    Mat channel[3];

    GaussianBlur(img,img,Size(5,5),0,0);
    split(img,channel);

    //黑帽运算，分离比邻近点暗的点，Size的参数待调
    //morphologyEx(channel[0]-channel[2], mask, MORPH_BLACKHAT, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21), cv::Point(-1, -1)));

    //开运算消除小物块，平滑物体的边界
    //morphologyEx(mask, mask, MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1, -1)));
    threshold(channel[0]-channel[2], mask, 0, 255, THRESH_BINARY+THRESH_OTSU); //自适应阈值
    Canny(mask, mask, 3, 9, 3);
    imshow("mask", mask);
    finish = clock();
    cout<<"time: "<<(double)(finish-start)/CLOCKS_PER_SEC<<endl;
}

void AutoAim::findLamp(Mat &src, Mat &mask, vector<RotatedRect> &lamps){
    imshow("mask", mask);
    lamps.clear();
    vector<vector<Point>> contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    RotatedRect temp;
    vector<RotatedRect> pre_lamps;
    cout<<"contours.size: "<<contours.size()<<endl;
    for(int i=0;i<contours.size();i++){
        if(contours[i].size()>5){
            temp = minAreaRect(contours[i]);
            cout<<"temp angle: "<<temp.angle<<endl;
            ellipse(src, temp, Scalar(0, 0, 0), 1, 8);
            float theta=temp.angle;
            pre_lamps.push_back(temp);
            if(abs(90-theta)<30||abs(theta)<30){

                //判断是否放入的条件 
                //if(max(temp.size.width, temp.size.height) < min(temp.size.width, temp.size.height)*2)
			        //continue;
                //if(temp.size.width>temp.size.height)
                    //continue;
            
                //ellipse(src, temp.center, Size(temp.size.width/2, temp.size.height/2), temp.angle, 0, 360, Scalar(0, 255, 0), 1, 8);
                //putText(img, to_string(i), temp.center, FONT_HERSHEY_SIMPLEX,1, Scalar(255,23,0), 2, 8);
                ellipse(src, temp, Scalar(0, 255, 0), 3, 8);
            }
        }
}

    //排序lamp找到满足比例条件的灯管
    sort(pre_lamps.begin(), pre_lamps.end(), cmp);
    
    //角度和高度的权重，角度更加重要，所以角度的偏差使得结果的值偏差更大
    int angle_diff_weight = 2;
    int height_diff_weight = 1;

    //按照比例　两灯管中心点之间的距离：两灯管长度的平均 近似等于 2:1
    //这里只跟右边三个进行比较 = clock();
    int size = pre_lamps.size();    Mat channel[3];
    float *diff = new float(size);
    float *best_match_index = new float(size);
    for(int i=0; i<size; i++){
        diff[i] = 0x3f3f3f3f;
        best_match_index[i] = -1;

        cout<<"angle: "<<pre_lamps[i].angle;
        cout<<" length: "<<pre_lamps[i].size.height<<endl;
    }
    
    //中心点间距离，平均高度，角度差，高度差
    float dist, avg_height, diff_angle, diff_height, ratio, totalDiff;
    int i,j;
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f3f3f;
        int currIndex = -1;
        for(j=1; j<=2 && (i+j)<size; j++){
            //计算比例，筛选灯管
            const RotatedRect &compare = pre_lamps[i+j];
            const RotatedRect &current = pre_lamps[i];

            diff_angle = abs(compare.angle - current.angle);
            if(diff_angle > 10) continue;
            diff_height = abs(compare.size.height - current.size.height);
            if(diff_height > 20) continue;
            
            dist = abs(compare.center.x - current.center.x) + abs(compare.center.y - current.center.y);
            avg_height = (compare.size.height + current.size.height) / 2;
            ratio = dist / avg_height;
            cout<<"ratio: "<<ratio<<endl;
            if(ratio < 1.5 || ratio > 10) continue;
            
            totalDiff = angle_diff_weight*diff_angle + height_diff_weight*diff_height;
            if(totalDiff < currDiff){
                currDiff = totalDiff;
                currIndex = i+j;
            }
        }
        //一对灯管肯定花费是最少的，所以如果当前花费比两个的花费都要少，就记录是最优
        if(currIndex==-1) continue;
        if(currDiff < diff[i] && currDiff < diff[currIndex]){
            diff[i] = currDiff;
            diff[currIndex] = currDiff;
            best_match_index[i] = currIndex;
            best_match_index[currIndex] = i;
        }
    }

    //遍历，将满足条件的灯管储存
    for(i=0; i<size; i++){
        int index = best_match_index[i];
        cout<<"best_match_index: "<<index;
        cout<<" diff: "<<diff[i]<<endl;
        if(index == -1 || index <= i) continue;
        //找到匹配对
        if(i == best_match_index[index]){
            lamps.push_back(pre_lamps[i]);
            lamps.push_back(pre_lamps[index]);
        }
    }
    cout<<"lamps counts: "<<(lamps.size()/2)<<endl;
}

int main(int argc, char const *argv[]){
    Mat src = imread(fileName);
    AutoAim autoAim;
    Mat mask;
    autoAim.setImage(src, mask);
    vector<RotatedRect> lamps;
    autoAim.findLamp(src, mask, lamps);
    for(int i=0; i<lamps.size(); i++){
        circle(src, lamps[i].center, 20, Scalar(0,0,255), 3);
    }
    imshow("src", src);
    waitKey(0);
    return 0;
}
