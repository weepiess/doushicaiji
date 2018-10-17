#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>
#include "time.h"
const String fileName = "../res/my_photo-190.jpg";
const Scalar Bluelow(100,43,46);
const Scalar Bluehigh(124,255,255);
const float AutoAim::max_offset_angle = 30;


AutoAim::AutoAim(){}

AutoAim::~AutoAim(){}


//排序得到的椭圆，使得角度近似、高度近似的点邻近
bool cmp(RotatedRect &x, RotatedRect &y){
    float theta_x=x.angle;
    float theta_y=y.angle;
    if(x.angle>95)
        theta_x=abs(180-theta_x);
    if(y.angle>95)
        theta_y=abs(180-theta_y);
   if(theta_x!=theta_y) return theta_x>theta_y;
   if(x.size.height!=y.size.height) return x.size.height<y.size.height;
   return x.size.width>y.size.width;
}


//两点距离
float distPoint(Point2f center1, Point2f center2){
    return abs(center1.x-center2.x);//sqrt((center1.x-center2.x)*(center1.x-center2.x) + (center1.y-center2.y)*(center1.y-center2.y));
}

bool opendetect(float &max, Mat &mid)
{   
    if(max>20)
    {
        morphologyEx(mid, mid, MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9), cv::Point(-1, -1))); //开运算消除小物块，平滑物体的边界
        Canny(mid, mid, 3, 9, 3);
        return false;
    }
    else
    {
        morphologyEx(mid, mid, MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1, -1))); //开运算消除小物块，平滑物体的边界
        Canny(mid, mid, 3, 9, 3);
        return false;
    }
}


void AutoAim::setImage(Mat &img, Mat &mask){
    Mat hsv;
    //Mat mid;
    Mat channel[3];
    GaussianBlur(img,img,Size(5,5),0,0);
    split(img,channel); 
    //dilate(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5),cv::Point(-1, -1)));
    threshold(channel[2]-channel[0], mask, 0, 255, THRESH_BINARY+THRESH_OTSU); //自适应阈值
    //morphologyEx(mask, mask, MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7), cv::Point(-1, -1))); //开运算消除小物块，平滑物体的边界
    Canny(mask, mask, 3, 9, 3);
    imshow("mask", mask);
}
void AutoAim::findLamp(Mat &mask, vector<RotatedRect> &lamps, Mat &src){
    
    lamps.clear();
    float lastCenterX = 0, lastCenterY = 0;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    float maxtemp=0;
    float mintemp=15;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    RotatedRect temp;
    vector<RotatedRect> pre_lamps;
    
    cout<<"contours.size: "<<contours.size()<<endl;
    if(contours.size()<30){
    for(int i=0;i<contours.size();i++){
        // if(fabs(lastCenterX - temp.center.x) + fabs(lastCenterY - temp.center.y) > 10){
        //     //ellipse(src, temp.center, Size(temp.size.width/2, temp.size.height/2), temp.angle, 0, 360, Scalar(0, 0, 255), 3, 8);
        if(i%2==0)
        {
            if(contours[i].size()>5){
                temp =fitEllipseAMS(Mat(contours[i]));
            
                float theta=temp.angle;
                 if(theta>90)
                 {
                    if((180-theta)<0)
                    {
                        temp.angle=-(180-theta);
                        theta=-(180-theta);
                    }
                    else
                    {
                        temp.angle=180-theta;
                        theta=180-theta;
                    }
                 }
                     
               

                if(abs(theta)<40){
                    
                    
                    
                //判断是否放入的条件 
                    // if(max(temp.size.width, temp.size.height) < min(temp.size.width, temp.size.height)*1.2)
			        //     continue;
                    if(temp.size.width > (temp.size.height)*1.2)
			            continue;
                    //if(temp.size.width<5||temp.size.height<5)
                        //continue;                    
                    ellipse(src, temp.center, Size(temp.size.width/2, temp.size.height/2), temp.angle, 0, 360, Scalar(0, 0, 255), 1, 8);
                    
                    //ellipse(src, temp.center, Size(temp.size.width/2, temp.size.height/2), temp.angle, 0, 360, Scalar(0, 255, 0), 1, 8);
                    putText(src, to_string(i), temp.center, FONT_HERSHEY_SIMPLEX,1, Scalar(255,23,0), 2, 8);
                    //cout<<i<<" : "<<temp.size.width<<" "<<temp.size.height<<" "<<temp.angle<<endl;
                    //ellipse(src, temp, Scalar(255, 0, 0), 2, 8);
                    pre_lamps.push_back(temp);
                    if(temp.size.height>maxtemp)
                        maxtemp=temp.size.height;
                    if(temp.size.height<mintemp)
                        mintemp=temp.size.height;
                
                }
                // lastCenterX = temp.center.x;
                //         lastCenterY = temp.center.y;
            }

            
        } 
    }
             
    }


    //排序lamp找到满足比例条件的灯管
    sort(pre_lamps.begin(), pre_lamps.end(), cmp);
    
    //角度和高度的权重，角度更加重要，所以角度的偏差使得结果的值偏差更大
    int angle_diff_weight = 3;
    int height_diff_weight = 1;

    //按照比例　两灯管中心点之间的距离：两灯管长度的平均 近似等于 2:1
    //这里只跟右边三个进行比较
    int size = pre_lamps.size();
    vector<float> diff(size,0);
    //float *diff = new float(size);
    vector<float> best_match_index(size,0);
    //float *best_match_index = new float(size);
    for(int i=0; i<size; i++){
        diff[i] = 0x3f3f3f3f;
        best_match_index[i] = -1;

        cout<<"angle: "<<pre_lamps[i].angle<<endl;
        //cout<<" length: "<<pre_lamps[i].size.height<<endl;
        //cout<<"x: "<<pre_lamps[i].center.x<<endl;
    }
    
    //中心点间距离，平均高度，角度差，高度差
    float dist, avg_height, diff_angle, diff_height, ratio, totalDiff;
    int i,j;
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f3f3f;
        int currIndex = -1;
        const RotatedRect &current = pre_lamps[i];
        float theta_current=abs(current.angle);
        if(theta_current>95)
                    theta_current=abs(180-theta_current);
        for(j=1; j<=4 || (i+j)<size; j++){
            //计算比例，筛选灯管
            
            const RotatedRect &compare = pre_lamps[i+j];
            
            float theta_compare=abs(compare.angle);
            if(theta_compare>95)
                    theta_compare=abs(180-theta_compare);
            diff_angle = abs(theta_compare - theta_current);
            
            if(diff_angle > 15) continue;
            diff_height = abs(compare.size.height - current.size.height);
            if(diff_height>30) continue;
            dist = distPoint(compare.center, current.center);
           
            //cout<<"distt: "<<dist<<endl;
            if(dist>10*current.size.height) continue;
            avg_height = (compare.size.height + current.size.height) / 2;
            ratio = dist / avg_height;
            //cout<<"ratio: "<<ratio<<endl;
            if(ratio > 10) continue;
            
            totalDiff = angle_diff_weight*diff_angle + height_diff_weight*diff_height+0.5*dist;
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
    cout<<"lamps counts: "<<lamps.size()/2<<endl;
}

void AutoAim::decide(Mat &src,vector<RotatedRect> &dec, vector<RotatedRect> &memory)
{
    int counts=0;
    int quit=0;
    float mid=0;
    float best=0;
    float bestscore=10;
    int bestcount=0;
    float temp=0;

    if(dec.size()==0) return;


    // for(int i=0;i<dec.size();i++)
    // {
    //     cout<<dec[i].angle<<"   angle    "<<endl;
    // }


    for(int i=0;i<dec.size()-1;i++)
    {
        cout<<i<<"   i    "<<endl;
        mid=abs(dec[i+1].center.x-dec[i].center.x);
        best=dec[i].center.x-memory[i].center.x;
        if(mid>temp)
        {
            counts=i;                       
        
        if(bestscore>best)
        {
            bestscore=best;
            bestcount=i;
        }
        }
        if((dec[counts+1].angle-dec[counts].angle)<10)
        {
            quit=i;
            continue;
        }
    }
    // cout<<counts<<endl;
    // cout<<dec[0].center.x<<"attention"<<endl;
    // cout<<dec[1].center.x<<"attention"<<endl;
    // cout<<dec.size()<<"attention"<<endl;
    if(bestcount!=counts)
        counts=bestcount;
    if(quit!=dec.size()-1)
    {
    Point p;
    p.x=(dec[counts].center.x+dec[counts+1].center.x)/2;
    p.y=(dec[counts].center.y+dec[counts+1].center.y)/2;
    circle(src,p, 10, Scalar(120, 0, 120), - 1);
    memory.push_back(dec[counts]);   
    }
}

int main(int argc, char const *argv[]){
    // double fps;
    // char string[10];  // 用于存放帧率的字符串
    // double t = 0;

    // // Mat src = imread(fileName);
     Mat src;
    // Mat mask;
    // // clock_t start,finish;
    // // double time_tol;
    // AutoAim autoAim;
    int i=0;
    AutoAim autoAim;
    Mat mask;
    vector<RotatedRect> memory;
    vector<RotatedRect> lamps;
    memory.clear();
    VideoCapture capture("/dev/video1");
    if(!capture.isOpened())
        cout<<"failed"<<endl;
    double rate=capture.get(CV_CAP_PROP_FPS);
    int delay =1000/rate;
    Mat frame;
    while(1)
    {   
        
        
        if(!capture.read(frame))
        {   
            break;
        }
        else
        {
            waitKey(delay);
        //     Mat srcImage,outImage;
        //     capture>>srcImage;
        //     threshold(srcImage);
        //     imshow("",frame);
        // }

        //Mat src;
        capture>>src;

        //Mat mask;
        memory.clear();
        autoAim.setImage(src, mask);

     
    
        autoAim.findLamp(mask, lamps,src);
        autoAim.decide(src,lamps,memory);
        
        // for(int i=0;i<lamps.size();i++)
        // {
        //     cout<<i<<" "<<lamps[i].center.x<<endl;
        //     circle(src,lamps[i].center,20,(255,255,0),2);
        // }
         imshow("src",src);
        }
        
    }
}




//     AutoAim autoAim;
//     Mat mask;
//     vector<RotatedRect> memory;
//     memory.clear();
//     Mat src = imread(fileName);

//     autoAim.setImage(src, mask);
     
//     vector<RotatedRect> lamps;
    
    
//     autoAim.findLamp(mask, lamps,src);
//     cout<<lamps.size()<<endl;
//     autoAim.decide(src,lamps,memory);
//     for(int i=0;i<lamps.size();i++)
//     {
//         cout<<i<<" "<<lamps[i].center.x<<endl;
//         circle(src,lamps[i].center,20,(255,255,255),2);
//     }
//     imshow("src",src);
//     waitKey(0);
//     return 0;
// }
