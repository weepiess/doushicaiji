#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>

const String fileName = "/home/wyx/图片/pic-final/my_photo-199.jpg";
const float AutoAim::max_offset_angle = 30;

AutoAim::AutoAim(int width, int height){
    this->IMG_WIDTH = width;
    this->IMG_HEIGHT = height;
    hasROI = false;
    resizeCount = 0;
}

AutoAim::~AutoAim(){}

//排序得到的椭圆，使得角度近似、高度近似的点邻近
bool cmp(RotatedRect &x, RotatedRect &y){
    float theta_x=x.angle;
    float theta_y=y.angle;
    if(x.angle>95)
        theta_x=-(180-theta_x);
    if(y.angle>95)
        theta_y=-(180-theta_y);
   if(theta_x!=theta_y) return theta_x>theta_y;
   if(x.size.height!=y.size.height) return x.size.height<y.size.height;
   return x.size.width>y.size.width;
}

//两点距离
float distPoint(Point2f center1, Point2f center2){
    return abs(center1.x-center2.x) + abs(center1.y-center2.y);
}

void AutoAim::setImage(Mat &img, Mat &mask, Color enemyColor){
    Mat channel[3];
    if(hasROI){
        mask = img(Rect(rectROI));
        GaussianBlur(mask, mask, Size(5,5), 0, 0);
    } else {
        GaussianBlur(img, mask, Size(5,5), 0, 0);
    }
    split(mask,channel); 
    threshold(enemyColor==red ? (channel[2]-channel[0]) : (channel[0] - channel[2]), mask, 0, 255, THRESH_BINARY+THRESH_OTSU); //自适应阈值
    Canny(mask, mask, 3, 9, 3);
    //imshow("mask", mask);                                             
}

void AutoAim::findLamp(Mat &mask, vector<RotatedRect> &lamps){
    
    lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    RotatedRect temp;
    vector<RotatedRect> pre_lamps;
    
    //cout<<"contours.size: "<<contours.size()<<endl;
    float lastCenterX = 0, lastCenterY = 0;
    if(contours.size()<40){
        for(int i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp =fitEllipse(Mat(contours[i]));

                //去掉重复拟合同一幅图片的情况
                if(fabs(lastCenterX - temp.center.x) + fabs(lastCenterY - temp.center.y) > 10){

                    float theta=temp.angle;
                    if(theta>95)
                        theta=abs(180-theta);

                    if(theta<40){
                        //判断是否放入的条件 
                        if(max(temp.size.width, temp.size.height) < min(temp.size.width, temp.size.height)*1.2)
			                continue;
                        //if(temp.size.width<8||temp.size.height<10)
                            //continue;
                        pre_lamps.push_back(temp);
                    }
                }

                lastCenterX = temp.center.x;
                lastCenterY = temp.center.y;
            }
        }
    }

    //排序lamp找到满足比例条件的灯管
    sort(pre_lamps.begin(), pre_lamps.end(), cmp);

    //for(int i=0; i<pre_lamps.size(); i++){
    //    cout<<i<<" "<<pre_lamps[i].center.x<<" "<<pre_lamps[i].center.y<<endl;
    //}
    
    //角度和高度的权重，角度更加重要，所以角度的偏差使得结果的值偏差更大
    int angle_diff_weight = 2;
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

        //cout<<"angle: "<<pre_lamps[i].angle;
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
                    theta_current=-(180-theta_current);
        for(j=1; j<=4 || (i+j)<size; j++){
            //计算比例，筛选灯管
            
            const RotatedRect &compare = pre_lamps[i+j];
            //cout<<"i"<<i<<"i+j"<<i+j<<endl;
            float theta_compare=abs(compare.angle);
            if(theta_compare>95)
                    theta_compare=-(180-theta_compare);
            diff_angle = abs(theta_compare - theta_current);
            //cout<<"disc: "<<theta_compare<<endl;
            //cout<<"discu: "<<theta_current<<endl;
            //cout<<"disang: "<<diff_angle<<endl;
            if(diff_angle > 15) continue;
            
            diff_height = abs(compare.size.height - current.size.height);
            if(diff_height>30) continue;

            float diff_y = abs(compare.center.y - current.center.y);
            if(diff_y > 30) continue;
            
            dist = distPoint(compare.center, current.center);
           
            //cout<<"distt: "<<dist<<endl;
            if(dist>200 || dist<10) continue;
            avg_height = (compare.size.height + current.size.height) / 2;
            ratio = dist / avg_height;
            //cout<<"ratio: "<<ratio<<endl;
            if(ratio > 5 || ratio < 1) continue;
            
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
        //cout<<"i: "<<i<<" best_match_index: "<<index;
        //cout<<" diff: "<<diff[i]<<endl;
        if(index == -1 || index <= i) continue;
        //找到匹配对
        if(i == best_match_index[index]){
            lamps.push_back(pre_lamps[i]);
            lamps.push_back(pre_lamps[index]);
        }
    }
    //cout<<"lamps counts: "<<lamps.size()/2<<endl;
}

void AutoAim::findBestArmor(vector<RotatedRect> &lamps, Point &bestCenter){

    int lowerIndex = -1;
    int lowerY = 0;
    float diff;
    for(int i=0; i<lamps.size(); i+=2){
        if(i+1 >= lamps.size()) break;
        int y = (lamps[i].center.y + lamps[i+1].center.y)/2;
        if(y > lowerY){
            lowerY = y;
            lowerIndex = i;
        }
    }

    if(lowerIndex == -1){
        if(hasROI){
            hasROI = resizeROI(rectROI, rectROI);
        }
    } else {
        if(!hasROI){
            hasROI = true;
            rectROI.x = lamps[lowerIndex].center.x - lamps[lowerIndex].size.width;
            rectROI.y = lamps[lowerIndex].center.y - lamps[lowerIndex].size.height/2;
            rectROI.width = abs(lamps[lowerIndex].center.x - lamps[lowerIndex+1].center.x) + lamps[lowerIndex].size.width + 
                                lamps[lowerIndex+1].size.width;
            rectROI.height =  lamps[lowerIndex].size.height + lamps[lowerIndex+1].size.height - fabs(lamps[lowerIndex].center.y - 
                                lamps[lowerIndex+1].center.y);                   
            if(rectROI.height < 0 || rectROI.x < 0 || rectROI.y < 0){
                rectROI.x = rectROI.y = rectROI.width = rectROI.height = 0;
            }
            bestCenter.x = (lamps[lowerIndex].center.x + lamps[lowerIndex+1].center.x)/2 + rectROI.x;
            bestCenter.y = (lamps[lowerIndex].center.y + lamps[lowerIndex+1].center.y)/2 + rectROI.y;
        } else {
            int height = (lamps[lowerIndex].size.height + lamps[lowerIndex+1].size.height)/2;
            if(height > 15){
                bestCenter.x = (lamps[lowerIndex].center.x + lamps[lowerIndex+1].center.x)/2 + rectROI.x;
                bestCenter.y = (lamps[lowerIndex].center.y + lamps[lowerIndex+1].center.y)/2 + rectROI.y;
            } else hasROI = false;
        }
    }
    
}

bool AutoAim::resizeROI(Rect &origin, Rect &current){ 
    //记录调用resize的次数
    if(resizeCount==2){
 	    resizeCount=0;//清零
        return false;
    }	
    
    //将ROI区域扩大
    current.x=origin.x-origin.height/2;
    current.y=origin.y-origin.width/2;
    current.height=origin.height+origin.height;
    current.width=origin.width+origin.width;
    
    //判断ROI是否越界
    if(IMG_WIDTH-1-rectROI.x<rectROI.width||IMG_HEIGHT-1-rectROI.y<rectROI.height)
        return false;
    ++resizeCount;
    return true;
}

void test(){
    clock_t start, finish;
    double time_tol;
    VideoCapture cap(1);
    if(!cap.isOpened()) return;
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    Mat src;

    AutoAim autoAim(1280, 720);
    Point bestCenter;
    while(1){
        start=clock();
        cap>>src;
        if(src.empty()) break;
        Mat mask;
        autoAim.setImage(src, mask, autoAim.red);

        vector<RotatedRect> lamps;
        autoAim.findLamp(mask, lamps);
        //cout<<lamps.size()<<endl;

        bestCenter.x = -1;
        vector<Point2f> posAndSpeed;
        autoAim.findBestArmor(lamps, bestCenter);

        //cout<<i<<" "<<centerPoints[i].x<<" "<<centerPoints[i].y<<endl;
        rectangle(src, autoAim.rectROI, Scalar(255,255,255), 7);
        if(bestCenter.x!=-1) circle(src, bestCenter, 20, Scalar(255,255,255), 5);
        finish = clock();
        //cout<<time_tol<<endl;
        time_tol = (double)(finish - start)/ CLOCKS_PER_SEC;
        putText(src, to_string(1.0/time_tol), Point(10,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 2, 8);
        imshow("src", src);
        char c = waitKey(1);
        if((char)c == 27) break;
    }
    cap.release();
}

int main(int argc, char const *argv[]){
    //Mat src = imread(fileName);
    
    test();
    return 0;
}