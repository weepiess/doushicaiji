#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>

#define DEBUG

AutoAim::AutoAim(){}

AutoAim::~AutoAim(){}

void AutoAim::init(int width, int height){
    IMG_WIDTH = width;
    IMG_HEIGHT = height;
    //一定要放在宽高初始化之后
    resetROI();
    resizeCount = 0;
    //第一帧初始化避免开始测距不对的情况
    //lastValiableTvec.z = 0;
    farEnsureCount = nearEnsureCount = landspaceEnsureCount = 0;

    //初始化三维坐标点
    pnpSolver.pushPoints3D(-72, -30, 0);
    pnpSolver.pushPoints3D(72, -30, 0);
    pnpSolver.pushPoints3D(72, 30, 0);
    pnpSolver.pushPoints3D(-72, 30, 0);
    //初始化相机参数
    pnpSolver.setCameraMatrix(1022.9, 0, 636.8581, 0, 1026.5, 529.7976, 0, 0, 1);
    pnpSolver.setDistortionCoef(-0.0986, 0.0413, 0, 0);

    //初始化卡尔曼滤波
    //需要调的参数为倒数三个，分别为过程噪声协方差、测量噪声协方差、初始状态不确定度
    simpleKF.init(6, 6, 0, 5, 10000, 0, 0.1);

    simpleKF.kFilter.transitionMatrix=(Mat_<float>(6, 6) <<
            1,0,0,1,0,0,
            0,1,0,0,1,0,
            0,0,1,0,0,1,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1 
    );

    simpleKF.kFilter.measurementNoiseCov=(Mat_<float>(6,6) <<
            2000,0,0,0,0,0,
            0,2000,0,0,0,0,
            0,0,2000,0,0,0,
            0,0,0,10000,0,0,
            0,0,0,0,10000,0,
            0,0,0,0,0,10000
    );
}

bool AutoAim::isGlobalSearch(){
    return rectROI.x==0 && rectROI.y==0 && rectROI.width==IMG_WIDTH && rectROI.height==IMG_HEIGHT;
}

Point2d AutoAim::cal_x_y(RotatedRect &rect, int is_up){
    float angle = (90-rect.angle)*CV_PI/180;
    Point2d point;
    if(is_up){
        point.x = rect.center.x + rect.size.height/2*cos(angle) + rectROI.x;
        point.y = rect.center.y - rect.size.height/2*sin(angle) + rectROI.y;
    } else {
        point.x = rect.center.x - rect.size.height/2*cos(angle) + rectROI.x;
        point.y = rect.center.y + rect.size.height/2*sin(angle) + rectROI.y;
    }
    return point;
}

void AutoAim::resetROI(){
    rectROI.x = 0;
    rectROI.y = 0;
    rectROI.width = IMG_WIDTH;
    rectROI.height = IMG_HEIGHT;
    frameCount = 0;
}

void AutoAim::set_parameters(int angle,int inside_angle, int height, int width){
    param_diff_angle = angle;
    param_inside_angle = inside_angle;
    param_diff_height = height;
    param_diff_width = width;
}

//图像预处理
bool AutoAim::setImage(Mat &img){
    if(img.empty()) return false;
    Mat channel[3], Mask, diff;
    int thresh = 50, substract_thresh = 100;

    if(++frameCount >= 120){ //强制全局搜索
        resetROI();
        split(img, channel);
    } else if(isGlobalSearch()){//正常情况下的全局搜索
        //不用mask=img(rectROI)，减少复制矩阵的时间
        split(img, channel);
    } else {
        //resetROI();
        mask = img(rectROI);
        split(mask, channel);
    }
    if(enemyColor == color_blue){
        diff = channel[0] - channel[1];
        Mask = channel[0];
    } else if (enemyColor == color_red){
        diff = channel[2] - channel[1];
        Mask = channel[2];
    } else {
        cout<<"enemyColor has an improper value, please check it again!!!";
        return false;
    }

    GaussianBlur(Mask, Mask, Size(5,5), 0);
    threshold(Mask, Mask, thresh, 255, THRESH_BINARY);
    threshold(diff, diff, substract_thresh, 255, THRESH_BINARY);

    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(1, 3));
    //dilate(diff, diff, element, Point(-1, -1), 2);

    bitwise_and(Mask, diff, mask);
    //imshow("mask", mask);
    return true;
}

//寻找灯管
void AutoAim::findLamp_rect(vector<RotatedRect> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    RotatedRect temp;
    float lastCenterX = 0, lastCenterY = 0;
    if(contours.size() < 20){
        for(int i=0; i<contours.size(); i++){
            if(contours[i].size() > 5){
                temp = adjustRRect(minAreaRect(contours[i]));//寻找最小外接矩形
                //筛选出一定不是灯条的矩形
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略
                if(temp.size.height < temp.size.width*1.2) continue;

                pre_armor_lamps.push_back(temp);
            }
        }
    }
}

//匹配灯管
void AutoAim::match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
    //权重
    int angle_diff_weight = 3;
    int height_diff_weight = 2;
    //初始化
    int size = pre_armor_lamps.size();
    vector<float> diff(size);
    vector<float> best_match_index(size);
    for(int i=0; i<size; i++){
        diff[i] = 0x3f3f3f3f;
        best_match_index[i] = -1;
    }

    for(int i=0; i<pre_armor_lamps.size(); i++){
        //cout<<pre_armor_lamps[i].angle<<" ";
        //cout<<pre_armor_lamps[i].size.height<<endl;
    }

    //计算灯管匹配之间的花费
    int dist, avg_height, diff_angle, diff_height, ratio, totalDiff, inside_angle, diff_width;
    int i,j;
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f3f3f;
        int currIndex = -1;
        const RotatedRect &current = pre_armor_lamps[i];
        int theta_current = current.angle;

        //cout<<"i: "<<i<<"!!!";

        for(j=i+1;j<size; j++){
            const RotatedRect &compare = pre_armor_lamps[j];

            //灯条角度差超过设定角度忽略
            diff_angle = abs(compare.angle - theta_current);
            //cout<<"diff_angle"<<diff_angle<<" "<<theta_compare<<" "<<theta_current<<endl;
            if(diff_angle > param_diff_angle) continue;

            //内角小于设定角度忽略
            if(abs(current.center.y - compare.center.y)==0) inside_angle=90;
            else inside_angle = atanf(abs(current.center.x-compare.center.x)/abs(current.center.y-compare.center.y))*180/CV_PI;
            //cout<<"inside"<<inside_angle<<endl;
            if(inside_angle < param_inside_angle) continue;
            
            //两灯条高度比例不在范围内则忽略
            diff_height = abs(compare.size.height - current.size.height);
            if(compare.size.height/current.size.height > 1.5 || compare.size.height/current.size.height<0.7) continue;

            //两灯条宽度差超过20个像素点忽略
            //diff_width = abs(compare.size.width - current.size.width);
            //cout<<"diffwidth"<<diff_width<<endl;
            //if(diff_width > param_diff_width) continue;

            dist = ImageTool::calc2PointApproDistance(compare.center, current.center);
            avg_height = (compare.size.height + current.size.height) / 2;
            ratio = dist / avg_height;
            //cout<<"ratio: "<<ratio<<endl;
            if(ratio > 4 || ratio < 1) continue;
            
            totalDiff = angle_diff_weight*diff_angle + height_diff_weight*diff_height;
            //cout<<j<<":"<<totalDiff<<" --- ";
            if(totalDiff < currDiff){
                currDiff = totalDiff;
                currIndex = j;
            }
        }
        //cout<<endl;

        //一对灯管肯定花费是最少的，所以如果当前花费比两个的花费都要少，就记录是最优
        if(currIndex==-1) continue;
        if(currDiff < diff[i] && currDiff < diff[currIndex]){
            diff[i] = currDiff;
            diff[currIndex] = currDiff;
            best_match_index[i] = currIndex;
            best_match_index[currIndex] = i;
        }
    }

    for(i=0; i<size; i++){
        //cout<<"i: "<<diff[i]<<endl;
        int index = best_match_index[i];
        if(index == -1 || index <= i) continue;
        if(i == best_match_index[index]){
            real_armor_lamps.push_back(pre_armor_lamps[i]);
            real_armor_lamps.push_back(pre_armor_lamps[index]);
        }
    }
}

//选择最优装甲板
void AutoAim::select_armor(vector<RotatedRect> real_armor_lamps){
    /*
    int leftIndex = -1;
    //由于每次枪管转动之后目标几乎都在相机的正中心，所以可以把上一帧所在的位置近似看在屏幕的正中心
    //采用权重法，由于y轴的距离很重要，而相机的移动距离是其次
    //分为roi内的搜索和全局搜索，由于全局搜索可能会导致转角很大，而roi内搜索转角不会很大，所以roi内搜索直接找最下面的点就行了
    bool isAllAbove = true;
    int lowerY = -1;
    if(isGlobalSearch()){
        int dist = 0x3f3f3f3f;
        int currIndex = -1;
        for(int i=0; i<real_armor_lamps.size(); i+=2){
            if(i+1 >= real_armor_lamps.size()) break;
            int y = (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
            int x = (real_armor_lamps[i].center.x + real_armor_lamps[i+1].center.x)/2;
            //如果全在镜头上方就找y最大的，有在镜头下方根据dist的大小选择相机转动距离最小的
            if(y > IMG_HEIGHT/7*4){
                isAllAbove = false;
                int currDist = abs(x - IMG_WIDTH/2) + abs(y - IMG_HEIGHT/2);
                if(currDist < dist){
                    dist = currDist;
                    leftIndex = i;
                }
            } else if(lowerY < y){
                lowerY = y;
                currIndex = i;
            }
        }
        if(isAllAbove)  leftIndex = currIndex;
    } else {
        for(int i=0; i<real_armor_lamps.size(); i+=2){
            if(i+1 >= real_armor_lamps.size()) break;
            int y = (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
            if(y > lowerY){
                lowerY = y;
                leftIndex = i;
            }
        }
    }
    */
    //由于上面的逻辑实现出来有问题，而且感觉并没有特别大的优势，所以还是采用优先选择最下方的方法
    int leftIndex = -1;
    int lowerY = 0;
    bool isAllAbove = true;
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        int y = (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
        if(y > lowerY){
            lowerY = y;
            leftIndex = i;
        }
        if(max(real_armor_lamps[leftIndex].size.height, real_armor_lamps[leftIndex+1].size.height) > 20) isAllAbove = false;
    }
    if(leftIndex == -1){
        resizeCount++;
        int widthAdded, heightAdded;
        //说明之前的roi在近处，由于近处车子移动导致像素点变的很快，所以roi扩大的程度需要变快
        if(rectROI.width > 100 || rectROI.height > 50){
            widthAdded = 15;
            heightAdded = 10;
        } else {
            widthAdded = 5;
            heightAdded = 3;
        }
        if(!broadenRect(widthAdded, heightAdded, rectROI) || resizeCount>60){
            resetROI();
            resizeCount = 0;
        }
    } else {
        resizeCount = 0;
        //确保偶数为左灯条，奇数为右灯条
	    if(real_armor_lamps[leftIndex].center.x > real_armor_lamps[leftIndex+1].center.x){
            swap(real_armor_lamps[leftIndex],real_armor_lamps[leftIndex+1]);
        }
        int maxHeight = max(real_armor_lamps[leftIndex].size.height, real_armor_lamps[leftIndex+1].size.height);
        //当灯条高度小于20个像素点时，车子已经跑远了，这时候必须全局搜索一下防止锁定太死
        if(maxHeight > 20 || isAllAbove){
            bestCenter.x = (real_armor_lamps[leftIndex].center.x + real_armor_lamps[leftIndex+1].center.x)/2 + rectROI.x;
            bestCenter.y = (real_armor_lamps[leftIndex].center.y + real_armor_lamps[leftIndex+1].center.y)/2 + rectROI.y;
        } else resetROI();
    }

    if(bestCenter.x != -1){
        best_lamps[0] = real_armor_lamps[leftIndex];
        best_lamps[1] = real_armor_lamps[leftIndex+1];
        rectROI.x = (best_lamps[0].center.x + best_lamps[1].center.x)/2 + rectROI.x - (best_lamps[1].center.x - best_lamps[0].center.x);
        rectROI.y = (best_lamps[0].center.y + best_lamps[1].center.y)/2 + rectROI.y - (best_lamps[0].size.height + best_lamps[1].size.height)/2;
        rectROI.height = best_lamps[0].size.height + best_lamps[1].size.height;
        rectROI.width = 2*(best_lamps[1].center.x - best_lamps[0].center.x);
        if(!makeRectSafe(rectROI))
            resetROI();
    }
}

//现在考虑的使用对vx vy vz都进行滤波，抑制数据跳变
bool AutoAim::rejudgeByTvec(double x, double y, double z){
    //根据先验知识，车子原来在中间，不可能突然消失，所以如果在这段时间内检测到了坐标点的tvec变化很大，说明可能出现了误解
    //x的变化不大，y的变化适中，z的变化可能最大
    //该方法出现的问题：第一是刚开始时前几帧的测距不稳定，所以很多帧后才会出现打击位置，第二是摄像头也晃动的时候，无法检测到，检测延迟很严重
    //实测下镜头晃动不是很剧烈的时候，可以检测到，第一帧的延迟通过对lastValiableTvec.x的初始化解决了
    if(lastValiableTvec.z == 0) return true;
    if(farEnsureCount >= 180 || nearEnsureCount >= 3 || landspaceEnsureCount>=20){
        farEnsureCount = nearEnsureCount = landspaceEnsureCount = 0;
        return true;
    }
    if(abs(lastValiableTvec.z-z) > 600){
        if(lastValiableTvec.z-z < 0) ++farEnsureCount;
        else ++nearEnsureCount; 
        return false;
    }
    if(abs(lastValiableTvec.x-x)>200 || abs(lastValiableTvec.y-y)>300){
        ++landspaceEnsureCount;
        return false;
    }
    return true;
}

AimResult AutoAim::aim(Mat &src, Point2f &pitYaw){
    if(!setImage(src))
        return AIM_IMAGE_ERROR;

    vector<RotatedRect> pre_armor_lamps;
    vector<RotatedRect> real_armor_lamps;
    bestCenter.x = -1;

    findLamp_rect(pre_armor_lamps);
    match_lamps(pre_armor_lamps, real_armor_lamps);
    select_armor(real_armor_lamps);
    
    if(bestCenter.x!=-1){
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],1));//左灯条上方
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],1));//右灯条上方
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],0));//右灯条下方
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],0));//左灯条上方
        pnpSolver.solvePnP();
        //pnpSolver.showParams();
        pnpSolver.clearPoints2D();
        Point3d tvec = pnpSolver.getTvec();
        //cout<<"tvec: "<<tvec<<endl;
        //暂时改成卡尔曼滤波抑制噪点
        //if(!rejudgeByTvec(tvec.x, tvec.y, tvec.z))
        //    return AIM_TARGET_NOT_FOUND;
        //lastValiableTvec = tvec;

        #ifdef DEBUG
        circle(src, bestCenter, 20, Scalar(255,255,255), 5);
        rectangle(src, rectROI, Scalar(255,0,0), 2);
        #endif

        if(isPredict){
            //第一帧做预测，不然后面收敛的很慢
            if(lastValiableTvec.z == 0){
                simpleKF.kFilter.predict();
            }
            //从开始读入图像到执行到此处所用的时间
            timeDelay = ImageTool::timeCount();

            simpleKF.measurement.at<float>(0) = tvec.x;
            simpleKF.measurement.at<float>(1) = tvec.y;
            simpleKF.measurement.at<float>(2) = tvec.z;
            simpleKF.measurement.at<float>(3) = (tvec.x - lastValiableTvec.x)/timeDelay;
            simpleKF.measurement.at<float>(4) = (tvec.y - lastValiableTvec.y)/timeDelay;
            simpleKF.measurement.at<float>(5) = (tvec.z - lastValiableTvec.z)/timeDelay;
            
            //先用测量值去纠正预测值，然后再超前一步预测
            //预测效果还行，没有上机测试
            simpleKF.kFilter.correct(simpleKF.measurement);
            simpleKF.kFilter.predict();

            float x = simpleKF.kFilter.statePost.at<float>(0);
            float y = simpleKF.kFilter.statePost.at<float>(1);
            float z = simpleKF.kFilter.statePost.at<float>(2);
            cout<<tvec.x<<" "<<tvec.y<<" "<<tvec.z<<" "<<x<<" "<<y<<" "<<z<<endl;
            lastValiableTvec.x = x;
            lastValiableTvec.y = y;
            lastValiableTvec.z = z;
        }else{   
            pitYaw = calPitchAndYaw(tvec.x, tvec.y, tvec.z, tvec.z/17, -90, 170);
        }
        return AIM_TARGET_FOUND;
    }
    return AIM_TARGET_NOT_FOUND;
}
