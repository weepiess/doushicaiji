#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <numeric>

AutoAim::AutoAim(){}

AutoAim::AutoAim(int width, int height, float dt_){
    IMG_WIDTH = width;
    IMG_HEIGHT = height;
    resetROI();
    resizeCount = 0;
    dt=dt_;

    //初始化三维坐标点
    pnpSolver.pushPoints3D(-70.5, -18, 10);
    pnpSolver.pushPoints3D(70.5, -18, 10);
    pnpSolver.pushPoints3D(70.5, 18, -10);
    pnpSolver.pushPoints3D(-70.5, 18, -10);
    //初始化相机参数
    pnpSolver.setCameraMatrix(1020.80666, 0., 695.74256, 0.,1020.80666,388.82902, 0., 0., 1.);
    pnpSolver.setDistortionCoef(0.0058917, 0.269857,0.0026559, 0.00903601,0.393959);

    kf.transitionMatrix=(Mat_<float>(6, 6) <<   
            1,0,dt,0,0,0,   
            0,1,0,dt,0,0,   
            0,0,1,0,dt,0,   
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1 );
    kf.measurementMatrix=(Mat_<float>(6, 6) <<   
            1,0,0,0,0,0,   
            0,1,0,0,0,0,   
            0,0,1,0,0,0,   
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1);  
    kf.measurementNoiseCov=(Mat_<float>(6, 6) <<   
            2000,0,0,0,0,0,  
            0,2000,0,0,0,0,   
            0,0,2000,0,0,0,   
            0,0,0,10000,0,0, 
            0,0,0,0,10000,0,
            0,0,0,0,0,10000);
    kf.statePost=(Mat_<float>(6, 1) <<  0,0,1,0,0,0);
    
    kf.init(6,20000,0);
}


AutoAim::~AutoAim(){}

Point2d cal_x_y(RotatedRect &rect, int is_up){
    float angle = (90-rect.angle)*CV_PI/180;
    Point2d point;
    if(is_up){
        point.x = rect.center.x + rect.size.height/2*cos(angle);
        point.y = rect.center.y - rect.size.height/2*sin(angle);
    } else {
        point.x = rect.center.x - rect.size.height/2*cos(angle);
        point.y = rect.center.y + rect.size.height/2*sin(angle);
    }
    return point;
}

void AutoAim::resetROI(){
    rectROI.x = 0;
    rectROI.y = 0;
    rectROI.width = IMG_WIDTH;
    rectROI.height = IMG_HEIGHT;
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
    Mat channel[3];
    resetROI();
    mask = img(rectROI);
    GaussianBlur(mask, mask, Size(5,5), 0, 0);
    split(mask, channel);
    if(enemyColor == color_blue){
        threshold(channel[0] - channel[2], mask, 10, 255, THRESH_BINARY);
    } else if (enemyColor == color_red){
        threshold(channel[2] - channel[0], mask, 0, 255, THRESH_BINARY+THRESH_OTSU);
    } else {
        cout<<"enemyColor has an improper value, please check it again!!!";
        return false;
    }
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(1,4));  
	for(int i=0; i<2; i++){
        dilate(mask,mask,element);
    }
    Canny(mask, mask, 3, 9, 3);
    imshow("mask", mask);
    return true;                            
}

//寻找灯管
void AutoAim::findLamp_rect(Mat &img, vector<RotatedRect> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    RotatedRect temp;
    float lastCenterX = 0, lastCenterY = 0;
    if(contours.size()<40){
        for(int i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp = adjustRRect(minAreaRect(contours[i]));//寻找最小外接矩形
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略

                pre_armor_lamps.push_back(temp);
            }
        }
    }

}
//匹配灯管
void AutoAim::match_lamps(Mat &img, vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
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
    //计算灯管匹配之间的花费
    int dist, avg_height, diff_angle, diff_height, ratio, totalDiff,inside_angle,diff_width;
    int i,j;
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f3f3f;
        int currIndex = -1;
        const RotatedRect &current = pre_armor_lamps[i];
        int theta_current = current.angle;

        for(j=i+1;j<size; j++){
            //计算比例，筛选灯管
            const RotatedRect &compare = pre_armor_lamps[j];
            int theta_compare = compare.angle;
            
            //灯条角度差超过设定角度忽略
            diff_angle = abs(theta_compare - theta_current);
            cout<<"diff_angle"<<diff_angle<<endl;
            if(diff_angle > param_diff_angle) continue;

            //内角小于设定角度忽略
            if(abs(current.center.y - compare.center.y)==0) inside_angle=90;
            else inside_angle = atanf(abs(current.center.x-compare.center.x)/abs(current.center.y-compare.center.y))*180/CV_PI;
            cout<<"inside"<<inside_angle<<endl;
            if(inside_angle<param_inside_angle) continue;
            
            //两灯条高度差超过30个像素点忽略
            diff_height = abs(compare.size.height - current.size.height);
            cout<<"diffhight"<<diff_height<<endl;
            if(diff_height>param_diff_height) continue;

            //两灯条宽度差超过20个像素点忽略
            diff_width=abs(compare.size.width - current.size.width);
            cout<<"diffwidth"<<diff_width<<endl;
            if(diff_width>param_diff_width) continue;

            dist = ImageTool::calc2PointApproDistance(compare.center, current.center);
            cout<<"dist"<<dist<<endl;
            //灯条间距小于10个像素点忽略
            if(dist<10) continue;
            avg_height = (compare.size.height + current.size.height) / 2;
            ratio = dist / avg_height;
            cout<<"ratio: "<<ratio<<endl;
            if(ratio > 4 || ratio < 1) continue;
            
            totalDiff = angle_diff_weight*diff_angle + height_diff_weight*diff_height+1*dist;
            if(totalDiff < currDiff){
                currDiff = totalDiff;
                currIndex = j;
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

    for(i=0; i<size; i++){
        cout<<best_match_index[i]<<endl;
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
    int lowerY=0;
    int lowerIndex=-1;
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        int y = (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
        if(y > lowerY){
            lowerY = y;
            lowerIndex = i;
        }
    }
    //优先锁定图像下方装甲板
    if(lowerIndex == -1){
        resizeCount++;
        if(!broadenRect(rectROI) || resizeCount>3){
            resetROI();
            resizeCount = 0;
        }
    } else {
        resizeCount = 0;
        //cout<<real_armor_lamps[lowerIndex].x<<"  "<<real_armor_lamps[lowerIndex+1].x<<endl;
	    if(real_armor_lamps[lowerIndex].center.x > real_armor_lamps[lowerIndex+1].center.x){
            swap(real_armor_lamps[lowerIndex],real_armor_lamps[lowerIndex+1]);//确保偶数为左灯条，奇数为右灯条
        }
        int height = (real_armor_lamps[lowerIndex].size.height + real_armor_lamps[lowerIndex+1].size.height)/2;
        if(height > 10){//当灯条高度小于10个像素点时放弃锁定，重新寻找合适目标
            //cout<<rectROI.x<<" "<<rectROI.y<<endl;
            bestCenter.x = (real_armor_lamps[lowerIndex].center.x + real_armor_lamps[lowerIndex+1].center.x)/2 + rectROI.x;
            bestCenter.y = (real_armor_lamps[lowerIndex].center.y + real_armor_lamps[lowerIndex+1].center.y)/2 + rectROI.y;
            //cout<<bestCenter<<endl;
        } else resetROI();
    }

    if(bestCenter.x!=-1){
        clock_t finish = clock();
        best_lamps[0] = real_armor_lamps[lowerIndex];
        best_lamps[1] = real_armor_lamps[lowerIndex+1];
        //cout<<best_lamps<<"bestlamps"<<endl;
        rectROI.x = (best_lamps[0].center.x + best_lamps[1].center.x)/2 + rectROI.x - (best_lamps[1].center.x - best_lamps[0].center.x);
        rectROI.y = (best_lamps[0].center.y + best_lamps[1].center.y)/2 + rectROI.y - (best_lamps[0].size.height + best_lamps[1].size.height)/2;
        rectROI.height = best_lamps[0].size.height + best_lamps[1].size.height;
        rectROI.width = 2*(best_lamps[1].center.x - best_lamps[0].center.x);
        if(!makeRectSafe(rectROI))
            resetROI();
    }
}

BaseAim::AimResult AutoAim::aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw){
    if(!setImage(src))
        return AIM_IMAGE_ERROR;

    vector<RotatedRect> pre_armor_lamps;
    vector<RotatedRect> real_armor_lamps;
    bestCenter.x = -1;

    findLamp_rect(src, pre_armor_lamps);
    match_lamps(src, pre_armor_lamps, real_armor_lamps);
    select_armor(real_armor_lamps);

    cout<<bestCenter.x<<endl;
    if(bestCenter.x != -1){
        circle(src, bestCenter, 20, Scalar(255,255,255), 5);
        rectangle(src, rectROI, Scalar(255,0,0), 2);
    }
    imshow("src", src);
    waitKey(1);
    
    if(bestCenter.x!=-1){
        //circle(src, Point(xc1,yc1), 20, Scalar(255,255,0), 2);
        //circle(src, Point(xc2,yc2), 20, Scalar(255,255,0), 2);
 
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],1));//P1
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],1));//P3
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[1],0));//P2
        pnpSolver.pushPoints2D(cal_x_y(best_lamps[0],0));//P4
        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        //pnpSolver.showParams();
        Point3d tvec = pnpSolver.getTvec();
        if(isPredict){
            measurement.at<float>(0)= tvec.x;
            measurement.at<float>(1) = tvec.y;  
            measurement.at<float>(2)= tvec.z;  
            measurement.at<float>(3) = (last_tvec.x-tvec.x)/timeDelay;
            measurement.at<float>(4) = (last_tvec.y-tvec.y)/timeDelay;
            measurement.at<float>(5) = (last_tvec.z-tvec.z)/timeDelay;

            Mat Predict = this->kf.predict();
            kf.correct(measurement);
            float x = Predict.at<float>(0) + Predict.at<float>(3)*dt;
            float y = Predict.at<float>(1) + Predict.at<float>(4)*dt;
            float z = Predict.at<float>(2) + Predict.at<float>(5)*dt;
            pitYaw = calPitchAndYaw(x, y, z, currPitch, currYaw);
        }else{   
            pitYaw = calPitchAndYaw(tvec.x, tvec.y, tvec.z, currPitch, currYaw);
        }
        return AIM_TARGET_FOUND;
    }
    return AIM_TARGET_NOT_FOUND;
}
