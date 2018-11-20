/**
 * todo:
 * 1.对于近在眼前的装甲板需要更高的反应速度，忽略很多条件
 * 2.灯条流水灯效果的识别不能完全依赖于角度
 * 3.灯条在整个车身是斜着的时候识别不是很稳
 * 4.运动中的灯条需要给更加严格的条件
 */
#include "infantry_auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>

//#define DEBUG
#define SHOW_IMG

InfantryAutoAim::InfantryAutoAim(){}

InfantryAutoAim::~InfantryAutoAim(){}

void InfantryAutoAim::init(unsigned char color){
    initParams(color);
    //一定要放在宽高初始化之后
    resetROI();
    resizeCount = 0;
    //第一帧初始化避免开始测距不对的情况
    //lastValiableTvec.z = 0;
    farEnsureCount = nearEnsureCount = landspaceEnsureCount = 0;

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

//图像预处理
bool InfantryAutoAim::setImage(Mat &img){
    if(img.empty()) return false;
    Mat channel[3];

    if(++frameCount >= 120){ //强制全局搜索
        resetROI();
        split(img, channel);
    } else if(isGlobalSearch()){//正常情况下的全局搜索
        //不用mask=img(rectROI)，减少复制矩阵的时间
        split(img, channel);
    } else {
        resetROI();
        mask = img(rectROI);
        split(mask, channel);
    }
    if(params.enemy_color == color_blue){
        GaussianBlur(channel[0] - channel[2], mask, Size(5,5), 0);
    } else if (params.enemy_color == color_red){
        GaussianBlur(channel[2] - channel[1], mask, Size(5,5), 3, 1);
        //medianBlur(channel[2] - channel[1], mask, 5);
    } else {
        cout<<"enemyColor has an improper value, please check it again!!!";
        return false;
    }
    threshold(mask, mask, params.substract_thresh, 255, THRESH_BINARY);
    imshow("mask", mask);
    return true;
}

//寻找灯管
void InfantryAutoAim::findLamp_rect(vector<RotatedRect> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(rectROI.x, rectROI.y));

    RotatedRect temp;
    //防止被敌方激光瞄到镜头，红光一下太多程序崩溃
    if(contours.size() > 50) return;
    for(int i=0; i<contours.size(); i++){
        //实践中发现矩形拟合的效果更好，对抖动的响应更小
        temp = adjustRRect(minAreaRect(contours[i]));//寻找最小外接矩形
        //筛选出一定不是灯条的矩形
        if(abs(temp.angle) > params.max_allow_angle) continue; //最大偏角限制
        if(temp.size.height < temp.size.width*1.2) continue; //圆形限制
        if(temp.size.area() < 10) continue; //面积限制
        pre_armor_lamps.push_back(temp);
    }
}

//匹配灯管
void InfantryAutoAim::match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
    if(pre_armor_lamps.size() <= 1) return;

    //权重，越不利的因素应该越大，角度差越大越不可能匹配，高度比例越大也越不可能，内角越小越不可能，所以内角采用90度减去当前内角
    //11.17更新：角度全面放开，从其他方面限制，减少跳动 效果：目前还行，但是可能耗时比较久，同时由于对角度的放开，在多车时可能误判增加
    //11.18更新：从图像处理的角度考虑解决水波纹，逻辑上解决可能会多出很多问题
    int angle_diff_weight = 5;
    int height_ratio_weight = 2;
    int inside_angle_weight = 3;

    //初始化
    int size = pre_armor_lamps.size();
    vector<float> diff(size, 0x3f3f3f3f);
    vector<float> best_match_index(size, -1);
    
    for(int i=0; i<pre_armor_lamps.size(); i++){
        cout<<pre_armor_lamps[i].angle<<" "<<pre_armor_lamps[i].size.height<<" "<<pre_armor_lamps[i].size.width<<" ---- ";
    }
    cout<<endl;

    //与灯管花费计算有关的变量
    float diff_angle, height_ratio, inside_angle, ratio, totalDiff, dist, avg_height;
    int i,j;

    /*
    //临时变量
    float temp_max_diff_angle = params.max_diff_angle;
    float temp_dis_height_ratio = params.max_dis_height_ratio;

    //预处理，如果有近处的灯条为了防止水波纹效果，降低角度的权重并且放宽角度差限制，远处则降低高度的权重，提高内角和角度的权重
    for(i=0, j=0, k=0, m=0; i<size; i++){
        if(pre_armor_lamps[i].size.height > params.near_lamp_height){
            ++j;
            if(j >= 2){ //有至少两个识别点在近处，认为有在近处的灯条
                angle_diff_weight = 2;
                temp_max_diff_angle = params.max_diff_angle - 2; //5
                temp_dis_height_ratio -= 0.3; //2.9
                break;
            }
        } else if(pre_armor_lamps[i].size.height > params.mid_lamp_height){
            ++k;
            if(k >= 2){ //有至少两个识别点在中等距离
                angle_diff_weight = 2;
                temp_max_diff_angle = params.max_diff_angle - 1; //6
                temp_dis_height_ratio -= 0.2; //3
                break;
            }
        } else if(pre_armor_lamps[i].size.height < params.far_lamp_height)
            ++m;
    }
    //近处没有适合的灯条，同时远处有适合的时候
    if(m >= size-1){
        height_ratio_weight = 1;
        angle_diff_weight = 3;
        inside_angle_weight = 5;
        temp_dis_height_ratio += 1; //4.2
        temp_max_diff_angle = params.max_diff_angle + 1; //8
    }
    */
    
    //精细化处理灯条，生成最后的匹配对
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f;
        int currIndex = -1;
        const RotatedRect &current = pre_armor_lamps[i];

        for(j=i+1; j<size; j++){
            const RotatedRect &compare = pre_armor_lamps[j];

            //灯条角度差超过设定角度忽略
            diff_angle = abs(compare.angle - current.angle);
            if(diff_angle > params.max_diff_angle) continue;

            //内角小于设定角度忽略
            if(current.center.y - compare.center.y == 0) inside_angle=90;
            else inside_angle = atanf(abs(current.center.x-compare.center.x)/abs(current.center.y-compare.center.y))*180/CV_PI;
            if(inside_angle < params.min_inside_angle) continue;
            
            //两灯条高度比例不在范围内则忽略
            if(compare.size.height > current.size.height)
                height_ratio = compare.size.height*1.0f/current.size.height;
            else
                height_ratio = current.size.height*1.0f/compare.size.height;
            if(height_ratio > params.max_height_ratio) continue;

            //灯条之间的距离与灯条的平均长度之比需要在一定范围之内
            dist = ImageTool::calc2PointApproDistance(compare.center, current.center);
            avg_height = (compare.size.height + current.size.height) / 2;
            ratio = dist / avg_height;
            cout<<ratio<<endl;
            if(ratio > params.max_dis_height_ratio || ratio < params.min_dis_height_ratio) continue;
            
            //角度差约束会在几度之内，高度比例约束会在1到1.x之内，内角约束大致在几十度到90度(目前70-90)，
            //实际上用90度减去角度应该在0-几十度之内，且越小越好
            //用归一化的值算最后的花费，避免不同值的取值范围不同
            totalDiff = angle_diff_weight * (diff_angle/params.max_diff_angle) //角度花费
                        + height_ratio_weight * ((height_ratio-1)/(params.max_height_ratio-1)) //高度比例花费
                        + inside_angle_weight * ((90-inside_angle)/(90-params.min_inside_angle)); //内角花费

            //更新最优匹配
            if(totalDiff < currDiff){
                currDiff = totalDiff;
                currIndex = j;
            }

            #ifdef DEBUG
            cout<<"diff_angle: "<<diff_angle<<" "<<compare.angle<<" "<<current.angle<<endl;
            cout<<"inside_angle: "<<inside_angle<<endl;
            cout<<"height_ratio: "<<height_ratio<<endl;
            cout<<"ratio: "<<ratio<<endl;
            cout<<"i: "<<i<<" j: "<<j<<" totalDiff: "<<totalDiff<<" currDiff: "<<currDiff<<endl;
            #endif
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
        int index = best_match_index[i];
        if(index == -1 || index <= i) continue;
        if(i == best_match_index[index]){
            real_armor_lamps.push_back(pre_armor_lamps[i]);
            real_armor_lamps.push_back(pre_armor_lamps[index]);
        }
    }
}

//选择最优装甲板
void InfantryAutoAim::select_armor(vector<RotatedRect> real_armor_lamps){
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
    bool isAllAbove = true; //避免都在远处的情况不会进入设置bestCenter的判断语句
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        int y = (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
        if(y > lowerY){
            lowerY = y;
            leftIndex = i;
        }
        if(max(real_armor_lamps[leftIndex].size.height, real_armor_lamps[leftIndex+1].size.height) > params.far_roi_height) isAllAbove = false;
    }
    if(leftIndex == -1){
        resizeCount++;
        int widthAdded, heightAdded;
        //说明之前的roi在近处，由于近处车子移动导致像素点变的很快，所以roi扩大的程度需要变快
        if(rectROI.width > params.near_roi_width || rectROI.height > params.near_roi_height){
            widthAdded = 15;
            heightAdded = 10;
        } else {
            widthAdded = 5;
            heightAdded = 3;
        }
        //扩大roi并且判断是否应该重置roi
        if(!broadenRect(widthAdded, heightAdded, rectROI) || resizeCount>30){
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
        if(maxHeight > params.far_roi_height || isAllAbove){
            bestCenter.x = (real_armor_lamps[leftIndex].center.x + real_armor_lamps[leftIndex+1].center.x)/2;
            bestCenter.y = (real_armor_lamps[leftIndex].center.y + real_armor_lamps[leftIndex+1].center.y)/2;
        } else resetROI();
    }

    //善后处理，更新roi区域
    if(bestCenter.x != -1){
        //cout<<best_lamps[0].angle<<" "<<best_lamps[0].size.height<<" "<<best_lamps[0].size.width<<" ---- ";
        //cout<<best_lamps[1].angle<<" "<<best_lamps[1].size.height<<" "<<best_lamps[1].size.width<<endl;
        best_lamps[0] = real_armor_lamps[leftIndex];
        best_lamps[1] = real_armor_lamps[leftIndex+1];
        rectROI.x = (best_lamps[0].center.x + best_lamps[1].center.x)/2 - (best_lamps[1].center.x - best_lamps[0].center.x);
        rectROI.y = (best_lamps[0].center.y + best_lamps[1].center.y)/2 - (best_lamps[0].size.height + best_lamps[1].size.height)/2;
        rectROI.height = best_lamps[0].size.height + best_lamps[1].size.height;
        rectROI.width = 2*(best_lamps[1].center.x - best_lamps[0].center.x);
        if(!makeRectSafe(rectROI))
            resetROI();
    }
}

//现在考虑的使用对vx vy vz都进行滤波，抑制数据跳变
bool InfantryAutoAim::rejudgeByTvec(double x, double y, double z){
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

AimResult InfantryAutoAim::aim(Mat &src, Point2f &pitYaw){
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

        #ifdef SHOW_IMG
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
            //cout<<tvec.x<<" "<<tvec.y<<" "<<tvec.z<<" "<<x<<" "<<y<<" "<<z<<endl;
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

void InfantryAutoAim::initParams(unsigned char color){
    //初始化总体参数
    FileStorage paramsFile("../libAim/infantryAutoAim/res/infantry_config.yaml", cv::FileStorage::READ);
    params.enemy_color = color;
    if(params.enemy_color == color_red){
        paramsFile["red_sbstract_thresh"] >> params.substract_thresh;
    } else {
        paramsFile["blue_sbstract_thresh"] >> params.substract_thresh;
    }
    paramsFile["max_allow_angle"] >> params.max_allow_angle;
    paramsFile["max_diff_angle"] >> params.max_diff_angle;
    paramsFile["max_height_ratio"] >> params.max_height_ratio;
    paramsFile["min_inside_angle"] >> params.min_inside_angle;
    paramsFile["near_roi_height"] >> params.near_roi_height;
    paramsFile["near_roi_width"] >> params.near_roi_width;
    paramsFile["img_width"] >> params.img_width;
    paramsFile["img_height"] >> params.img_height;
    paramsFile["max_dis_height_ratio"] >> params.max_dis_height_ratio;
    paramsFile["min_dis_height_ratio"] >> params.min_dis_height_ratio;
    paramsFile["near_lamp_height"] >> params.near_lamp_height;
    paramsFile["far_lamp_height"] >> params.far_lamp_height;
    paramsFile["mid_lamp_height"] >> params.mid_lamp_height;
    paramsFile.release();

    //读入相机参数
    FileStorage cameraFile("../libAim/infantryAutoAim/res/camera_params.yaml", cv::FileStorage::READ);
    float fx, fy, cx, cy, rdx, rdy, tdx, tdy, x_length, y_length, z_length;
    cameraFile["fx"] >> fx;
    cameraFile["fy"] >> fy;
    cameraFile["cx"] >> cx;
    cameraFile["cy"] >> cy;
    cameraFile["rdx"] >> rdx;
    cameraFile["rdy"] >> rdy;
    cameraFile["tdx"] >> tdx;
    cameraFile["tdy"] >> tdy;
    cameraFile["x_length"] >> x_length;
    cameraFile["y_length"] >> y_length;
    cameraFile["z_length"] >> z_length;

    //初始化三维坐标点
    pnpSolver.pushPoints3D(x_length, y_length, z_length);
    //初始化相机参数
    pnpSolver.setCameraMatrix(fx, fy, cx, cy);
    pnpSolver.setDistortionCoef(rdx, rdy, tdx, tdy);
    cameraFile.release();
}

bool InfantryAutoAim::isGlobalSearch(){
    return rectROI.x==0 && rectROI.y==0 && rectROI.width==params.img_width && rectROI.height==params.img_height;
}

Point2d InfantryAutoAim::cal_x_y(RotatedRect &rect, int is_up){
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

void InfantryAutoAim::resetROI(){
    rectROI.x = 0;
    rectROI.y = 0;
    rectROI.width = params.img_width;
    rectROI.height = params.img_height;
    frameCount = 0;
}
