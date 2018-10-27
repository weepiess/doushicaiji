#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>

const float AutoAim::max_offset_angle = 30;

AutoAim::AutoAim(){}

AutoAim::AutoAim(int width, int height,float dt_){
    IMG_WIDTH = width;
    IMG_HEIGHT = height;
    rectROI.x=0;
    rectROI.y=0;
    rectROI.width=width;
    rectROI.height=height;
    resizeCount = 0;
    dt=dt_;
    //初始化三维坐标点

    pnpSolver.pushPoints3D(-70.5, -18, 10);
    pnpSolver.pushPoints3D(-70.5, 18, -10);
    pnpSolver.pushPoints3D(64.5, -18, 10);
    pnpSolver.pushPoints3D(64.5, 18, -10);
    //初始化相机参数
    pnpSolver.setCameraMatrix(1020.80666, 0., 695.74256, 0.,1020.80666,388.82902, 0., 0., 1.);
    pnpSolver.setDistortionCoef(0.0058917, 0.269857,0.0026559, 0.00903601,0.393959);
    cout<<"3"<<endl;
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
    cout<<"4"<<endl;
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

bool cmp_rect(Point2f &x, Point2f &y){
   return x.y<y.y;
}
bool AutoAim::checkBorder(){
    if(IMG_WIDTH-1-rectROI.x<rectROI.width || IMG_HEIGHT-1-rectROI.y<rectROI.height || rectROI.x<0 || rectROI.y <0){
        rectROI.x = rectROI.y = 0;
        rectROI.width = IMG_WIDTH;
        rectROI.height = IMG_HEIGHT;
        return false;
    }
    return true;
}
bool AutoAim::resizeROI(Rect &origin, Rect &current){
    if(origin.width==IMG_WIDTH||origin.height==IMG_HEIGHT)
        return false; 
    //记录调用resize的次数
    if(resizeCount==3){
 	    resizeCount=0;
        current.x=0;
        current.y=0;
        current.height=IMG_HEIGHT;
        current.width=IMG_WIDTH;
        return false;
    }	
    
    //将ROI区域扩大
    current.x=origin.x-origin.width/2 ;
    current.y=origin.y-origin.height/2 ;
    current.height=origin.height+origin.height ;
    current.width=origin.width+origin.width ;
    
    //判断ROI是否越界
    bool isSuccess = checkBorder();
    if(isSuccess){
        ++resizeCount;
    }
    return isSuccess;
}
Point2d cal_x_y(int x,int y,int H,float angle,int is_up){
    float theta;
    Point2d point;
    if(angle>90&&angle<=180){
        theta=(180-angle)*CV_PI/180;
        if(is_up){
            point.x=x-sin(theta)*H/2;
            point.y=y-cos(theta)*H/2;
        }else{
            point.x=x+sin(theta)*H/2;
            point.y=y+cos(theta)*H/2;
        }
    }
    else if(angle>180){
        theta=-(180-angle)*CV_PI/180;
        if(is_up){
            point.x=x+sin(theta)*H/2;
            point.y=y-cos(theta)*H/2;
        }else{
            point.x=x-sin(theta)*H/2;
            point.y=y+cos(theta)*H/2;
        }
    }
    else{ 
        theta=angle*CV_PI/180;
        if(is_up){
            point.x=x+sin(theta)*H/2;
            point.y=y-cos(theta)*H/2;
        }else{
            point.x=x-sin(theta)*H/2;
            point.y=y+cos(theta)*H/2;
        }
        
    }
    return point;
}

Point2f AutoAim::calPitchAndYaw(float x, float y, float z, float currPitch, float currYaw){   
    Point2f angle;
    
    angle.x = MathTools::gravityKiller((z+170)/ 1000.0, (y-60)/1000.0, 15, currPitch); //pitch
    angle.y = -atanf((x+30)/(z+170))*180/CV_PI;   //yaw

    return angle;
}
void AutoAim::change_roi(int &x, int &y, int &width, int &height){
    if(x<0){
        width = width+x;
        x = 0;
    }
    if(y<0){
        height = height+y;
        y = 0;
    }
    if((x+width) > IMG_WIDTH-1)
        width += IMG_WIDTH-1-(x+width);
    if((y+height) > IMG_HEIGHT-1)
        height += IMG_HEIGHT-1-(y+height);

    rectROI.x = x;
    rectROI.y = y;
    rectROI.width = width;
    rectROI.height = height;
}

int AutoAim::cal_angle(Point2f point[4])
{
    
    int x1 = (point[0].x + point[1].x)/2;
    int y1 = (point[0].y + point[1].y)/2;
    int x2 = (point[2].x + point[3].x)/2;
    int y2 = (point[2].y + point[3].y)/2;
    if(x1==x2) return 90;
    else return 90-(atan((y1-y2)/(x1-x2)*180/CV_PI));
}

//两点距离
float distPoint(Point2f center1, Point2f center2){
    return abs(center1.x-center2.x) + abs(center1.y-center2.y);
}
void AutoAim::set_parameters(int angel,int inside_angle, int height, int width){
    param_diff_angle = angel;
    param_inside_angle = inside_angle;
    param_diff_height = height;
    param_diff_width = width;
}
//图像预处理
void AutoAim::setImage(Mat &img, int enemyColor){
    Mat channel[3];
    mask = img(rectROI);
    
    GaussianBlur(mask, mask, Size(5,5), 0, 0);
    split(mask,channel); 
    threshold(enemyColor==AutoAim::color_red ? (channel[2]-channel[0]) : (channel[0] - channel[2]), mask, 0, 255, THRESH_BINARY+THRESH_OTSU); //自适应阈值
	Mat element = getStructuringElement(MORPH_ELLIPSE,Size(1,4));  
	dilate(mask,mask,element);
 	dilate(mask,mask,element);
    Canny(mask, mask, 3, 9, 3);                                     
}
//寻找灯管
void AutoAim::findLamp_rect(Mat &img,vector<Armor_lamps> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    // findContours(mask, contours, hierarcy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    findContours(mask,contours,hierarcy,CV_RETR_EXTERNAL,CHAIN_APPROX_NONE);    
    RotatedRect temp;
    
    Armor_lamps armor_lamp;

    float lastCenterX = 0, lastCenterY = 0;
    if(contours.size()<40){
        for(int i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp=minAreaRect(contours[i]);//寻找最小外接矩形
                
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略
                temp.points(armor_lamp.point);
                if(temp.size.height<temp.size.width) continue;
                sort(armor_lamp.point,armor_lamp.point+4,cmp_rect);
                armor_lamp.angle=cal_angle(armor_lamp.point);
                cout<<armor_lamp.angle<<endl;
                armor_lamp.width=abs(armor_lamp.point[0].x-armor_lamp.point[1].x);
                armor_lamp.height=abs(armor_lamp.point[0].y-armor_lamp.point[3].y);
                armor_lamp.x=((armor_lamp.point[0].x+armor_lamp.point[1].x)/2+(armor_lamp.point[2].x-armor_lamp.point[3].x)/2);
                armor_lamp.y=((armor_lamp.point[0].y+armor_lamp.point[1].y)/2+(armor_lamp.point[2].y-armor_lamp.point[3].y)/2);
                cout<<armor_lamp.x<<endl;
                if(armor_lamp.width>armor_lamp.height) continue;
                for(int i=0;i<4;i++){
                    line(img,armor_lamp.point[i],armor_lamp.point[(i+1)%4],Scalar(0,255,0),2,8);
                }
                
                putText(img,to_string(i),Point(armor_lamp.point[3].x+10,armor_lamp.point[3].y+50),FONT_HERSHEY_SIMPLEX,1,Scalar(255,0,0),4,8);
                pre_armor_lamps.push_back(armor_lamp);
            }
        }
    }

}
//匹配灯管
void AutoAim::match_lamps(Mat &img,vector<Armor_lamps> &pre_armor_lamps,vector<Armor_lamps> &real_armor_lamps){
    int angle_diff_weight = 3;
    int height_diff_weight = 2;
    int size = pre_armor_lamps.size();
    vector<float> diff(size,0);
    vector<float> best_match_index(size,0);
    for(int i=0; i<size; i++){
        diff[i] = 0x3f3f3f3f;
        best_match_index[i] = -1;
    }
    int dist, avg_height, diff_angle, diff_height, ratio, totalDiff,inside_angle,diff_width;
    int i,j;
    for(i=0; i<size; i++){
        float currDiff = 0x3f3f3f3f;
        int currIndex = -1;
        const Armor_lamps &current = pre_armor_lamps[i];
        int theta_current=current.angle;
        for(j=i+1;j<size; j++){
            cout<<i<<j<<endl;
            //计算比例，筛选灯管
            const Armor_lamps &compare = pre_armor_lamps[j];
            int theta_compare=compare.angle;
            
            diff_angle = abs(theta_compare - theta_current);
            cout<<"diff_angle"<<diff_angle<<endl;
            if(diff_angle > param_diff_angle) continue;//灯条角度差超过设定角度忽略
            if(abs(current.y-compare.y)==0) inside_angle=90;
            else inside_angle=int(atanf(abs(current.x-compare.x)/abs(current.y-compare.y))*180/CV_PI);
            cout<<"inside"<<inside_angle<<endl;
            if(inside_angle<param_inside_angle) continue;//内角小于设定角度忽略
            
            diff_height = abs(compare.height - current.height);
            cout<<"diffhight"<<diff_height<<endl;
            if(diff_height>param_diff_height) continue;//两灯条高度差超过30个像素点忽略
            diff_width=abs(compare.width-current.width);
            cout<<"diffwidth"<<diff_width<<endl;
            if(diff_width>param_diff_width) continue;//两灯条宽度差超过20个像素点忽略    
            dist = distPoint(Point2f(compare.x,compare.y),Point2f(current.x,current.y));
            cout<<"dist"<<dist<<endl;
            if(dist<10) continue;//灯条间距小于10个像素点忽略
            avg_height = (compare.height + current.height) / 2;
            ratio = dist / avg_height;
            cout<<"ratio: "<<ratio<<endl;
            if(ratio > 10) continue;//|| ratio < 1) continue;//两灯条间距与灯条高度比值大于10或小于1忽略
            
            totalDiff = angle_diff_weight*diff_angle + height_diff_weight*diff_height+1*dist;
            cout<<"Diff: "<<totalDiff<<endl;
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
    //找到匹配对
        if(i == best_match_index[index]){
            cout<<"find"<<endl;
            real_armor_lamps.push_back(pre_armor_lamps[i]);
            real_armor_lamps.push_back(pre_armor_lamps[index]);
        }
    }
}
void AutoAim::select_armor(vector<Armor_lamps> real_armor_lamps,Mat &best_lamps){
    int lowerY=0;
    int lowerIndex=-1;
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;
        int y = (real_armor_lamps[i].y + real_armor_lamps[i+1].y)/2;
        if(y > lowerY){
            lowerY = y;
            lowerIndex = i;
        }
    }//优先锁定图像下方装甲板
    if(lowerIndex == -1){
        resizeROI(rectROI, rectROI);
        
    } else {
        resizeCount = 0;
        cout<<"ds"<<endl;
        cout<<real_armor_lamps[lowerIndex].x<<"  "<<real_armor_lamps[lowerIndex+1].x<<endl;
	    if(real_armor_lamps[lowerIndex].x>real_armor_lamps[lowerIndex+1].x){
            swap(real_armor_lamps[lowerIndex],real_armor_lamps[lowerIndex+1]);//确保偶数为左灯条，奇数为右灯条
        }
        int height = (real_armor_lamps[lowerIndex].height + real_armor_lamps[lowerIndex+1].height)/2;
        if(height > 1){//当灯条高度小于10个像素点时放弃锁定，重新寻找合适目标
            cout<<rectROI.x<<" "<<rectROI.y<<endl;
            bestCenter.x = (real_armor_lamps[lowerIndex].x + real_armor_lamps[lowerIndex+1].x)/2 + rectROI.x;
            bestCenter.y = (real_armor_lamps[lowerIndex].y + real_armor_lamps[lowerIndex+1].y)/2 + rectROI.y;
            cout<<bestCenter<<endl;
            //if(!checkBorder()) bestCenter.x = -1;
        } else {
            rectROI.x = 0;
            rectROI.y = 0;
            rectROI.width = IMG_WIDTH;
            rectROI.height = IMG_HEIGHT;
        }
    }

    if(bestCenter.x!=-1){
        clock_t finish = clock();
        best_lamps.at<int>(0)=real_armor_lamps[lowerIndex].x + rectROI.x;
        best_lamps.at<int>(1)=real_armor_lamps[lowerIndex].y + rectROI.y;
        best_lamps.at<int>(4)=real_armor_lamps[lowerIndex+1].x + rectROI.x;
        best_lamps.at<int>(5)=real_armor_lamps[lowerIndex+1].y + rectROI.y;
        best_lamps.at<int>(2)=real_armor_lamps[lowerIndex].height;
        best_lamps.at<int>(3)=real_armor_lamps[lowerIndex].angle;
        best_lamps.at<int>(6)=real_armor_lamps[lowerIndex+1].height;
        best_lamps.at<int>(7)=real_armor_lamps[lowerIndex+1].angle;
        cout<<best_lamps<<"bestlamps"<<endl;
        int rectx = ((real_armor_lamps[lowerIndex].x + real_armor_lamps[lowerIndex+1].x)/2)+rectROI.x - (real_armor_lamps[lowerIndex+1].x - real_armor_lamps[lowerIndex].x);
        int recty = (real_armor_lamps[lowerIndex].y + real_armor_lamps[lowerIndex+1].y)/2+rectROI.y - ( real_armor_lamps[lowerIndex].height + real_armor_lamps[lowerIndex+1].height)/2;
        int recthight = real_armor_lamps[lowerIndex].height + real_armor_lamps[lowerIndex+1].height;
        int rectwidth = 2*(real_armor_lamps[lowerIndex+1].x - real_armor_lamps[lowerIndex].x);
        change_roi(rectx,recty,rectwidth,recthight);
    }
}

Point2f AutoAim::aim(Mat src,Mat &best_lamps , float currPitch, float currYaw, int is_predict,double time_delay){  
    Point2f angle;
    if(bestCenter.x != -1){
        circle(src, bestCenter, 20, Scalar(255,255,255), 5);
        rectangle(src, rectROI, Scalar(255,0,0), 10);
    }

    
    if(bestCenter.x!=-1)
    {
        int h1=best_lamps.at<float>(2);
        int a1=best_lamps.at<float>(3);//first angle
        int h2=best_lamps.at<float>(6);//第二个灯条 hight
        int a2=best_lamps.at<float>(7);
        int xc1,xc2,yc1,yc2;
        xc1=best_lamps.at<float>(0);//first center x 
        yc1=best_lamps.at<float>(1);//first center y
        xc2=best_lamps.at<float>(4);//第二个灯条 x
        yc2=best_lamps.at<float>(5);//第二个灯条 y
        circle(src, Point(xc1,yc1), 20, Scalar(255,255,0), 2);
        circle(src, Point(xc2,yc2), 20, Scalar(255,255,0), 2);
 
        pnpSolver.pushPoints2D(cal_x_y(xc1,yc1,h1,a1,1));//P1
        pnpSolver.pushPoints2D(cal_x_y(xc1,yc1,h1,a1,0));//P3
        pnpSolver.pushPoints2D(cal_x_y(xc2,yc2,h2,a2,1));//P2
        pnpSolver.pushPoints2D(cal_x_y(xc2,yc2,h2,a2,0));//P4
        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        //pnpSolver.showParams();
        Point3d tvec = pnpSolver.getTvec();
        imshow("src",src);
        waitKey(0);
        if(is_predict){
            measurement.at<float>(0)= tvec.x;
            measurement.at<float>(1) = tvec.y;  
            measurement.at<float>(2)= tvec.z;  
            measurement.at<float>(3) = (last_tvec.x-tvec.x)/time_delay;
            measurement.at<float>(4) = (last_tvec.y-tvec.y)/time_delay;
            measurement.at<float>(5) = (last_tvec.z-tvec.z)/time_delay;
            

            Mat Predict = this->kf.predict();
            kf.correct(measurement);
            float x = Predict.at<float>(0) + Predict.at<float>(3)*dt;
            float y = Predict.at<float>(1) + Predict.at<float>(4)*dt;
            float z = Predict.at<float>(2) + Predict.at<float>(5)*dt;
            angle=calPitchAndYaw(x, y, z, currPitch, currYaw);
        }else{   
            angle=calPitchAndYaw(tvec.x, tvec.y, tvec.z, currPitch, currYaw);
        }
        
        return angle;
    } else {
        return Point2f(180,180);
    }
}