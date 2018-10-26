#include"buff.h"
#include <string>
#include <iostream>
#include <algorithm>

Buff::Buff(){}

Buff::~Buff(){}

int cmp(Point &p1, Point &p2){
    return p1.x < p2.x;
}

void Buff::setImage(Mat &image, Mat &mask)
{
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    cvtColor(image,mask,COLOR_BGR2GRAY);
    GaussianBlur(mask, mask, Size(5,5), 5, 5);
    threshold(mask,mask,0,255, THRESH_BINARY+THRESH_OTSU);
    morphologyEx(mask,mask, MORPH_CLOSE, element);
    //Canny(mask, mask, 3, 9, 3);
    imshow("mask",mask);
    ///imshow("image",image);
}
void Buff::findbigbuff(Mat &src,Mat &image)
{
    Rect rect;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    vector<Rect> boundrect(contours.size());
    for(int i=0;i<contours.size();i++)
    {
        rect=boundingRect(contours[i]);
        if((rect.area() > 1000)&&(rect.area() < 2000))
        rectangle(image,rect,Scalar(0,255,0),3);
        boundrect[i] = boundingRect(Mat(contours[i]));
        float x0=boundrect[i].x;
        float y0=boundrect[i].y;
        float w0=boundrect[i].width;
        float h0=boundrect[i].height;
        // if(w0*h0> 1000)
        //     if(w0/h0<1.1)
        if(( w0*h0> 1000)&&(w0/h0 < 1.1)&&(y0 > 200))
        {
            rectangle(image,Point(x0,y0),Point(x0+w0,y0+h0),Scalar(0,255,255),4);
            if(y0<min_y)
                min_y=y0;       
            if(x0<min_x)    
                min_x=x0;
            if(x0+w0>max_x)
                max_x=x0+w0;
        }
    }
    LEDroi.x=min_x;
    LEDroi.y=0;
    LEDroi.width=max_x - min_x;
    LEDroi.height=min_y-10;
    rectangle(image,LEDroi,Scalar(0,255,0),3);
    imshow("src",image);
}

void Buff::findcontours(Mat &src, Mat &image)
{   vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Point pt;
    Rect rect;
    findContours(src,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    for(int i=0;i<contours.size();i++)
    {
        
        rect=boundingRect(contours[i]);
        if((rect.area() > 10000)&&(rect.area() < 20000))
        {
            rectangle(image,rect,Scalar(0,255,0),3);
            pt.x=((rect.tl().x+rect.br().x)/2);
            pt.y=((rect.tl().y+rect.br().y)/2);
            circle(image,pt,5,Scalar(255,255,255),4);
            if(rect.tl().y<min_y)
                min_y=rect.tl().y;       
            if(rect.tl().x<min_x)    
                min_x=rect.tl().x;
            if(rect.br().x>max_x)
                max_x=rect.br().x;
            /*
            找出矩形框区域的左上角&右下角的坐标
            */
        }
    } 
    LEDroi.x=min_x;
    LEDroi.y=0;
    LEDroi.width=max_x - min_x;
    LEDroi.height=min_y;
    /*
    画出第一个粗略ROI区域框选全部的LED部分
    */
    rectangle(image,LEDroi,Scalar(0,255,0),3);
    imshow("srccccsdfsd",image);
}

void Buff::findLED(Mat &src, Mat &image)
{
    Rect rect2;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    src=image(LEDroi);
    findContours(src,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    for(int i=0;i<contours.size();i++)
    {
        rect2=boundingRect(contours[i]);
        cout<<i<<endl;
            
        // for(int j=0;j<contours[i].size();j++)
        // {
        //     Point p=Point(contours[i][j].x,contours[i][j].y);
        //     circle(src,p,2,Scalar(255,0,0),2);
        // }
        ROI.width=abs(rect2.tl().x-rect2.br().x);
        ROI.height=abs(rect2.tl().y-rect2.br().y);
        ROI.x=(rect2.tl().x+rect2.br().x)/2-ROI.width/2+LEDroi.x;
        ROI.y=(rect2.tl().y+rect2.br().y)/2-ROI.height/2+LEDroi.y;
        /*
        筛选出每个LED并存入roisaver中
        */
        roisaver.push_back(ROI);
        circle(src,Point(ROI.x,ROI.y),ROI.width,Scalar(0,255,255),4);
        rectangle(src,ROI,Scalar(0,255,0),4);
        roi=image(ROI);
        if(ROI.width>max)
            max=ROI.width;
        if(ROI.height>max2)
            max2=ROI.height;
            
    }
    ROI.width=max;
    ROI.height=max2;
    /*
    用于将模板放大至和LED一样大
    */
}
    

/*
void Buff::Match(Mat &src,Mat &image)
{ float max=0;
    Mat tem=imread("/home/weepies/4.jpg");
    cvtColor(tem,tem,COLOR_BGR2GRAY);
    for(int i=0;i<roisaver.size();i++)
    {
        circle(image,Point(roisaver[i].x,roisaver[i].y),3,Scalar(0,255,0),3);
        imshow("srccc",image);
        roi=src(roisaver[i]);
        int result_cols = roi.cols - tem.cols + 1;
        int result_rows = roi.rows - tem.rows + 1;
        Mat result(result_cols, result_rows, CV_32FC1);
        resize(roi,roi,Size(30,30),0,0,CV_INTER_CUBIC);
        resize(tem,tem,Size(15,15),0,0,CV_INTER_CUBIC);
        matchTemplate(roi,tem,result,CV_TM_SQDIFF_NORMED);
        normalize(result,result,0,1,NORM_MINMAX,-1);
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc,Mat());
        cout << "匹配度：" << minVal << endl;
        matchLoc = minLoc;
        matchLoc.x=matchLoc.x+roisaver[i].x;
        matchLoc.y=matchLoc.y+roisaver[i].y;
        circle(image,Point(roisaver[1].x,roisaver[1].y),3,Scalar(255,0,0),4);
        //circle(src,Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows),4,Scalar(0,255,255),3);
        if(minVal<0.0000000001)
        {
            cout<<"fucking perfect ****"<<i<<endl;
            circle(image,matchLoc,4,Scalar(255,0,0),3);
            rectangle(image, matchLoc, Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows), Scalar(0, 255, 0), 2, 8, 0);
        }
        //imshow("temtem",tem);
        imshow("dsjf",roi);
    }
}
*/

//直接暴力在粗略LED的roi区域内模板匹配
/*
void Buff::Match(Mat &src,Mat &image)
{   string picname;
    string str1, str2;
    roi=src(LEDroi);
    for(int i=0;i<7;i++)
    {
        stringstream ss1;
        ss1 << i;
		ss1 >> str1;
        picname="/home/weepies/t"+str1+".jpg";
        cout<<picname<<"    str1"<<endl;
        Mat tem=imread(picname);
        resize(tem,tem,Size(max,max2));
        cvtColor(tem,tem,COLOR_BGR2GRAY);
        int result_cols = roi.cols - tem.cols + 1;
        int result_rows = roi.rows - tem.rows + 1;
        Mat result(result_cols, result_rows, CV_32FC1);
        matchTemplate(roi,tem,result,CV_TM_SQDIFF_NORMED);
        normalize(result,result,0,1,NORM_MINMAX,-1);
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc,Mat());
        cout << "匹配度：" << minVal << endl;
        matchLoc = minLoc;
        matchLoc.x=matchLoc.x+LEDroi.x;
        matchLoc.y=matchLoc.y+LEDroi.y;
        circle(image,Point(roisaver[1].x,roisaver[1].y),3,Scalar(255,0,0),4);
        //circle(src,Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows),4,Scalar(0,255,255),3);
        if(minVal<0.0000001)
        {
            cout<<"fucking perfect ****"<<endl;
            cout<<"   i   "<<i<<endl;
            position.push_back(Point(matchLoc.x,i));
            cout<<position<<"    position"<<endl;
            circle(image,matchLoc,4,Scalar(255,0,0),3);
            rectangle(image, matchLoc, Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows), Scalar(0, 255, 0), 2, 8, 0);
        }
        //imshow("temtem",tem);
    }
    // for(int i=0;i<5;i++)
    // {
    //     for(int j=0;j<5-i;j++)
    //     {
    //         if(position[i].x<position[j].x)
    //         swap(position[i],position[j]);
    //     }
    // }

    imshow("dsjf",roi);
    imshow("iamge",image);
}
*/

//在框选出来的每个区域进行模板匹配
/*
void Buff::Match(Mat &src,Mat &image)
{  
    string picname;
    string str1, str2;
    //roi=src(LEDroi);
    for(int i=0;i<7;i++)
    {
        cout<<i<<"       ******   i"<<endl;
        stringstream ss1;
        ss1 << i;
		ss1 >> str1;
        picname="/home/weepies/"+str1+".jpg";
        cout<<picname<<"    str1"<<endl;
        Mat tem=imread(picname);
        resize(tem,tem,Size(max,max2));
        cvtColor(tem,tem,COLOR_BGR2GRAY);
        for(int j=0;j<roisaver.size();j++)
        {
            roi=src(roisaver[j]);
            int result_cols = roi.cols+10 - tem.cols + 1;
            int result_rows = roi.rows+10 - tem.rows + 1;
            Mat result(image.cols, image.rows, CV_32FC1);
            matchTemplate(roi,tem,result,CV_TM_SQDIFF_NORMED);
            normalize(result,result,0,1,NORM_MINMAX,-1);
            minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc,Mat());
            cout << "匹配度：" << minVal << endl;
            matchLoc = minLoc;
            matchLoc.x=matchLoc.x+roisaver[i].x;
            matchLoc.y=matchLoc.y+roisaver[i].y;
            circle(image,Point(roisaver[1].x,roisaver[1].y),3,Scalar(255,0,0),4);
            //circle(src,Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows),4,Scalar(0,255,255),3);
            if(minVal<0.00000000000000001)
            {
                cout<<"fucking perfect ****"<<endl;
                cout<<"   i   "<<i<<endl;
                position.push_back(Point(matchLoc.x,i));
                cout<<position<<"    position"<<endl;
                circle(image,matchLoc,4,Scalar(255,0,0),3);
                rectangle(image, matchLoc, Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows), Scalar(0, 255, 0), 2, 8, 0);
            }
        }
    }
    imshow("dsjf",roi);
    imshow("iamge",image);
}
*/

void Buff::hashcal(Mat &src,Mat &image)
{
    int iDiffNum = 0;
    float minval=10000;
    string picname;
    string str1, str2;
    Mat matDst1, matDst2;
    int iAvg1 = 0, iAvg2 = 0;
    int arr1[64], arr2[64];
    for(int i=1;i<8;i++)
    {
        int count= -1;
        minval=10000;
        cout<<i<<"       ******   i"<<endl;
        stringstream ss1;
        ss1 << i;
		ss1 >> str1;
        picname="/home/weepies/"+str1+".jpg";
        cout<<picname<<"    str1"<<endl;
        Mat tem=imread(picname);
        cvtColor(tem,tem,COLOR_BGR2GRAY);
        matDst2=tem;
        resize(matDst2,matDst2,Size(8,8),0,0,CV_INTER_CUBIC);
        //将尺寸缩小为8×8
        for(int j=0;j<roisaver.size();j++)
        {
            roi=src(roisaver[j]);
            matDst1=roi;
            resize(matDst1,matDst1,Size(8,8),0,0,CV_INTER_CUBIC);
        /*
        简化色彩，将缩小后的图像，转为64级灰度
        */
        for (int i = 0; i < 8; i++)
        {
            uchar* data1 = matDst1.ptr<uchar>(i);
            uchar* data2 = matDst2.ptr<uchar>(i);
            int tmp = i * 8;
            for (int j = 0; j < 8; j++) 
            {
                int tmp1 = tmp + j;
                arr1[tmp1] = data1[j] / 4 * 4;
                arr2[tmp1] = data2[j] / 4 * 4;
                iAvg1 += arr1[tmp1];
                iAvg2 += arr2[tmp1];
            }
        }
        //计算64个像素灰度平均值
        iAvg1 /= 64;
        iAvg2 /= 64;
        //将每个像素的灰度，与平均值进行比较，大于或等于平均值记为1，小于平均值记为0
        for (int i = 0; i < 64; i++) 
        {
            arr1[i] = (arr1[i] >= iAvg1) ? 1 : 0;
            arr2[i] = (arr2[i] >= iAvg2) ? 1 : 0;
        }
        iDiffNum = 0;
        //计算哈希值
        for (int i = 0; i < 64; i++)
            if (arr1[i] != arr2[i])
                ++iDiffNum;
        if(iDiffNum < minval)
        {
            minval = iDiffNum;
            count = j;
        }
        }
        /*
        找到一个最优解
        */
        if (minval < 10)
        {
            cout<<"匹配成功"<<endl;
            position.push_back(Point(roisaver[count].x,i));
        }
    }
    sort(position.begin(),position.end(), cmp);
        for(int i=0;i<5;i++)
            cout<<position[i].y<<endl;;
}



