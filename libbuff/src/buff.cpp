#include"buff.h"
#include <string>
#include <iostream>
#include <algorithm>

Buff::Buff(){}

Buff::~Buff(){}

void Buff::setImage(Mat &image, Mat &mask)
{
    Mat channel[3];
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    cvtColor(image,mask,COLOR_BGR2GRAY);
    GaussianBlur(mask, mask, Size(5,5), 5, 5);
    threshold(mask,mask,0,255, THRESH_BINARY+THRESH_OTSU);
    morphologyEx(mask,mask, MORPH_CLOSE, element);
    //Canny(mask, mask, 3, 9, 3);
    ///imshow("mask",mask);
    ///imshow("image",image);
}

void Buff::findcontours(Mat &src, Mat &image)
{
    int min_y=0x3f3f3f3f;
    int max_x=0;
    int min_x=0x3f3f3f3f;
    Mat roi;
    Mat tem=imread("/home/weepies/t2.jpg");
    cvtColor(tem,tem,COLOR_BGR2GRAY);
    //GaussianBlur(tem, tem, Size(5,5), 5, 5);
    int result_cols = src.cols - tem.cols + 1;
    int result_rows = src.rows - tem.rows + 1;
    Mat result(result_cols, result_rows, CV_32FC1);
    
    //findContours(src,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    // for(int i=0;i<contours.size();i++)
    // {
        
    //     rect=boundingRect(contours[i]);
    //     if((rect.area() > 10000)&&(rect.area() < 20000))
    //     {
    //         //cout<<i<<"       i     "<<endl;
    //         //cout<<length<<"  big       length"<<endl;
    //         rectangle(image,rect,Scalar(0,255,0),3);
    //         pt.x=((rect.tl().x+rect.br().x)/2);
    //         pt.y=((rect.tl().y+rect.br().y)/2);
    //         rects.push_back(pt);
    //         match.push_back(Point(pt.x,1));
    //         circle(image,pt,5,Scalar(255,255,255),4);
    //         //cout<<min_length_templete<<"     min_length_templete"<<endl;
    //         //cout<<"length     "<<length<<endl;
    //         if(rect.tl().y<min_y)
    //             min_y=rect.tl().y;       
    //         if(rect.tl().x<min_x)    
    //             min_x=rect.tl().x;
    //         if(rect.br().x>max_x)
    //             max_x=rect.br().x;
    //     }
    // }
    //for(int i=0;i<contours.size();i++)
    //{
        // rect2=boundingRect(contours[71]);
        // if((rect2.tl().y < min_y-1)&&(rect2.tl().x > min_x)&&(rect2.br().x < max_x))
        // {
            //cout<<i<<endl;
            
            // for(int j=0;j<contours[i].size();j++)
            // {
            // Point p=Point(contours[i][j].x,contours[i][j].y);
            // circle(image,p,2,Scalar(255,0,0),2);
            // }
            ROI.width=abs(rect2.tl().x-rect2.br().x);
            ROI.height=abs(rect2.tl().y-rect2.br().y);
            ROI.x=(rect2.tl().x+rect2.br().x)/2-ROI.width/2;
            ROI.y=(rect2.tl().y+rect2.br().y)/2-ROI.height/2;
            //circle(image,Point(ROI.x,ROI.y),ROI.width,Scalar(0,255,255),4);
            //rectangle(image,ROI,Scalar(0,255,0),4);
            //roi=image(ROI);
            //GaussianBlur(result,result,Size(5,5),0,0);
            //imshow("roi",roi);
            //imwrite("/home/weepies/5.jpg",result);
            // GaussianBlur(roi,roi,Size(5,5),0,0);
            // imshow("roi",image);
            matchTemplate(src,tem,result,CV_TM_SQDIFF_NORMED);
            normalize(result,result,0,1,NORM_MINMAX,-1);
            minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
            cout << "匹配度：" << minVal << endl;
            matchLoc = minLoc;
            rectangle(image, matchLoc, Point(matchLoc.x + tem.cols, matchLoc.y + tem.rows), Scalar(0, 255, 0), 2, 8, 0);
            imshow("result",src);
            imshow("tem",tem);
            
       // }
    //}
    /*
    for(int j=0;j<contours[74].size();j++)
    {
        Point p=Point(contours[74][j].x,contours[74][j].y);
        circle(image,p,4,Scalar(255,0,0),4);
    }
    */
   imshow("mask2",image);

}

void Buff::anglesolover(Point &center)
{

}



