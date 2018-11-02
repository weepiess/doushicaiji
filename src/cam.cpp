#include "cam.h"

cam::cam(){}

cam::~cam(){}


Mat region;
const int nFrames = 9; 
void CheckOpenNIError( Status result, string status )
{
    if( result != STATUS_OK )
        cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}


/*
将深度图噪点像素模糊处理
并将映射后的无效区域剔除
*/
void cam::filter(Mat &Pointvalue)
{
    Rect roi=Rect(34,32,545,415);
    Mat PointLabel = Mat::zeros(Pointvalue.size(), CV_8UC1);
    for(int i=32;i<415;i++)
    {
        for(int j=34;j<545;j++)
        {
            if(i>32&&i<414&&j>34&&j<545)
            {
                if((Pointvalue.at<uchar>(i, j)==0)&&(PointLabel.at<uchar>(i, j)==0))
                {
                    Pointvalue.at<uchar>(i, j)=(Pointvalue.at<uchar>(i, j)+Pointvalue.at<uchar>(i+1, j)+Pointvalue.at<uchar>(i-1, j)+Pointvalue.at<uchar>(i, j+1)+Pointvalue.at<uchar>(i, j-1))/5;
                    PointLabel.at<uchar>(i, j)=1;
                    PointLabel.at<uchar>(i+1, j)=1;
                    PointLabel.at<uchar>(i-1, j)=1;
                    PointLabel.at<uchar>(i, j+1)=1;
                    PointLabel.at<uchar>(i, j-1)=1;
                }
            }
        }
    }
region=Pointvalue(roi);
rectangle(Pointvalue,roi,Scalar(100),1);
imshow("ttt",Pointvalue);
imshow("roi",region);
//waitKey(1);
}


/*
在图片中取出5×5的核
在每个核中对像素值取众值
进行众值滤波
*/
void cam::PixelFilter(Mat &image)
{
    Mat DeapthLable=Mat::zeros(image.size(), CV_8UC1);
    int widthval=545;
    int heightval=415;
    int innerthershold=3;
    int outerthershold=7;
    for(int i=0; i<heightval; i++)
    {
        for(int j=0; j<widthval; j++)
        {
            if(image.at<uchar>(i,j)==0)
            {
                unsigned char filterCollection[255][2] = { 0 };
				int innerCount = 0;
				int outerCount = 0;

                for(int dy=-2; dy<3; dy++)
                {
                    for(int dx=-2; dx<3; dx++)
                    {
                        if(dx!=0||dy!=0)
                        {
                            int x_search=j+dx;
                            int y_search=i+dy;
                            if(x_search>=0 && x_search<=widthval && y_search>=0 && y_search<=heightval)
                            {
                                if(image.at<uchar>(i,j)!=0)
                                {
                                    for(int i=0; i<255; i++)
                                    {
                                        if(filterCollection[i][0]==image.at<uchar>(i,j))
                                        {
                                            filterCollection[i][1]++;
                                            break;
                                        }
                                        else if(filterCollection[i][0]==0)
                                        {
                                            filterCollection[i][0] = image.at<uchar>(i,j);
											filterCollection[i][1]++;
											break;
                                        }
                                    }
                                    if (dy != 2 && dy != -2 && dx != 2 && dx != -2)
										innerCount++;
									else
										outerCount++;
                                }
                            }
                        }
                    }
                }
                if (innerCount >= innerthershold || outerCount >= outerthershold)
                {
                    int depth = 0;
					int frequency = 0;
                    for (int i = 0; i < 255; i++)
					{
						if (filterCollection[i][0] == 0)
							break;
						if (filterCollection[i][1] > frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}
                    image.at<uchar>(i,j)=depth;
                }
                else
                    image.at<uchar>(i,j)=0;
            }
        }
    }
}

/*
多帧像素滤波（未使用）
*/
void cam::framfilter(Mat &src0,Mat &src1,Mat &src2,Mat &src3,Mat &src4, Mat &image, int size)
{
    ushort average;
    //cout<<src[4].cols<<endl;
    for(int i=32;i<415;i++){
        for(int j=60;j<545;j++){
            //cout<<"233"<<endl;
            image.at<uchar>(i, j)=(src0.at<uchar>(i, j)+src1.at<uchar>(i, j)+src2.at<uchar>(i, j)+src3.at<uchar>(i, j)+src4.at<uchar>(i, j))/5;
        }
    }
}
/*
寻找立柱特征
*/
void cam::contourfinder(Mat &src,Mat &image)
{
    float area;//区域面积
    float angle;//偏转角
    float width,height;
    float heightCwidth ;//长宽比
    Rect rect;
    RotatedRect rotaRect;
    vector<RotatedRect> rotaRects;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src,contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE);
    vector<Rect> boundrect(contours.size());
    for(int i=0;i<contours.size();i++)
    {
        rect=boundingRect(contours[i]);
        if(rect.tl().x==0||rect.tl().y==0||rect.br().x==src.cols||rect.br().y==src.rows) continue;

        if((rect.height/rect.width>1))
        {
            rotaRect=minAreaRect(contours[i]);
		    angle=rotaRect.angle;
		    width=rotaRect.size.width;
		    height=rotaRect.size.height;	
		    heightCwidth = height / width;
           // cout<<contours.size()<<"size     !!!"<<endl;
            circle(image,Point(rect.x,rect.y),3,Scalar(0,255,0),3);
        }
    }	    

}

/*
寻找台阶边缘特征（未完）
*/
void cam::stagefinder(Mat &src,Mat &image)
{
    vector<Vec4i> lines;;//定义一个矢量结构lines用于存放得到的线段矢量集合
    Mat midImage,dstImage;//临时变量和目标图的定义
    vector<float> distance;
	Canny(src, midImage, 50, 100, 3);//进行一此canny边缘检测
	cvtColor(midImage,dstImage, CV_GRAY2BGR);//转化边缘检测后的图为灰度图
	HoughLinesP(midImage, lines, 1, CV_PI/180, 120, 60, 100 );
    distance.clear();
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186,88,255), 1, CV_AA);
        cout<<l[0]<<"   x  "<<l[1]<<"   y"<<endl;
        if((i<lines.size()-1)&&(lines[i+1][1]-l[1]>5)&&(lines[i+1][1]-l[1]<100))
        {
            distance.push_back(fabs(lines[i+1][1]-l[1]));
            circle(image,Point(lines[i+1][0],lines[i+1][1]),25,Scalar(255,0,0));
            circle(image,Point(l[0],l[1]),25,Scalar(0,255,0));
        }
	}
    cout<<lines.size()<<"   size"<<endl;
    for(int i=0;i<distance.size();i++)
    {
        cout<<distance[i]<<"dis"<<endl;
    }





    float area;//区域面积
    float angle;//偏转角
    float width,height;
    float widthCheight ;//长宽比
    Rect rect;
    RotatedRect rotaRect;
    vector<RotatedRect> rotaRects;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src,contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE);
    vector<Rect> boundrect(contours.size());
    for(int i=0;i<contours.size();i++)
    {
        rect=boundingRect(contours[i]);
        //if(rect.tl().x==0||rect.tl().y==0||rect.br().x==src.cols||rect.br().y==src.rows) continue;

      //  if((rect.width/rect.height>3))
      //  {
            rotaRect=minAreaRect(contours[i]);
		    angle=rotaRect.angle;
		    width=rotaRect.size.width;
		    height=rotaRect.size.height;	
		    widthCheight =width/height;
           // cout<<contours.size()<<"size     !!!"<<endl;
            //circle(image,Point(rotaRect.center.x,rotaRect.center.y),rotaRect.size.width,Scalar(0,0,255),5);
           // circle(image,Point(rect.x,rect.y),3,Scalar(0,255,0),3);
            //rectangle(image,rect,Scalar(255,0,0),4);
      //  }
    }	    
}

/*
剔除图像中的噪点，
1,1--联通域噪点
1,0 or 0,1--噪点
*/
void RemoveSmallRegion(Mat &Src, Mat &Dst,int AreaLimit, int CheckMode, int NeihborMode)
{
	int RemoveCount = 0;
	//新建一幅标签图像初始化为0像素点，为了记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查 
	//初始化的图像全部为0，未检查
	Mat PointLabel = Mat::zeros(Src.size(), CV_8UC1);
    Mat Pointvalue = Mat::zeros(Src.size(), CV_8UC1);
	if (CheckMode == 1)//去除小连通区域的白色点
	{
		cout << "去除小连通域.";
		for (int i = 0; i < Src.rows; i++)
		{
			for (int j = 0; j < Src.cols; j++)
			{
				if (Src.at<uchar>(i, j) < 10)
				{
					PointLabel.at<uchar>(i, j) = 3;//将背景黑色点标记为合格，像素为3
				}
			}
		}
	}
	else//去除孔洞，黑色点像素
	{
		cout << "去除孔洞";
		for (int i = 0; i < Src.rows; i++)
		{
			for (int j = 0; j < Src.cols; j++)
			{
				if (Src.at<uchar>(i, j) > 10)
				{
					PointLabel.at<uchar>(i, j) = 3;//如果原图是白色区域，标记为合格，像素为3
				}
			}
		}
	}
	vector<Point2i>NeihborPos;//将邻域压进容器
	NeihborPos.push_back(Point2i(-1, 0));
	NeihborPos.push_back(Point2i(1, 0));
	NeihborPos.push_back(Point2i(0, -1));
	NeihborPos.push_back(Point2i(0, 1));
	if (NeihborMode == 1)
	{
		cout << "Neighbor mode: 8邻域." << endl;
		NeihborPos.push_back(Point2i(-1, -1));
		NeihborPos.push_back(Point2i(-1, 1));
		NeihborPos.push_back(Point2i(1, -1));
		NeihborPos.push_back(Point2i(1, 1));
	}
	else cout << "Neighbor mode: 4邻域." << endl;
	int NeihborCount = 4 + 4 * NeihborMode;
	int CurrX = 0, CurrY = 0;
    int diff_up_y=0, diff_down_y=0;
    int diff_r_x=0, diff_l_x=0;
	//开始检测
	for (int i = 0; i < Src.rows; i++)
	{
		for (int j = 0; j < Src.cols; j++)
		{
			if (PointLabel.at<uchar>(i, j) == 0)//标签图像像素点为0，表示还未检查的不合格点
			{   //开始检查
				vector<Point2i>GrowBuffer;//记录检查像素点的个数
				GrowBuffer.push_back(Point2i(j, i));
				PointLabel.at<uchar>(i, j) = 1;//标记为正在检查
				int CheckResult = 0;
				for (int z = 0; z < GrowBuffer.size(); z++)
				{
					for (int q = 0; q < NeihborCount; q++)
					{
						CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
						CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
						if (CurrX >= 0 && CurrX<Src.cols&&CurrY >= 0 && CurrY<Src.rows)  //防止越界  
						{
							if (PointLabel.at<uchar>(CurrY, CurrX) == 0)
							{
								GrowBuffer.push_back(Point2i(CurrX, CurrY));  //邻域点加入buffer  
								PointLabel.at<uchar>(CurrY, CurrX) = 1;           //更新邻域点的检查标签，避免重复检查 
                                diff_up_y=abs(Pointvalue.at<uchar>(i, j)-Pointvalue.at<uchar>(i,j-1));
                                diff_down_y=abs(Pointvalue.at<uchar>(i, j)-Pointvalue.at<uchar>(i,j+1));
                                diff_l_x=abs(Pointvalue.at<uchar>(i, j)-Pointvalue.at<uchar>(i-1,j));
                                diff_r_x=abs(Pointvalue.at<uchar>(i, j)-Pointvalue.at<uchar>(i+1,j));
							}
						}
					}
				}
				if (GrowBuffer.size()>AreaLimit) //判断结果（是否超出限定的大小），1为未超出，2为超出  
					CheckResult = 2;
				else
				{
					CheckResult = 1;
					RemoveCount++;//记录有多少区域被去除
				}
				for (int z = 0; z < GrowBuffer.size(); z++)
				{
					CurrX = GrowBuffer.at(z).x;
					CurrY = GrowBuffer.at(z).y;
					PointLabel.at<uchar>(CurrY,CurrX)+=CheckResult;//标记不合格的像素点，像素值为2
				}
			}
		}
	}
	CheckMode = 255 * (1 - CheckMode);
	//开始反转面积过小的区域  
	for (int i = 0; i < Src.rows; ++i)
	{
		for (int j = 0; j < Src.cols; ++j)
		{
			if (PointLabel.at<uchar>(i,j)==2)
			{
				Dst.at<uchar>(i, j) = CheckMode;
			}
			else if (PointLabel.at<uchar>(i, j) == 3)
			{
				Dst.at<uchar>(i, j) = Src.at<uchar>(i, j);
				
			}
		}
	}
	cout << RemoveCount << " objects removed." << endl;
}

int main(int argc, char const *argv[])
{
    cam Cam;
	Mat i_before(480, 640, CV_8UC4);				
	Mat i_after(480, 640, CV_8UC4);				
	Mat i_result(480, 640, CV_16UC1);				
    Mat src;
    Mat src2;
    Status result = STATUS_OK;
    VideoFrameRef oniDepthImg;
    VideoFrameRef oniColorImg;
    VideoFrameRef  frameDepth;
    Mat results(480, 640,CV_8U);
    Mat cvDepthImg;
    Mat cvBGRImg;
    Mat cvFusionImg;
    char key=0;
    int count=0;
    result = OpenNI::initialize();
    Array<DeviceInfo> aDeviceList;  
    OpenNI::enumerateDevices( &aDeviceList );   
    
    cout << "         " << aDeviceList.getSize() << endl;  
    
    for( int i = 0; i < aDeviceList.getSize(); ++ i )  
    {    
        cout << "设备 " << i << endl;    
        const DeviceInfo& rDevInfo = aDeviceList[i];      
        cout << "URI: " << rDevInfo.getUri() << endl;     

    }
    CheckOpenNIError( result, "initialize context" );

    Device device;
    result = device.open( openni::ANY_DEVICE );
cout<<result<<endl;

    VideoStream oniDepthStream;
    result = oniDepthStream.create( device, openni::SENSOR_DEPTH );
getchar();
    VideoMode modeDepth;
    modeDepth.setResolution( 480, 640 );
    modeDepth.setFps( 60 );
    modeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
    oniDepthStream.setVideoMode(modeDepth);
    result = oniDepthStream.start();

    VideoStream oniColorStream;
    result = oniColorStream.create( device, openni::SENSOR_COLOR );

    VideoMode modeColor;
    modeColor.setResolution( 480, 640 );
    modeColor.setFps( 60 );
    modeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );
    oniColorStream.setVideoMode( modeColor);

    if( device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }

    result = oniColorStream.start();
    oniDepthStream.setMirroringEnabled(false);
    oniColorStream.setMirroringEnabled(false);
    while( key!=27 )
    {
        if( (oniDepthStream.readFrame( &oniDepthImg ) == STATUS_OK) && (oniColorStream.readFrame( &oniColorImg ) == STATUS_OK) )
        {
            DepthPixel* pDepth = (DepthPixel*)oniDepthImg.getData();
            src=Mat( oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData() );
            float x,y,z;
            float x0,y0,z0;
            float x1,y1,z1;
            CoordinateConverter coorvert; 
            //cout<<(void*)oniDepthImg.getData()<<endl;
            int middleIndex = (oniDepthImg.getHeight()+1)*oniDepthImg.getWidth()/2;
            //cout<<oniDepthImg.getHeight()<<"    height"<<oniDepthImg.getWidth()<<"    width"<<endl;
            coorvert.convertDepthToWorld(oniDepthStream, oniDepthImg.getWidth()/2+100, oniDepthImg.getHeight()/2, src.at<ushort>(480,220), &x, &y, &z);
            printf("[%08llu] %8d %f %f %f\n", (long long)oniDepthImg.getTimestamp(), pDepth[middleIndex], x, y, z);
            //输出中心点的世界坐标系中的x，y，z
            coorvert.convertDepthToWorld(oniDepthStream, oniDepthImg.getWidth()/2, oniDepthImg.getHeight()/2, pDepth[middleIndex], &x0, &y0, &z0);
            cout<<x0<<"  "<<y0<<"  "<< z0<<"  00"<<endl;
            coorvert.convertDepthToWorld(oniDepthStream, oniDepthImg.getWidth()/2-100, oniDepthImg.getHeight()/2, src.at<ushort>(480,420), &x1, &y1, &z1);
            cout<<x1<<"    "<<y1<<"    "<<z1<<"   "<<"111"<<endl;
            src.convertTo( src2, CV_8U, 255.0/(oniDepthStream.getMaxPixelValue()));
            //cout<<src.at<ushort>(240,320)<<"****"<<endl;
            //cout<<src.at<ushort>(240,321)<<"**@"<<endl;
            
            //将深度16通道图传如8通道图
            Cam.filter(src2);
            Cam.PixelFilter(region);
            Mat src;
            Mat dst;
            Mat fal;
            //将剔除无效区域后的区域扩大至640×480，方便处理
            resize(region,dst,Size(640,480),0,0,INTER_LANCZOS4);
            //中值滤波
            medianBlur(dst,fal,9);
            //Canny(fal,fal,20,21);
            Mat BGRImg;
            Mat RGBImg( oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData() );
            GaussianBlur(BGRImg,BGRImg,Size(5,5),5,5);
            cvtColor( RGBImg, src,CV_RGB2BGR );
            cvtColor( RGBImg, BGRImg,CV_RGB2GRAY );
            //medianBlur(BGRImg,BGRImg,7);
            //GaussianBlur(BGRImg,BGRImg,Size(5,5),3,3);
            //threshold(BGRImg,BGRImg,0,255, THRESH_BINARY+THRESH_OTSU);
            //GaussianBlur(dst,dst,Size(3,3),3,3);
            Cam.stagefinder(fal,src);
            //RemoveSmallRegion(BGRImg,BGRImg,20,0,1);

            //Canny(dst,dst,20,22);
            Canny(fal,fal,50,100,3);
            //threshold(fal,fal,5,255, THRESH_BINARY+THRESH_OTSU);
            imshow("Bgr",BGRImg);
            imshow("region",region);
            imshow("dst",dst);
            imshow("src",src);
            //imshow("src2",src2);
            imshow("fal",fal);
            waitKey(1);   
        }
        key = waitKey(20);
    }
    oniDepthStream.destroy();
    oniColorStream.destroy();
    device.close();
    OpenNI::shutdown();
    return 0;
}



