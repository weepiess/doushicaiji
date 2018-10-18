bool resize(Rect &origin, Rect &current)
{   
    //记录调用resize的次数
    if(resize_count==3)
    {
 	resize_count=0;//清零
        return false;
        
    }	
    
    //将ROI区域扩大
    current.x=origin.x-origin.height/2;
    current.y=origin.y-origin.width/2;
    current.height=origin.height+origin.height;
    current.width=origin.width+origin.width;
    
    //判断ROI是否越界
    if(img_width-1-rect.x<rect.width||img_height-1-rect.y<rect.height)
        return false;
    resize_count++;
}



//使用到的参数
//-----------------------------------------
int counts=0;
int resize_count=0;
int img_width;
int img_height;
