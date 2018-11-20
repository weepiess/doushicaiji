#ifndef AIM_PARAMS_H
#define AIM_PARAMS_H

class AimParams{
public:
    AimParams(){}
    ~AimParams(){}

public:
    //与图像的处理有关
    int substract_thresh;           //图像两个对应通道的阈值
    int channel_thresh;             //对应颜色的通道阈值

    //用于灯条识别
    unsigned char enemy_color;      //敌方颜色
    float max_allow_angle;          //最大允许角度，超过该角度不视作灯条
    float max_diff_angle;           //拟合灯条匹配允许的最大角度差
    float min_inside_angle;         //两个灯条组成的四边形最小的内角
    float max_height_ratio;         //两个灯条最大的高度比例
    float max_dis_height_ratio;     //灯条之间距离与灯条高度比值的最大值
    float min_dis_height_ratio;     //最小值
    int near_lamp_height;           //近处灯条的判断值，高于该值就判断为近处灯条
    int far_lamp_height;            //远处灯条判断值
    int mid_lamp_height;            //中等距离灯条

    //与ROI有关，用在灯条已经更新了，但是roi还未更新的时候，判断之前的灯条在什么位置
    int near_roi_width;             //高于该值视为装甲板在近处
    int near_roi_height;            //同上
    int far_roi_height;             //低于该值视为装甲板在远处

    //与图片视频相关
    int img_width;                  //显示图片的宽
    int img_height;                 //显示图片的高
};

#endif