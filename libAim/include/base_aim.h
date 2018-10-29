////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2019      Base aim for all aim
///ALL RIGHTS RESERVED
///@file:base_aim.h
///@brief: 瞄准基类头文件，封装基本瞄准函数。
///@vesion 1.0
///@author: wyx
///@email: 1418555317@qq.com
///@date: 2018.10.27
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#ifndef BASE_AIM_H
#define BASE_AIM_H

#include <opencv2/opencv.hpp>
#include <pnp_solver.h>
#include <image_tool.h>
#include <iostream>

using namespace cv;
using namespace std;

//该类作为所有瞄准的父类，不能被实例化，只能继承
class BaseAim{

public:
    //aim()返回的结果标志位
    typedef enum: unsigned char{
        AIM_TARGET_FOUND,
        AIM_TARGET_NOT_FOUND,
        AIM_IMAGE_ERROR
    }AimResult;

    /** 设置屏幕的宽高
     * @param: winSize, 新屏幕宽高
     */
    void setImgSize(Size imgSize);

    /** 在实例化时如果子类需要使用预测值，则调用此方法开启预测
     * @return: void
     */
    void openPredict();

    /** 设置敌方颜色
     * @param: enemyColor, 敌方颜色，只能用类似BaseAim::color_red去调用，不需要颜色则不调用
     * @return: void
     */
    void setEnemyColor(int enemyColor);

    /** 设置预测下一帧所要使用的延时
     * @param: timeDelay, 延时
     */
    void setTimeDelay(double timeDelay);

public:
    //敌方颜色，调用时只能使用变量名，不能直接使用0或１
    const static int color_red = 0;
    const static int color_blue = 1;

protected:
    /** 根据pnp解算的tvec矩阵计算需要移动的pitch和yaw轴角度，默认使用迭代法
     * 该方法的调用需要确保在Points3D和Points2D都已经初始化完成以后，直接调用此方法就可获取偏移角度
     * @param: x_offset，tvec返回的x需要手动添加的增量
     * @param: currPitch，当前pitch角度
     * @param: currYaw，当前yaw角度
     * @param: flags, 使用的解算方法
     * @return: Point2f，计算出来的转移角度
     */
    Point2f calPitchAndYaw(float x_offset, float y_offset, float z_offset, float currPitch, float currYaw, 
                            bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE);

    /** 该函数与上面的区别是内部没有封装pnp解算的步骤，传入的值是tvec的偏移量和手动调整的参数offset
     * 调用该函数前需要调用pnpSolver.getTvec()获取tvec矩阵，然后传入相应的值就可获取偏移角度
     * @param: x, 直接传入tvec.x
     * @param: x_offset，tvec返回的x需要手动添加的增量
     * @param: currPitch，当前pitch角度
     * @param: currYaw，当前yaw角度
     * @return: Point2f，计算出来的转移角度 
     */
    Point2f calPitchAndYaw(float x, float y, float z, float x_offset, float y_offset, float z_offset, float currPitch, float currYaw);

    /** 纠正矩形拟合后矩形的宽不一定比长短的情况
     * @param: rect, 需要进行变换的旋转矩形，一般为矩形拟合后调用
     * @return: RotatedRect, 变换后的旋转矩形，长一定比宽长
     */
    RotatedRect adjustRRect(const RotatedRect & rect);

    /** 两个旋转矩形顶点依次相连构成四边形
     * @param: left, 左边的旋转矩形
     * @param: right, 右边的旋转矩形
     * @return: RotatedRect, 以两个矩形为对边的旋转矩形
     */
    RotatedRect boundingRRect(const RotatedRect &left, const RotatedRect &right);

    /** 调整rect的大小，使得在接触边界时能够自动贴边
     * @param: rect, 需要调整的rect
     * @return: bool, 如果调整到rect.width<0或者rect.height<0则返回false, 否则返回true
     */
    bool makeRectSafe(Rect & rect);

    /** 扩大rect
     * @param: rect, 需要扩大的rect
     * @return: bool, 内部调用makeRectSafe(Rect &rect)返回是否还能调整大小
     */
    bool broadenRect(Rect & rect);

protected:
    //每一个子类中都默认拥有，方便调用pnp解算
    PNPSolver pnpSolver;

    //是否使用预测值，默认为false，对于步兵等自瞄可能需要使用预测值，对于吊射等应该不需要
    bool isPredict = false;

    //目标的颜色，用于分离通道等操作，默认为红色
    int enemyColor = color_red;

    //用在预测过程中，用于通过时延计算下一帧预测位置
    double timeDelay = 0;

    //图像的宽高
    int IMG_WIDTH;
    int IMG_HEIGHT;
};

#endif