#pragma once

#include "opencv2/opencv.hpp"
#include"./Armor/armordetector.h"

namespace RM {

struct AngleSolverParam
{
    cv::Mat CameraIntrinsicMatrix; //相机内参矩阵
    cv::Mat DistortionCoefficient; //相机畸变系数
    //单位为mm
    std::vector<cv::Point3f> POINT_3D_OF_ARMOR_BIG;
    std::vector<cv::Point3f> POINT_3D_OF_ARMOR_SMALL;
    float Y_DISTANCE_BETWEEN_GUN_AND_CAM;//如果摄像头在枪管的上面，这个变量为正
    float Z_DISTANCE_BETWEEN_MUZZLE_AND_CAM;//如果摄像头在枪口的后面，这个变量为正

    AngleSolverParam();
    //读取摄像头参数
    void read_XMLFile(void);
};

class AngleSolver
{
public:
    AngleSolver(){}
    AngleSolver(const AngleSolverParam& angleSolverParam);

    //初始化角度解算的参数
    void init(const AngleSolverParam& angleSolverParam);

    //利用小孔成像原理进行单点解算，只能得到相对于摄像头中心的转角,没有深度信息，仅需一个点
    void onePointSolution(const std::vector<cv::Point2f> centerPoint);

    //利用solvePNP进行位置解算，包含深度信息,可以自定义中心,需要四个点
    std::vector<double> p4pSolution(const std::vector<cv::Point2f> objectPoints,int objectType);

    //重力补偿
    void compensateGravity();

private:
    AngleSolverParam _params;

    cv::Mat _rVec = cv::Mat::zeros(3, 1, CV_32FC1);//旋转向量
    cv::Mat _tVec = cv::Mat::zeros(3, 1, CV_32FC1);//平移矩阵

    double _euclideanDistance;//x坐标下的差值，y坐标下的差值，欧氏距离
    double yawErr,pitchErr; //yaw轴的误差，pitch轴的误差
};


}
