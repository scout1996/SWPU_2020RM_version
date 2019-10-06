#include "AngleSolver.hpp"
#include "cmath"

using namespace cv;
using namespace std;

namespace RM {

AngleSolverParam::AngleSolverParam()
{
    //大装甲板真实的世界坐标，把大装甲板看做一个平板，Z坐标为0，把大装甲板的中心对准定义的图像中心
    POINT_3D_OF_ARMOR_BIG = {
        cv::Point3f(-117.5f, -63.5f, 0.0f), //tl
        cv::Point3f(117.5f, -63.5f, 0.0f),	//tr
        cv::Point3f(117.5f, 63.5f, 0.0f),	//br
        cv::Point3f(-117.5f, 63.5f, 0.0f)	//bl
    };
    //如上
    POINT_3D_OF_ARMOR_SMALL = {
        cv::Point3f(-70.0f, -62.5f, 0.0f),	//tl
        cv::Point3f(70.0f, -62.5f, 0.0f),	//tr
        cv::Point3f(70.0f, 62.5f, 0.0f),    //br
        cv::Point3f(-70.0f, 62.5f, 0.0f)    //bl
    };
}

void AngleSolverParam::read_XMLFile(void)
{
    FileStorage fsRead("/home/young-pc/SWPU_2020RoboMaster_version/SWPU_2020RoboMaster_version/camera_param/camera.xml",FileStorage::READ);

    if(!fsRead.isOpened())
    {
        cout << "failed to open xml" << endl;
        return;
    }

    fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> Y_DISTANCE_BETWEEN_GUN_AND_CAM;
    fsRead["Z_DISTANCE_BETWEEN_MUZZLE_AND_CAM"] >> Z_DISTANCE_BETWEEN_MUZZLE_AND_CAM;
    fsRead["Camera_Matrix"] >> CameraIntrinsicMatrix;
    fsRead["Distortion_Coefficients"] >> DistortionCoefficient;
    return;
}

AngleSolver::AngleSolver(const AngleSolverParam& angleSolverParam)
{
    _params = angleSolverParam;
}

void AngleSolver::init(const AngleSolverParam& angleSolverParam)
{
    _params = angleSolverParam;
}

void AngleSolver::onePointSolution(const vector<Point2f> centerPoint)
{
    double fx = _params.CameraIntrinsicMatrix.at<double>(0,0);
    double fy = _params.CameraIntrinsicMatrix.at<double>(1,1);
    double cx = _params.CameraIntrinsicMatrix.at<double>(0,2);
    double cy = _params.CameraIntrinsicMatrix.at<double>(1,2);

    vector<Point2f> dstPoint;
    //单点矫正
    undistortPoints(centerPoint,dstPoint,_params.CameraIntrinsicMatrix,
                    _params.DistortionCoefficient,noArray(),_params.CameraIntrinsicMatrix);
    Point2f pnt = dstPoint.front();//返回dstPoint中的第一个元素
    //去畸变后的比值，根据像素坐标系与世界坐标系的关系得出,pnt的坐标就是在整幅图像中的坐标
    double rxNew=(pnt.x-cx)/fx;
    double ryNew=(pnt.y-cy)/fy;

    yawErr = atan(rxNew)/CV_PI*180;//转换为角度
    pitchErr = atan(ryNew)/CV_PI*180;//转换为角度
}

std::vector<double> AngleSolver::p4pSolution(const std::vector<cv::Point2f> objectPoints,int objectType)
{
    if(objectType == RM::BIG_ARMOR)
        solvePnP(_params.POINT_3D_OF_ARMOR_BIG,objectPoints,_params.CameraIntrinsicMatrix,
                 _params.DistortionCoefficient,_rVec,_tVec,false, CV_ITERATIVE);
    else if(objectType == RM::SMALL_ARMOR)
        solvePnP(_params.POINT_3D_OF_ARMOR_SMALL,objectPoints,_params.CameraIntrinsicMatrix,
                 _params.DistortionCoefficient,_rVec,_tVec,false, CV_ITERATIVE);

    _tVec.at<float>(1, 0) -= _params.Y_DISTANCE_BETWEEN_GUN_AND_CAM;
    _tVec.at<float>(2, 0) -= _params.Z_DISTANCE_BETWEEN_MUZZLE_AND_CAM;

    yawErr = atan(_tVec.at<float>(0, 0)/_tVec.at<float>(2, 0))/CV_PI*180;//转换为角度
    pitchErr = atan(_tVec.at<float>(1, 0)/_tVec.at<float>(2, 0))/CV_PI*180;//转换为角度
    //计算三维空间下的欧氏距离
    _euclideanDistance = sqrt(_tVec.at<float>(0, 0)*_tVec.at<float>(0, 0) + _tVec.at<float>(1, 0)*
                              _tVec.at<float>(1, 0) + _tVec.at<float>(2, 0)* _tVec.at<float>(2, 0));
    vector<double> result;
    result.resize(3); //指定容器的大小为3
    result[0] = yawErr;
    result[1] = pitchErr;
    result[2] =_euclideanDistance;
    return result;
}

}
