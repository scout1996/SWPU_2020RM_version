#ifndef THREADCONTROL_H
#define THREADCONTROL_H

#include "opencv2/opencv.hpp"
#include<memory>
#include"./Armor/armordetector.h"
#include "./Pose/AngleSolver.hpp"
#include<mutex>


namespace RM {

#define ARMOR_VIDEO_PATH "/home/young-pc/SWPU_2020RoboMaster_version/SWPU_2020RoboMaster_version/TestVideo/1.avi"
#define ARMOR_IMAGE_PATH "/home/young-pc/SWPU_2020RoboMaster_version/SWPU_2020RoboMaster_version/TestImage/1.jpg"

#define DEBUG_USED_VIDEO
//#define DEBUG_USED_IMAGE

#define GET_WORK_DURATION

class FrameBuffer
{
public:
    FrameBuffer(size_t size);
    ~FrameBuffer(){}

    bool push(const cv::Mat& frame);

    bool getLatest(cv::Mat& frame);

private:
    std::vector<cv::Mat> _frames;

    std::vector<std::mutex> _mutexs;

    size_t _tailIdx;
    size_t _headIdx;
};

class ThreadControl
{
public:
    ThreadControl();
    ~ThreadControl() {}

    //初始化所有模块
    void init();

    //串口发送云台控制命令与接收STM32数据
    void serialProcess();

    //从摄像头获取图像或从文件中获取图像
    void imageProduce();

    //图像处理
    void imageProcess();

private:
    cv::Mat _frame;
    cv::Mat _srcImg;
    bool _quitFlag;

    float startTime;
    float workDuration;

    FrameBuffer _buffer;

    /* Armor detector */
    std::unique_ptr<ArmorDetector> _armorDetectorPtr;

    /* Angle solver */
    std::unique_ptr<AngleSolver> _solverPtr;
};

}

#endif // THREADCONTROL_H
