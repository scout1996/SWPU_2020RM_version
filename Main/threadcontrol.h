#ifndef THREADCONTROL_H
#define THREADCONTROL_H

#include "opencv2/opencv.hpp"

#define ARMOR_VIDEO_PATH "../TestVideo/1.avi"

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
};

#endif // THREADCONTROL_H
