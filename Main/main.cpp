/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @project    RM视觉组_初识装甲板
  * @brief      加入SVM降低装甲板误识别率，未加角度解算
  * @note       图像预处理过程：
  *             把图像转为HSV，分离通道，取明亮度通道的图像，然后阈值化，开操作完成预处理。
  *
  *             改进思路：
  *             1、通过qv4l2降低摄像头曝光度，增加gamma值，分离彩色通道，红蓝通道相减
  *             得到二值图，完成预处理。
  *             2、装甲板识别方法尝试用模板匹配来做，尽量提高帧率。
  * @history
  *  Version       Date            Author          status
  *  V1.0.0      2019-9-29         Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "opencv2/opencv.hpp"
#include "./Main/threadcontrol.h"
#include "thread"

int main()
{
    RM::ThreadControl _threadControl;

    //初始化所需要的模块
    _threadControl.init();

    //图像生成线程
    std::thread imageProduceThread(&RM::ThreadControl::imageProduce,&_threadControl);
    //图像处理线程
    std::thread imageProcessThread(&RM::ThreadControl::imageProcess,&_threadControl);

    //主线程等待与子线程汇合
    imageProduceThread.join();
    imageProcessThread.join();

    return 0;
}
