#include "opencv2/opencv.hpp"
#include "./Main/threadcontrol.h"
#include "thread"

int main()
{
    RM::ThreadControl _threadControl;

    _threadControl.init();

    std::thread imageProduceThread(&RM::ThreadControl::imageProduce,&_threadControl);
    std::thread imageProcessThread(&RM::ThreadControl::imageProcess,&_threadControl);

    imageProduceThread.join();
    imageProcessThread.join();

    return 0;
}
