#include "threadcontrol.h"

using namespace cv;
using namespace std;

void ThreadControl::imageProduce()
{
    VideoCapture capture(ARMOR_VIDEO_PATH);
    while(1)
    {
        capture >> _frame;
        if(_frame.empty())
        {
            cout << "视频已放完" << endl;
            break;
        }
    }
}

void ThreadControl::imageProcess()
{
    while(1)
    {
        if(_frame.empty()) continue;

    }

}
