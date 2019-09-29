#include "threadcontrol.h"
#include "thread"

using namespace cv;
using namespace std;

namespace RM {

FrameBuffer::FrameBuffer(size_t size):
    _frames(size),//_frames容器大小为size
    _mutexs(size),//_mutexs容器大小为size
    _tailIdx(0),
    _headIdx(0)
{}

bool FrameBuffer::push(const cv::Mat& frame)
{
    const size_t newHeadIdx = (_headIdx + 1) % _frames.size();

    unique_lock<mutex> lock(_mutexs[newHeadIdx],try_to_lock);
    if(!lock.owns_lock())
    {
        return false;
    }
    //拿到了锁
    _frames[newHeadIdx] = frame;
    if(newHeadIdx == _tailIdx)
    {
        _tailIdx = (_tailIdx + 1) % _frames.size();
    }
    _headIdx = newHeadIdx;
    return true;
}

bool FrameBuffer::getLatest(cv::Mat& frame)
{
    const size_t headIdx = _headIdx;

    unique_lock<mutex> lock(_mutexs[headIdx],try_to_lock);
    if(!lock.owns_lock() || _frames[headIdx].empty())
    {
        return false;
    }
    //拿到了锁
    frame = _frames[headIdx];
    return true;
}

ThreadControl::ThreadControl():
    _quitFlag(false),
    startTime(0.0f),
    workDuration(0.0f),
    _buffer(6),
    _armorDetectorPtr(new ArmorDetector)
{}

void ThreadControl::init()
{
    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetectorPtr->init(armorParam);
    _armorDetectorPtr->setEnemyColor(RM::RED);
}

void ThreadControl::imageProduce()
{
#ifdef DEBUG_USED_VIDEO
    VideoCapture capture(ARMOR_VIDEO_PATH);
    while(1)
    {      
        capture >> _frame;
        if(_frame.empty())
        {
            cout << "视频已放完" << endl;
            _quitFlag = true;
            break;
        }
        _buffer.push(_frame);
        uchar KeyValue = (uchar)waitKey(1);
        if(KeyValue == 27)
        {
            _quitFlag = true;
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(32));//读入视频的帧率为30
    }
#endif

#ifdef DEBUG_USED_IMAGE
    _frame = imread(ARMOR_IMAGE_PATH);
#endif
}

void ThreadControl::imageProcess()
{
    int armorFindFlag = 0;
    this_thread::sleep_for(chrono::milliseconds(200));//等待图片或视频加载成功

    while(1)
    {
#ifdef GET_WORK_DURATION
        startTime = static_cast<float>(getTickCount());
#endif

        if(_quitFlag == true) break;

        if(!_buffer.getLatest(_srcImg)) continue;

        _armorDetectorPtr->loadImg(_srcImg);
        armorFindFlag = _armorDetectorPtr->detect();

#ifdef GET_WORK_DURATION
        workDuration =  ((float)getTickCount() - startTime)/getTickFrequency();
        cout << "处理一帧的时间 = " << workDuration*1000 << " ms" << endl;
#endif
    }
}


}
