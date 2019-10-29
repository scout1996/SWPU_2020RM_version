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
    _armorDetectorPtr(new ArmorDetector),
    _solverPtr(new AngleSolver)
{}

void ThreadControl::init()
{
    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetectorPtr->init(armorParam);
    _armorDetectorPtr->setEnemyColor(RM::RED);

    //Initialize angle solver
    AngleSolverParam angleParam;
    angleParam.read_XMLFile();
    _solverPtr ->init(angleParam);
}

void ThreadControl::imageProduce()
{
#ifdef DEBUG_USED_VIDEO
    //VideoCapture capture(ARMOR_VIDEO_PATH);
    VideoCapture capture(CAMERA_ID);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);//宽度
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 512);//高度，工业摄像头不支持640*480
    capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));//图像格式为mjpg，只有此格式下能到最高帧率
    capture.set(CV_CAP_PROP_FPS, 120);//帧数
    capture.set(CV_CAP_PROP_GAMMA, 300);//gamma值 工业摄像头支持gamma值范围为100-300
    capture.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.25);//改为手动曝光
    capture.set(CV_CAP_PROP_EXPOSURE,0.02); //新买的摄像头曝光值0.02较为合适

    cout<< capture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
    cout<< capture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
    cout<< capture.get(CV_CAP_PROP_FPS) << endl;
    cout<< capture.get(CV_CAP_PROP_EXPOSURE) << endl;
    cout<< capture.get(CV_CAP_PROP_GAMMA) << endl;

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
        //this_thread::sleep_for(chrono::milliseconds(32));//使读入视频的帧率为30
    }
#endif

#ifdef DEBUG_USED_IMAGE
    _frame = imread(ARMOR_IMAGE_PATH);
#endif
}

void ThreadControl::imageProcess()
{
    int armorFindFlag = 0;
    vector<Point2f> armorVertex;
    int armorType;
    vector<double> p4p_Result;

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
        waitKey(1);//**********************************************************************************************************
        if(armorFindFlag == ArmorDetector::ARMOR_LOCAL || armorFindFlag == ArmorDetector::ARMOR_GLOBAL)
        {
            armorType = _armorDetectorPtr->getArmorType();
            armorVertex = _armorDetectorPtr->getArmorVertex();
            p4p_Result = _solverPtr->p4pSolution(armorVertex,armorType);
            //cout << "yawErr= " <<p4p_Result[0]<< " pitchErr= "<<p4p_Result[1]<<" 距离= "<<p4p_Result[2]<<endl;
        }
#ifdef GET_WORK_DURATION
        workDuration =  ((float)getTickCount() - startTime)/getTickFrequency();
        cout << "处理一帧的时间 = " << workDuration*1000 << " ms" << endl;
#endif
    }
}


}
