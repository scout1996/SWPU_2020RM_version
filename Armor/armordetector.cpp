/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       armordetector.c/h
  * @brief      装甲板检测，包括装甲板参数类，灯条描述类，装甲板描述类，装甲板检测类。
  * @note       1、全部包含在RM的命名空间下
  *             2、装甲板的寻找方式分为全图寻找或以装甲板图像2倍大的区域寻找（追踪模式），处理第一帧
  *                图像时先是全局搜索，找到装甲板后进行局部搜索（追踪模式），提高图像处理的
  *                速度，而在局部搜索处理3000张图片后，变为全图搜索，排除有潜在的高分装甲板出现
  * @history
  *  Version       Date            Author          status
  *  V1.0.0      2019-9-21         Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "armordetector.h"
#include"../General/opencv_extended.h"
#include<algorithm> //swap的头文件
#include<cmath>

using namespace cv;
using namespace std;

namespace RM {

//矫正矩形，swap互换width和height，再旋转，矩形不变
#define adjustRec(rec)       \
{                            \
    if(rec.angle >= 90.0f)   \
        rec.angle -= 180.0f; \
    else if(rec.angle < -90.0f)\
        rec.angle += 180.0f; \
    if(rec.angle >= 45.0f)   \
    {                        \
        swap(rec.size.width,rec.size.height); \
        rec.angle -= 90.0f;  \
    }                        \
    else if(rec.angle < -45.0f) \
    {                           \
        swap(rec.size.width,rec.size.height); \
        rec.angle += 90.0f;     \
    }                           \
}


ArmorDescriptor::ArmorDescriptor(const LightDescriptor& leftLight, const LightDescriptor& rightLight, const int type, const cv::Mat& roiImg,const float rotaScore, ArmorParam armorParam)
{
    _param = armorParam;

    lightPairs[0] = leftLight.rect();
    lightPairs[1] = rightLight.rect();
    //把灯条的高度扩大两倍，因为灯条高度本来就比装甲板的高度短，扩大两倍，使得可以描述装甲板的高度
    Size e_LeftLightSize(int(lightPairs[0].size.width),int(lightPairs[0].size.height*2));
    Size e_RightLightSize(int(lightPairs[1].size.width),int(lightPairs[1].size.height*2));

    RotatedRect extendLeftLightRec(lightPairs[0].center,e_LeftLightSize,lightPairs[0].angle);
    RotatedRect extendRightLightRec(lightPairs[1].center,e_RightLightSize,lightPairs[1].angle);

    //获得装甲板的四个顶点坐标
    Point2f _leftLightVertex[4];
    extendLeftLightRec.points(_leftLightVertex);
    Point2f _rightLightVertex[4];
    extendRightLightRec.points(_rightLightVertex);
    vertex[0] = _leftLightVertex[0];
    vertex[1] = _rightLightVertex[1];
    vertex[2] = _rightLightVertex[2];
    vertex[3] = _leftLightVertex[3];

    armorType = type;

    Rect digitalRect;
    Mat digitalImg;
    if(armorType == BIG_ARMOR)
    {
        //保留大装甲板的数字区域
        digitalRect = Rect(Point2f(vertex[0].x+(vertex[1].x-vertex[0].x)/4,vertex[0].y),Size((vertex[1].x-vertex[0].x)/2,extendLeftLightRec.size.height));
        digitalImg = roiImg(digitalRect);
    }
    else if(armorType == SMALL_ARMOR)
    {
        //保留小装甲板的数字区域
        digitalRect = Rect(_leftLightVertex[1],Size(_rightLightVertex[0].x-_leftLightVertex[1].x,extendLeftLightRec.size.height));
        digitalImg = roiImg(digitalRect);
    }
    //得到数字区域的正视图
    getFrontImg(digitalImg);

    // 计算旋转分数
    rotationScore = rotaScore;
    // 计算尺寸分数
    float normalized_area = contourArea(vertex) / _param.area_normalized_base;
    sizeScore = exp(normalized_area);
    // 计算距离分数（目标中心与图像中心的距离）
    Point2f srcImgCenter(roiImg.cols / 2.0f, roiImg.rows / 2.0f);
    float distanceOffset = cvex::distance(srcImgCenter, cvex::crossPointOf(array<Point2f, 2>{vertex[0],vertex[2]}, array<Point2f, 2>{vertex[1],vertex[3]}));
    distanceScore = exp(-distanceOffset / _param.distance_offset_normalized_base);
}

//要先用上面的构造函数完成初始化，_param才有值，不然该程序里面要出错
void ArmorDescriptor::getFrontImg(const Mat& digitalImg)
{
    int width, height;
    if(armorType == BIG_ARMOR)
    {
        width = _param.big_armor_width;
        height = _param.big_armor_height;
    }
    else if(armorType == SMALL_ARMOR)
    {
        width = _param.small_armor_width;
        height = _param.small_armor_height;
    }

    Point2f src[4]={vertex[0],vertex[1],vertex[2],vertex[3]};
    Point2f dis[4]={Point2f(0.0f,0.0f),Point2f(width,0.0f),Point2f(width,height),Point2f(0.0f,height)};
    Mat _perspectiveTransformationMatrix = getPerspectiveTransform(src,dis);//得到透视变换矩阵
    warpPerspective(digitalImg,frontImg,_perspectiveTransformationMatrix,Size(width,height));//进行透视变换
}

bool ArmorDescriptor::isArmorPattern() const
{
    Mat regulatedImg,imageNewSize;
    frontImg.copyTo(regulatedImg);
    resize(regulatedImg,regulatedImg,SVM_SAMPLE_SIZE); //统一训练集尺寸
    imageNewSize = regulatedImg.reshape(0,1);//图像深度不变，把图片矩阵转为一行储存
    imageNewSize.convertTo(imageNewSize,CV_32FC1);

    Ptr<ml::SVM> svmClassifier = ml::StatModel::load<ml::SVM>(SVM_TRAINING_MODEL_PATH);
    int result = static_cast<int>(svmClassifier->predict(imageNewSize));
    if(result == 1) return true;
    else
        return false;
}

ArmorDetector::ArmorDetector(const ArmorParam& armorParam)
{
    _param = armorParam;
    _armorFindFlag = ARMOR_NO;
    _roi = Rect(cv::Point(0, 0), _srcImg.size());
    _isTracking = false;
}

//初始化装甲板参数
void ArmorDetector::init(const ArmorParam& armorParam)
{
    _param = armorParam;
}

void ArmorDetector::loadImg(const cv::Mat&  srcImg)
{
    _srcImg = srcImg;
    Rect srcImgRect = Rect(Point(0,0),_srcImg.size());

    if(_armorFindFlag == ARMOR_LOCAL && _trackCounter != _param.max_track_num)//在追踪模式下，连续处理3000张图片后，进行全局检测一次
    {
        Rect tempRect = boundingRect(_targetArmor.vertex);
        tempRect = cvex::scaleRect(tempRect, Vec2f(2,2));	//以中心为锚点，长宽各放大2倍
        _roi = tempRect & srcImgRect;//防止tempRect越界
        _roiImg = _srcImg(_roi).clone();
    }
    else
    {
        _roi = srcImgRect;
        _roiImg = _srcImg.clone();
        _trackCounter = 0;
    }
}

int ArmorDetector::detect()
{
    vector<Mat> channels;
    Mat imageBlueChannel;
    Mat imageRedChannel;

    _armors.clear();
    vector<LightDescriptor> lightInformations;

    split(_roiImg,channels); //把一个3通道图像转换成3个单通道图像
    imageBlueChannel = channels.at(0);
    imageRedChannel = channels.at(2);

    //预处理删除己方装甲板颜色
    if(_enemyColor == RED)
        _grayImg = imageRedChannel - imageBlueChannel;
    else if(_enemyColor == BLUE)
        _grayImg = imageBlueChannel - imageRedChannel;

    Mat binaryImg;
    //阈值化
    threshold(_grayImg,binaryImg,_param.brightness_threshold,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    //形态学滤波：开运算（消除小物体）
    morphologyEx(binaryImg,binaryImg,MORPH_OPEN,element);

    vector<vector<Point>> lightContours;
    vector<Vec4i> hierarchy;

    findContours(binaryImg,lightContours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

    for(const auto& contour : lightContours)
    {
        //得到面积
        double lightContourArea = contourArea(contour);
        //轮廓点数不够或面积太小的不要
        if(contour.size()<=_param.light_min_size || lightContourArea < _param.light_min_area) continue;
        //椭圆拟合区域得到外接旋转矩形
        RotatedRect lightRec = fitEllipse(contour);
        adjustRec(lightRec);

        if(lightRec.size.width > lightRec.size.height) continue;

        //因为颜色通道相减后己方灯条直接过滤，不需要判断颜色了,可以直接将灯条保存
        lightInformations.emplace_back(LightDescriptor(lightRec));
    }
    if(lightInformations.empty()) return _armorFindFlag = ARMOR_NO;
    //将灯条从左到右排序，区别装甲板的左右灯条
    sort(lightInformations.begin(),lightInformations.end(),compareLight);

    for(size_t i = 0;i<lightInformations.size();i++)
    {
        for(size_t j = i+1;j<lightInformations.size();j++)
        {
            const LightDescriptor& leftLight = lightInformations[i];
            const LightDescriptor& rightLight = lightInformations[j];
            //角差
            float angleDiff = fabs(leftLight.angle - rightLight.angle);
            //高度差比率
            float heightDiffRatio = fabs(leftLight.height - rightLight.height) / max(leftLight.height, rightLight.height);
            //筛选
            if(angleDiff > _param.light_max_angle_diff || heightDiffRatio>_param.light_max_height_diff_ratio) continue;
            //左右灯条相距距离
            float distance = cvex::distance(leftLight.center,rightLight.center);
            float averageHeight = (leftLight.height + rightLight.height)/2.0f;
            //左右灯条中心点y的差值
            float yDiff = fabs(leftLight.center.y-rightLight.center.y);
            //y差比率
            float yDiffRatio = yDiff/averageHeight;
            //相距距离与灯条高度比值
            float disRatio = distance/averageHeight;
            //筛选
            if(yDiffRatio >_param.light_max_y_diff_ratio_ || disRatio>_param.armor_max_aspect_ratio_
                    || disRatio<_param.armor_min_aspect_ratio_) continue;

            //按比值来确定大小装甲
            int armorType = disRatio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
            //计算旋转得分,yDiff差的大，旋转角度越大
            float rotationScore = -(yDiffRatio * yDiffRatio);
            //得到匹配的装甲板,_roiImg为原始图像640*480，或以上一帧最优装甲板的中心位置扩大装甲板的两倍区域
            ArmorDescriptor armor(leftLight, rightLight, armorType,_roiImg, rotationScore, _param);

            _armors.emplace_back(armor);
        }
    }
    if(_armors.empty()) return _armorFindFlag = ARMOR_NO;
    //排除虚假的装甲板
    _armors.erase(remove_if(_armors.begin(), _armors.end(), [this](ArmorDescriptor i)
       {//isArmorPattern函数判断是不是装甲板，将装甲板中心的图片提取后让识别函数去识别，识别可以用svm或者模板匹配等
           return false == (i.isArmorPattern());
    }), _armors.end());

    if(_armors.empty())
    {
        _targetArmor.informationClear();

        if(_armorFindFlag == ARMOR_LOCAL)
            return _armorFindFlag = ARMOR_LOST;
        else
            return _armorFindFlag = ARMOR_NO;
    }
    //计算最终得分
    for(auto & armor : _armors)
    {
        armor.finalScore = armor.sizeScore + armor.distanceScore + armor.rotationScore;
    }
    //选择出一个最高得分的矩阵给_targetArmor
    sort(_armors.begin(), _armors.end(), compareArmor);
    _targetArmor = _armors[0];

    //更新在追踪模式下的处理次数，达到3000，变为整幅检测
    _trackCounter++;

    return _armorFindFlag = ARMOR_LOCAL;
}


}