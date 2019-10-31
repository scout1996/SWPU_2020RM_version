#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include<opencv2/opencv.hpp>


namespace RM {


#define SVM_SAMPLE_SIZE Size(50,50)

#define SVM_TRAINING_MODEL_PATH "/home/young-pc/SWPU_2020RoboMaster_version/SWPU_2020RoboMaster_version/SVM_Model/svm.xml"

#define SHOW_RESULT
//#define DEBUG_DETECTION
#define USED_CAMERA

enum RobotType
{
    Hero = 1,      //英雄
    Engineer = 2,  //工程
    Standard = 3,  //步兵
    Sentry =4,     //哨兵
    Aerial = 5     //无人机
};

enum ColorChannels
{
    BLUE  = 0,
    GREEN = 1,
    RED   = 2
};

//目标类型
enum ObjectType
{
    UNKNOWN_ARMOR = 0,
    SMALL_ARMOR   = 1,
    BIG_ARMOR     = 2,
    SMALL_BUFF    = 3,
    BIG_BUFF      = 4
};

//该结构体存储关于装甲检测的所有参数
struct ArmorParam
{
    //Pre-treatment
    int brightness_threshold;
    int color_threshold;
    float light_color_detect_extend_ratio;

    //Filter lights
    float light_min_area;
    float light_max_angle;
    float light_min_size;
    float light_contour_min_solidity;
    float light_max_ratio;

    //Filter pairs
    float light_max_angle_diff;
    float light_max_height_diff_ratio; // hdiff / max(r.length, l.length)
    float light_max_y_diff_ratio_;  // ydiff / max(r.length, l.length)
    float light_min_x_diff_ratio_;

    //Filter armor
    float armor_big_armor_ratio;
    float armor_small_armor_ratio;
    float armor_min_aspect_ratio_;
    float armor_max_aspect_ratio_;

    //装甲板尺寸mm
    int big_armor_width;
    int big_armor_height;
    int small_armor_width;
    int small_armor_height;

    //other params
    float distance_offset_normalized_base;
    float area_normalized_base;
    int enemy_color;
    int max_track_num;

    //为各项参数赋默认值
    ArmorParam()
    {
        //pre-treatment
        brightness_threshold = 250;
        color_threshold = 40;
        light_color_detect_extend_ratio = 1.1f;

        // Filter lights
        light_min_area = 10.0f; //10.0
        light_max_angle = 45.0f;
        light_min_size = 5.0f;  //5.0
        light_contour_min_solidity = 0.5f;
        light_max_ratio = 1.0f;

        // Filter pairs
        light_max_angle_diff = 7.0f; //20
        light_max_height_diff_ratio = 0.4f; //0.4
        light_max_y_diff_ratio_ = 1.5f; //100
        light_min_x_diff_ratio_ = 0.5f; //100

        // Filter armor
        armor_big_armor_ratio = 3.2f;
        armor_small_armor_ratio = 2.0f;
        armor_min_aspect_ratio_ = 1.0f;
        armor_max_aspect_ratio_ = 5.0f;

        //装甲板尺寸mm
        big_armor_width = 100;
        big_armor_height = 60;
        small_armor_width =50;
        small_armor_height = 50;

        //other params
        distance_offset_normalized_base = 200.0f;
        area_normalized_base = 1000.0f;
        enemy_color = BLUE;
        max_track_num = 3000;
    }
};

//关于灯条信息的描述
class LightDescriptor
{
public:
    LightDescriptor(const cv::RotatedRect& light)
    {
        width = light.size.width;
        height= light.size.height;
        angle = light.angle;
        area  = light.size.area();
        center= light.center;
    }

    const LightDescriptor& operator =(const LightDescriptor& another)
    {
        this->width = another.width;
        this->height= another.height;
        this->angle = another.angle;
        this->area  = another.area;
        this->center= another.center;
        return *this;
    }

    //传入灯条对象，返回一个RotatedRect,const修饰this指针
    cv::RotatedRect rect() const
    {
        return cv::RotatedRect(center,cv::Size2f(width,height),angle);
    }

public:
    cv::Point2f center;
    float width;
    float height;
    float angle;
    float area;
};

class ArmorDescriptor
{
public:

    //无参构造函数
    ArmorDescriptor();

    //有参构造函数
    ArmorDescriptor(const LightDescriptor& leftLight, const LightDescriptor& rightLight, const int type, const cv::Mat& roiImg,const float rotationScore, ArmorParam armorParam);

    //从原图像四个顶点经过透视变换后得到正视图
    bool getFrontImg(const cv::Rect& digitalRect,const cv::Mat& roiImg);

    //利用SVM判断中心图案是否为数字，是不是装甲板
    bool isArmorPattern() const;

    //清除装甲板数据
    void informationClear();

public:
    std::array<cv::RotatedRect,2> lightPairs;//0 left light, 1 right light
    float sizeScore;    //尺寸得分
    float distanceScore;//距离得分
    float rotationScore;//旋转得分
    float finalScore;   //最终得分 判断当前画面中谁是最优位置的装甲板  
    cv::Mat frontImg;//从原图像四个顶点经过透视变换后的正视图
    int armorType;
    ArmorParam _param;
    std::vector<cv::Point2f> vertex;//装甲板的四个顶点
    std::string pictureName;
};

class ArmorDetector
{
public:

    enum ArmorFindFlag
    {
        ARMOR_NO = 0,		// not found
        ARMOR_LOST = 1,		// lose tracking
        ARMOR_GLOBAL = 2,	// armor found globally
        ARMOR_LOCAL = 3		// armor found locally(in tracking mode)
    };

    //无参构造函数
    ArmorDetector()
    {
        _armorFindFlag = ARMOR_NO;
        _roi = cv::Rect(cv::Point(0, 0), _srcImg.size());
    }

    //有参构造函数
    ArmorDetector(const ArmorParam& armorParam);

    //初始化装甲板参数
    void init(const ArmorParam& armorParam);

    //设置敌人的颜色
    void setEnemyColor(int enemy_color)
    {
        _enemyColor = enemy_color;
        _selfColor = enemy_color == BLUE ? RED : BLUE;
    }

    //加载图像并设置追踪区域
    void loadImg(const cv::Mat& srcImg);

    //装甲板检测
    int detect();

    //返回装甲板的四个顶点坐标，const修饰this指针
    const std::vector<cv::Point2f> getArmorVertex() const;

    //返回装甲板类型
    int getArmorType() const;

private:
    int _enemyColor;
    int _selfColor;
    int _armorFindFlag;
    bool _isTracking;  //判断是否在追踪模式下
    int _trackCounter = 0; //记录在追踪模式下处理图片的张数，达到max_track_num后变为全局搜索模式
    ArmorDescriptor _targetArmor;
    ArmorParam _param;
    cv::Rect _roi;
    cv::Mat _srcImg;
    cv::Mat _roiImg;
    cv::Mat _grayImg;

    std::vector<ArmorDescriptor> _armors;

private:
    //sort函数的回调函数，按照下列方法排序
    static bool compareLight(const LightDescriptor ld1, const LightDescriptor ld2)
    {
        return ld1.center.x < ld2.center.x;
    }
    static bool compareArmor(const ArmorDescriptor a, const ArmorDescriptor b)
    {
        return a.finalScore > b.finalScore;
    }
};


}

#endif // ARMORDETECTOR_H
