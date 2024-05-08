#include "yaw_from_lidar/lowpassfilter.h"

class Lidar
{
    public:
        ros::NodeHandle nh_lidar;
        ros::Subscriber sub_lidar;

        std::mutex mtx_lidar;

        Lowpassfilter LPF;

        bool isInitial;
        bool debugMode;

        int m_to_pixel;

        double period;
        double prevTime;

        std::vector<double> prevResult;
        std::vector<double> prevData;

        void subscribe_lidar();
        void lidar_callback(const sensor_msgs::LaserScanConstPtr& _msg);
        void first_loop(const sensor_msgs::LaserScan& __point, cv::Mat& __img, cv::Point& __center);
        void filteredPoints(const sensor_msgs::LaserScan& __point, const std::vector<double>& __prevPoint, const std::vector<double>& __prevResult, cv::Mat& __img, cv::Point& __center, const double __samplingFreq);

    Lidar()
    {
        debugMode = false;
        isInitial = true;
        m_to_pixel = 500;
    }
    ~Lidar(){}
};

void Lidar::subscribe_lidar()
{
    sub_lidar = nh_lidar.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Lidar::lidar_callback, this);
}

void Lidar::lidar_callback(const sensor_msgs::LaserScanConstPtr& _msg)
{
    sensor_msgs::LaserScan scan;

    mtx_lidar.lock();

    scan.angle_increment    = _msg->angle_increment;
    scan.angle_max          = _msg->angle_max;
    scan.angle_min          = _msg->angle_min;
    scan.header             = _msg->header;
    scan.range_max          = _msg->range_max;
    scan.range_min          = _msg->range_min;
    scan.ranges             = _msg->ranges;
    scan.scan_time          = _msg->scan_time;
    
    mtx_lidar.unlock();

    double samplingFreq = 1 / scan.scan_time;

    cv::Mat img(640, 480, CV_8UC3, cv::Scalar(127, 127, 127));
    cv::Point center(img.cols / 2 - 1, img.rows / 2 - 1);

    std::vector<double> pointsVec;
    std::vector<double> tempPrevResult;
    std::vector<double> tempPrevData;

    tempPrevResult.clear();
    tempPrevData.clear();

    std::copy(prevResult.begin(), prevResult.end(), tempPrevResult.begin());
    std::copy(prevData.begin(), prevData.end(), tempPrevData.begin());

    prevResult.clear();
    prevData.clear();

    //tempPrevResult.swap(prevResult);
    //tempPrevData.swap(prevData);

    if(isInitial)
    {
        if(debugMode)
            ROS_INFO("first loop");
        first_loop(scan, img, center);
        isInitial = false;
    }
    else
    {
        if(debugMode)
            ROS_INFO("filtered point");
        filteredPoints(scan, tempPrevData, tempPrevResult, img, center, samplingFreq);
    }

    cv::namedWindow("Laser Scan");
    cv::imshow("Laser Scan", img);
    cv::waitKey(1);
}

void Lidar::first_loop(const sensor_msgs::LaserScan& __point, cv::Mat& __img, cv::Point& __center)
{
    for(int i = 0; i < __point.ranges.size(); i++)
    {
        double ang;
        double pointX, pointY;

        prevData.push_back(__point.ranges[i]);
        prevResult.push_back(__point.ranges[i]);

        ROS_INFO("STEP 1");
        ROS_INFO("__point.ranges.size() = %ld", __point.ranges.size());
        if(std::isnan(__point.ranges[i]))
            continue;
        ROS_INFO("STEP 1_1");
        if(std::isinf(__point.ranges[i]))
            continue;
        ROS_INFO("STEP 1_2");
        if(__point.ranges[i] == 0.0)
        {
            ROS_INFO("check zero");
            continue;
        }

        ROS_INFO("STEP 2");
        ang = __point.angle_min + __point.angle_increment * i;

        ROS_INFO("STEP 3");
        if(__point.angle_min <= ang && ang <= __point.angle_max)
        {
            pointX = __center.x - std::round(std::sin(ang) * __point.ranges[i] * m_to_pixel);
            pointY = __center.y - std::round(std::cos(ang) * __point.ranges[i] * m_to_pixel);
        }

        cv::Point pt(pointX, pointY);

        cv::circle(__img, pt, 3, cv::Scalar(0, 0, 0), 2);
        cv::line(__img, __center, pt, cv::Scalar(255, 255, 255), 2);
    }
}

void Lidar::filteredPoints(const sensor_msgs::LaserScan& __point, const std::vector<double>& __prevPoint, const std::vector<double>& __prevResult, cv::Mat& __img, cv::Point& __center, const double __samplingFreq)
{
    if(debugMode)
        ROS_INFO("Lowpass Filter");
    std::vector<double> resultVec = LPF.lowpassFilter(__samplingFreq, __prevPoint, __prevResult, __point);

    for(int i = 0; i < resultVec.size(); i++)
    {
        prevData.push_back(__point.ranges[i]);
        prevResult.push_back(resultVec[i]);

        ROS_INFO("STEP 4_1");
        if(std::isnan(resultVec[i]))
            continue;
        ROS_INFO("STEP 4_2");
        if(std::isinf(resultVec[i]))
            continue;
        ROS_INFO("STEP 4_3");
        if(resultVec[i] == 0.0)
            continue;

        double ang;
        double pointX, pointY;

        ang = __point.angle_min + __point.angle_increment * i;

        ROS_INFO("STEP 5");
        if(__point.angle_min <= ang && ang <= __point.angle_max)
        {
            pointX = __center.x - std::round(std::sin(ang) * resultVec[i] * m_to_pixel);
            pointY = __center.y - std::round(std::cos(ang) * resultVec[i] * m_to_pixel);

            cv::Point pt(pointX, pointY);

            cv::circle(__img, pt, 3, cv::Scalar(0, 0, 0), 2);
            cv::line(__img, __center, pt, cv::Scalar(255, 255, 255), 2);
        }
    }
}