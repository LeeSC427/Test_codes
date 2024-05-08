#include "yaw_from_lidar/lowpassfilter.h"

class LidarEst
{
    public:
        ros::NodeHandle nh_lidar;
        ros::Subscriber sub_lidar;

        std::mutex mtx_lidar;

        Lowpassfilter LPF;

        bool isInitial;
        bool debugMode;

        int m_to_pixel;

        std::vector<float> prevResult;
        std::vector<float> prevData;

        void subscribe_lidar();
        void lidar_callback(const sensor_msgs::LaserScanConstPtr& _msg);
        void first_loop(const sensor_msgs::LaserScan& __point, cv::Mat& __img, cv::Point& __center);
        void filteredPoints(const sensor_msgs::LaserScan& __point, const std::vector<float>& __prevPoint, const std::vector<float>& __prevResult, cv::Mat& __img, cv::Point& __center, const double __samplingFreq);

    LidarEst(){
        isInitial = false;
        debugMode = true;
        m_to_pixel = 500;
    }
    ~LidarEst(){}
};

void LidarEst::subscribe_lidar()
{
    sub_lidar = nh_lidar.subscribe<sensor_msgs::LaserScan>("/scan", 10, &LidarEst::lidar_callback, this);
}

void LidarEst::lidar_callback(const sensor_msgs::LaserScanConstPtr& _msg)
{
    sensor_msgs::LaserScan scanData;

    mtx_lidar.lock();

    scanData.angle_increment    = _msg->angle_increment;
    scanData.angle_max          = _msg->angle_max;
    scanData.angle_min          = _msg->angle_min;
    scanData.header             = _msg->header;
    scanData.range_max          = _msg->range_max;
    scanData.range_min          = _msg->range_min;
    scanData.ranges             = _msg->ranges;
    scanData.scan_time          = _msg->scan_time;

    mtx_lidar.unlock();

    double samplingFreq = 1/ scanData.scan_time;

    cv::Mat img(640, 480, CV_8UC3, cv::Scalar(127, 127, 127));
    cv::Point center(img.cols / 2 - 1, img.rows / 2 - 1);

    std::vector<float> tempPrevResult;
    std::vector<float> tempPrevData;

    //tempPrevResult.clear();
    //tempPrevData.clear();

    tempPrevResult = prevResult;
    tempPrevData = prevData;

    prevResult.clear();
    prevData.clear();

    if(isInitial)
    {
        first_loop(scanData, img, center);
        isInitial = false;
    }
    else
    {
        filteredPoints(scanData, tempPrevData, tempPrevResult, img, center, samplingFreq);
    }

    cv::namedWindow("Laser Scan");
    cv::imshow("Laser Scan", img);
    cv::waitKey(1);
}

void LidarEst::first_loop(const sensor_msgs::LaserScan& __point, cv::Mat& __img, cv::Point& __center)
{
    ROS_INFO("in first_loop point size: %d", __point.ranges.size());
    for(int i = 0; i < (int)__point.ranges.size(); i++)
    {
        double ang;
        double pointX, pointY;

        prevData.push_back(__point.ranges[i]);
        prevResult.push_back(__point.ranges[i]);

        ang = __point.angle_min + __point.angle_increment * i;

        pointX = __center.x - std::round(std::sin(ang) * __point.ranges[i] * m_to_pixel);
        pointY = __center.y - std::round(std::cos(ang) * __point.ranges[i] * m_to_pixel);

        cv::Point pt(pointX, pointY);
        cv::circle(__img, pt, 3, cv::Scalar(0, 0, 0), 2);
        cv::line(__img, __center, pt, cv::Scalar(255, 255, 255), 2);
    }
}

void LidarEst::filteredPoints(const sensor_msgs::LaserScan& __point, const std::vector<float>& __prevPoint, const std::vector<float>& __prevResult, cv::Mat& __img, cv::Point& __center, const double __samplingFreq)
{
    std::vector<float> resultVec = LPF.lowpassFilter(__samplingFreq, __prevPoint, __prevResult, __point);
    ROS_INFO("in filteredPoints point size: %d", __point.ranges.size());
    for(int i = 0; i < (int)__point.ranges.size(); i++)
    {
        prevData.push_back(__point.ranges[i]);
    }

    for(int i = 0; i < resultVec.size(); i++)
    {
        prevResult.push_back(resultVec[i]);

        double ang;
        double pointX, pointY;

        ang = __point.angle_min + __point.angle_increment * i;

        pointX = __center.x - std::round(std::sin(ang) * resultVec[i] * m_to_pixel);
        pointY = __center.y - std::round(std::cos(ang) * resultVec[i] * m_to_pixel);

        cv::Point pt(pointX, pointY);
        cv::circle(__img, pt, 3, cv::Scalar(0, 0, 0), 2);
        cv::line(__img, __center, pt, cv::Scalar(255, 255, 255), 2);
    }
}