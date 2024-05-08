#include "yaw_from_lidar/headers.h"

class Lidar
{
    public:
        ros::NodeHandle nh_det;
        ros::Subscriber sub_points;

        std::mutex mtx_lidar;

        bool isInitial;

        int m_to_pixel;

        double samplingFreq;
        double cutoffFreq;
        double alpha;
        double prevTime;

        std::vector<double> prevResult;
        std::vector<double> prevData;

        void subscribe_lidar();
        void lidar_callback(const sensor_msgs::LaserScanConstPtr& _msg);

        double bilinearT(const double __data);

    Lidar()
    {
        isInitial = true;
        m_to_pixel = 500;
        cutoffFreq = 9.0;
        alpha = (cutoffFreq / samplingFreq) / (1 + 2 * cutoffFreq / samplingFreq);
    }
    ~Lidar(){}
};

void Lidar::subscribe_lidar()
{
    sub_points = nh_det.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Lidar::lidar_callback, this);
}

void Lidar::lidar_callback(const sensor_msgs::LaserScanConstPtr& _msg)
{
    double start_time = ros::Time::now().toNSec();
    prevTime = start_time;
    samplingFreq = 1000000000 / (start_time - prevTime);
    cv::Mat img(640,480, CV_8UC3, cv::Scalar(127, 127, 127));
    sensor_msgs::LaserScan data;
    cv::Point center(img.cols / 2 - 1, img.rows / 2 - 1);

    mtx_lidar.lock();

    data.ranges = _msg->ranges;
    data.angle_min = _msg->angle_min;
    data.angle_max = _msg->angle_max;
    data.angle_increment = _msg->angle_increment;

    mtx_lidar.unlock();

    std::vector<double> pointsVec;
    std::vector<double> tempPrevResult;
    std::vector<double> tempPrevData;

    tempPrevResult.clear();
    tempPrevData.clear();

    tempPrevResult.swap(prevResult);
    tempPrevData.swap(prevData);

    if(isInitial)
    {
        for(int i = 0; i < data.ranges.size(); i++)
        {
            double ang;
            double pointX, pointY;
            if(std::isnan(data.ranges[i]))
                continue;
            if(std::isinf(data.ranges[i]))
                continue;
            if(data.ranges[i] == 0.0)
                continue;

            ang = data.angle_min + data.angle_increment * i;

            if(data.angle_min <= ang && ang <= data.angle_max)
            {
                pointX = center.x - std::round(std::sin(ang) * data.ranges[i] * m_to_pixel);
                pointY = center.y - std::round(std::cos(ang) * data.ranges[i] * m_to_pixel);
            }

            cv::Point pt(pointX, pointY);

            cv::circle(img, pt, 3, cv::Scalar(0,0,0), 2);
            cv::line(img, center, pt, cv::Scalar(255,255,255), 2);
            prevResult.push_back(data.ranges[i]);
            prevData.push_back(data.ranges[i]);
            isInitial = false;
        }
    }
    else
    {
        for(const auto& data : data.ranges)
        {
            pointsVec.push_back(data);
        }

        std::vector<double> curResult;

        for(int i = 0; i < pointsVec.size(); i++)
        {
            double result;

            result = alpha * (pointsVec[i] + tempPrevData[i]) / 2 + (1 - alpha) * tempPrevResult[i];
            curResult.push_back(result);
            prevData.push_back(pointsVec[i]);
            prevResult.push_back(result);
        }

        for(int i = 0; i < curResult.size(); i++)
        {
            if(std::isnan(curResult[i]))
                continue;
            if(std::isinf(curResult[i]))
                continue;
            if(curResult[i] == 0.0)
                continue;

            double ang;
            double pointX, pointY;
            ang = data.angle_min + data.angle_increment * i;

            if(data.angle_min <= ang && ang <= data.angle_max)
            {
                pointX = center.x - std::round(std::sin(ang) * curResult[i] * m_to_pixel);
                pointY = center.y - std::round(std::cos(ang) * curResult[i] * m_to_pixel);

                cv::Point pt(pointX, pointY);

                cv::circle(img, pt, 3, cv::Scalar(0,0,0), 2);
                cv::line(img, center, pt, cv::Scalar(255,255,255), 2);
            }
        }
    }

    cv::namedWindow("laser scan");
    cv::imshow("laser scan", img);
    cv::waitKey(1);
}

double Lidar::bilinearT(const double __data)
{
    return 2 * samplingFreq * (__data - 1) / (__data + 1);
}