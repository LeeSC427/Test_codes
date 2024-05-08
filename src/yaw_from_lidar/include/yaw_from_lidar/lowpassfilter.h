#include "yaw_from_lidar/headers.h"

class Lowpassfilter
{
    public:
        double cutoffFreq;
        double alpha;

        double bilinear(const double __data, const double __samplingFreq);
        float getAlpha(const float __samplingFreq);

        std::vector<float> lowpassFilter(const double __samplingFreq, const std::vector<float>& __prevpoint, const std::vector<float>& __prevresult, const sensor_msgs::LaserScan& __scan);

    Lowpassfilter(){}
    ~Lowpassfilter(){}
};

double Lowpassfilter::bilinear(const double __data, const double __samplingFreq)
{
    return 2 * __samplingFreq * (__data - 1) / (__data + 1);
}

float Lowpassfilter::getAlpha(const float __samplingFreq)
{
    return (cutoffFreq / __samplingFreq) / (1 + 2 * cutoffFreq / __samplingFreq);
}

std::vector<float> Lowpassfilter::lowpassFilter(const double __samplingFreq, const std::vector<float>& __prevpoint, const std::vector<float>& __prevresult, const sensor_msgs::LaserScan& __scan)
{
    std::vector<float> resultVec;
    //ROS_INFO("getAlpha");
    double alpha = getAlpha(__samplingFreq);

    //ROS_INFO("for loop");
    for(int i = 0; i < (int)__scan.ranges.size(); i++)
    {
        ROS_INFO("STEP 1");
        if(std::isnan(__scan.ranges[i]))
            continue;
        if(std::isinf(__scan.ranges[i]))
            continue;
        if(__scan.ranges[i] == 0.0)
            continue;

        ROS_INFO("STEP 2");
        if(__scan.ranges.size() == __prevpoint.size())
        {
            ROS_INFO("STEP 3");
            double result = alpha * (__scan.ranges[i] + __prevpoint[i]) / 2 + (1 - alpha) * __prevresult[i];
            resultVec.push_back(result);
        }
        else
        {
            resultVec = __prevresult;
            ROS_WARN("Current points: %ld, Previous points: %ld", __scan.ranges.size(), __prevpoint.size());
        }
    }

    return resultVec;
}