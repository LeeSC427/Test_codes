#include "lidar_depth_dist/headers.h"

class Detection
{
    public:
        ros::NodeHandle nh_det;
        ros::Subscriber sub_scan;
        ros::Publisher pub_mindist;

        int m_to_pixel;

        std::mutex mtx_scan;

        double deg2rad(double degree);
        void subscribe_scan();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& _msg);
        double getMinDist(const sensor_msgs::LaserScan& __scan, cv::Mat& __img);

    Detection(){
        m_to_pixel = 500;
        pub_mindist = nh_det.advertise<std_msgs::Float64>("/depth_dist", 10);
    }
    ~Detection(){}
};

double Detection::deg2rad(double __degree)
{
    return __degree * M_PI / 180.0;
}

void Detection::subscribe_scan()
{
    sub_scan = nh_det.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Detection::scanCallback, this);
}

void Detection::scanCallback(const sensor_msgs::LaserScan::ConstPtr& _msg)
{
    sensor_msgs::LaserScan scan;
    cv::Mat img(640, 480, CV_8UC3, cv::Scalar(127, 127, 127));

    mtx_scan.lock();

    scan.angle_increment = _msg->angle_increment;
    scan.angle_max = _msg->angle_max;
    scan.angle_min = _msg->angle_min;
    scan.ranges = _msg->ranges;
    scan.scan_time = _msg->scan_time;

    mtx_scan.unlock();

    double mindist = getMinDist(scan, img);

    cv::namedWindow("laser scan");
    cv::imshow("laser scan", img);
    cv::waitKey(1);
}

double Detection::getMinDist(const sensor_msgs::LaserScan& __scan, cv::Mat& __img)
{
    double mindist = std::numeric_limits<double>::max();
    int minIndex = 0;
    cv::Point center(__img.cols / 2 - 1, __img.rows / 2 - 1);
    int cnt = 0;
    std_msgs::Float64 depth_dist;

    std::cout << __scan.ranges.size() << std::endl;
    for(int i = 0; i < __scan.ranges.size(); i++)
    {
        double ang;
        double pointX, pointY;

        if(std::isnan(__scan.ranges[i]))
        {
            continue;
        }
        if(std::isinf(__scan.ranges[i]))
        {
            continue;
        }
        if(__scan.ranges[i] == 0.0)
        {
            cnt++;
            continue;
        }
        
        if(__scan.ranges[i] < 1.0)
        {
            //ang = __scan.angle_max + __scan.angle_increment * i;
            ang = __scan.angle_min + __scan.angle_increment * i;

            if(__scan.angle_min <= ang && ang <= __scan.angle_max)
            {
                if(__scan.ranges[i] < mindist)
                {
                    mindist = __scan.ranges[i];
                    minIndex = i;
                }
                pointX = center.x - std::round(std::sin(ang) * __scan.ranges[i] * m_to_pixel);
                pointY = center.y - std::round(std::cos(ang) * __scan.ranges[i] * m_to_pixel);

                cv::Point pt(pointX, pointY);

                cv::circle(__img, pt, 3, cv::Scalar(0,0,0), 2);
                cv::line(__img, center, pt, cv::Scalar(255,255,255), 2);
            }
        }
    }

    depth_dist.data = mindist;
    pub_mindist.publish(depth_dist);

    //double minAngle = __scan.angle_max + __scan.angle_increment * minIndex;
    double minAngle = __scan.angle_min + __scan.angle_increment * minIndex;
    double minPointX = center.x - std::round(std::sin(minAngle) * __scan.ranges[minIndex] * m_to_pixel);
    double minPointY = center.y - std::round(std::cos(minAngle) * __scan.ranges[minIndex] * m_to_pixel);
    cv::circle(__img, cv::Point(minPointX, minPointY), 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(__img, center, 2, cv::Scalar(255, 0, 255), -1);

    std::cout << "minimum distance: " << mindist << std::endl;
    std::cout << cnt << std::endl;

    return __scan.ranges[minIndex];
}