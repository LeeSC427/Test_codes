#include "image_rpy/headers.h"

class Image_RPY
{
    public:
        ros::NodeHandle nh_img;
        ros::Subscriber sub_img;
        ros::Subscriber sub_info;

        std::mutex mtx_img;
        std::mutex mtx_info;

        double fLength;

        void subscribe_img();
        void subscribe_info();
        void img_callback(const sensor_msgs::ImageConstPtr& _msg);
        void info_callback(const sensor_msgs::CameraInfoConstPtr& _msg);
        cv::Mat invrpy(cv::Mat input, double roll, double pitch, double yaw);

    Image_RPY(){}
    ~Image_RPY(){}
};

void Image_RPY::subscribe_info()
{
    sub_info = nh_img.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 10, &Image_RPY::info_callback, this);
}

void Image_RPY::info_callback(const sensor_msgs::CameraInfoConstPtr& _msg)
{
    double focalX, focalY;
    mtx_info.lock();
    focalX = _msg->K.at(0);
    focalY = _msg->K.at(4);
    mtx_info.unlock();

    fLength = (focalX + focalY) / 2.0;
    ROS_INFO("Focal Length: %lf", fLength);
    sub_info.shutdown();
}

void Image_RPY::subscribe_img()
{
    sub_img = nh_img.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &Image_RPY::img_callback, this);
}

void Image_RPY::img_callback(const sensor_msgs::ImageConstPtr& _msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    mtx_img.lock();
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        return;
    }
    mtx_img.unlock();

    cv::Mat img;
    cv_ptr->image.copyTo(img);

    int key = cv::waitKey(1);

    //if(key == 83)
    if(key == 83)
    {
        double roll, pitch, yaw;
        cv::imwrite("/home/lsc/test_ws/image_rpy/test.png", img);
        std::cout << "roll: "; std::cin >> roll;
        std::cout << "pitch: "; std::cin >> pitch;
        std::cout << "yaw: "; std::cin >> yaw;

        cv::Mat rotImg = invrpy(img, roll, pitch, yaw);
        cv::imwrite("/home/lsc/test_ws/image_rpy/test_result.png", img);
    }
}

cv::Mat Image_RPY::invrpy(cv::Mat input, double roll, double pitch, double yaw)
{
    int rows = input.rows;
    int cols = input.cols;
    cv::Mat output(rows, cols, CV_8UC3);
    double cam_roll = -pitch;
    double cam_pitch = -yaw;
    double cam_yaw = roll;
    double cr = std::cos(roll);
    double sr = -std::sin(roll);
    double cp = std::cos(pitch);
    double sp = -std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = -std::sin(yaw);

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            double denom = (j*(cp*sy*sr-sp*cr))+(i*cy*sr)+(fLength*(sp*sy*sr+cp*cr));
            cv::Point coord;
            coord.x = fLength * ((j*cp*cy)+(fLength*sp*cy)-(i*sy))/denom;
            coord.y = fLength * ((j*(cp*sy*cr+sp*sr))+(i*cy*cr)+(fLength*(sp*sy*cr-cp*sr)))/denom;
            output.at<cv::Scalar>(coord) = input.at<cv::Scalar>(j,i);
        }
    }

    return output;
}