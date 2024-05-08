#include "yaw_from_lidar/lidar_est.h"
//#include "yaw_from_lidar/lidar.h"
//#include "yaw_from_lidar/function.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yaw_from_lidar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    LidarEst LIDAR;

    nh_param.getParam("Debug_mode", LIDAR.debugMode);
    nh_param.getParam("Cutoff_Freq", LIDAR.LPF.cutoffFreq);
    nh_param.getParam("m_to_pixel", LIDAR.m_to_pixel);

    LIDAR.subscribe_lidar();

    ros::spin();

    return 0;
}