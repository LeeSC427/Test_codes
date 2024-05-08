#include "lidar_depth_dist/function.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_depth_dist");
    ros::NodeHandle nh;

    Detection DETECT;

    DETECT.subscribe_scan();

    ros::spin();
    
    return 0;
}