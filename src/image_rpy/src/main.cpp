#include "image_rpy/function.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_rpy");
    ros::NodeHandle nh;
    
    Image_RPY IMGRPY;

    IMGRPY.subscribe_info();
    IMGRPY.subscribe_img();

    ros::spin();

    return 0;
}