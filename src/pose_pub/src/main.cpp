#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle nh;
    ros::Publisher pub_pose;
    pub_pose = nh.advertise<geometry_msgs::Twist>("/pose", 10);

    while(ros::ok())
    {
        geometry_msgs::Twist poseVec;
        double tx, ty, tz;
        double roll, pitch, yaw;

        std::cout << "translate x: ";
        std::cin >> tx;
        std::cout << "translate y: ";
        std::cin >> ty;
        std::cout << "translate z: ";
        std::cin >> tz;
        std::cout << "rotate roll: ";
        std::cin >> roll;
        std::cout << "rotate pitch: ";
        std::cin >> pitch;
        std::cout << "rotate yaw: ";
        std::cin >> yaw;

        poseVec.linear.x = tx;
        poseVec.linear.y = ty;
        poseVec.linear.z = tz;
        poseVec.angular.x = roll;
        poseVec.angular.y = pitch;
        poseVec.angular.z = yaw;

        pub_pose.publish(poseVec);
    }

    return 0;
}