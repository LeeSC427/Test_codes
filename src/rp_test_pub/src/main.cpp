#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rp_test_pub");
    ros::NodeHandle nh;
    ros::Publisher pub_rp = nh.advertise<geometry_msgs::TwistStamped>("/indy/roll_pitch", 10);

    ros::Rate r(15);

    while(ros::ok())
    {
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.twist.linear.x = 0.0;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;

        pub_rp.publish(msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}