#include "vector_test/headers.h"

void poseCallback(const geometry_msgs::TwistConstPtr& _msg)
{
    std::mutex transMtx;
    geometry_msgs::Twist transVec;

    transMtx.lock();
    transVec.angular = _msg->angular;
    transVec.linear = _msg->linear;
    transMtx.unlock();

    tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(transVec.linear.x, transVec.linear.y, transVec.linear.z));
    tf::Quaternion q;
    q.setRPY(transVec.angular.x, transVec.angular.y, transVec.angular.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "cam"));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vector_test");
    ros::NodeHandle nh;
    ros::Subscriber sub_pose;

    sub_pose = nh.subscribe<geometry_msgs::Twist>("/pose", 10, poseCallback);

    ros::spin();

    return 0;
}