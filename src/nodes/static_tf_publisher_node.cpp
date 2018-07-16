#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_publisher_node");
    ros::NodeHandle n;

    ros::Rate r(60.0f);

    tf::TransformBroadcaster broadcaster;

    while (n.ok()) {
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.00651, 0, 0.03909f)),
                        ros::Time::now(), "base_link", "laser"));

        r.sleep();
    }

    return 0;
}




