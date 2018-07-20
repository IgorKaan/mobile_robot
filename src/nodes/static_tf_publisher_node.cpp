#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_publisher_node");
    ros::NodeHandle n;

    ros::Rate r(60.0f);

    tf::TransformBroadcaster broadcaster;

    constexpr float lidar_x = -0.00651f; + 0.07329f;
    constexpr float lidar_y = 0.0f;
    constexpr float lidar_z = 0.03909f;

    while (n.ok()) {
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(lidar_x, lidar_y, lidar_z)),
                        ros::Time::now(), "base_link", "laser"));

        r.sleep();
    }

    return 0;
}




