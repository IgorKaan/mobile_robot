#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_publisher_node");
    ros::NodeHandle n;

    ros::Rate r(60.0f);

    tf::TransformBroadcaster broadcaster;

    constexpr float lidar_front_x = 0.2496f;
    constexpr float lidar_front_y = 0.2510f;
    constexpr float lidar_front_z = 0.3305f;
    tf::Quaternion lidar_front_quat;
    //lidar_front_quat.setRPY(0.0, 0.0, 0.0);
    lidar_front_quat.setRPY(0.0, 0.0, -M_PI);

    constexpr float lidar_rear_x = -0.2496f;
    constexpr float lidar_rear_y = -0.2510f;
    constexpr float lidar_rear_z = 0.3305f;
    tf::Quaternion lidar_rear_quat;
    lidar_rear_quat.setRPY(0.0, 0.0, 0.0);

    while (n.ok()) {
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(lidar_front_quat, tf::Vector3(lidar_front_x, lidar_front_y, lidar_front_z)),
                        ros::Time::now(), "base_link", "laser_front"));
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(lidar_rear_quat, tf::Vector3(lidar_rear_x, lidar_rear_y, lidar_rear_z)),
                        ros::Time::now(), "base_link", "laser_rear"));
        r.sleep();
    }

    return 0;
}




