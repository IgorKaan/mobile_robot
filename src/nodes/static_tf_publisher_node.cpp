#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_publisher_node");
    ros::NodeHandle n;

    ros::Rate r(60.0f);

    tf::TransformBroadcaster broadcaster;

    
    /*
    constexpr float lidar_front_x = 0.2498f;
    constexpr float lidar_front_y = 0.2498f;
    constexpr float lidar_front_z = 0.3305f;
    */
    constexpr float lidar_front_x = 0.268;
    constexpr float lidar_front_y = 0.0;
    constexpr float lidar_front_z = 0.3305f;
    tf::Quaternion lidar_front_quat;
    //lidar_front_quat.setRPY(0.0, 0.0, M_PI+M_PI/4);
    lidar_front_quat.setRPY(0.0, 0.0, 0.0);

    /*
    constexpr float lidar_rear_x = -0.2498f;
    constexpr float lidar_rear_y = -0.2498f;
    constexpr float lidar_rear_z = 0.3305f;
    tf::Quaternion lidar_rear_quat;
    lidar_rear_quat.setRPY(0.0, 0.0, M_PI/4);

    tf::Quaternion lidar_merged_quat;
    lidar_merged_quat.setRPY(0.0, 0.0, 0.0);
    */

    /*
     * LEFT
     */
    const tf::Vector3 sensor_fl_0(0, 0.3, lidar_front_z);
    tf::Quaternion sensor_fl_0_quat;
    sensor_fl_0_quat.setRPY(0.0, 0.0, M_PI/2.0f);

    const tf::Vector3 sensor_fl_1(-0.2, 0.301, lidar_front_z);
    tf::Quaternion sensor_fl_1_quat;
    sensor_fl_1_quat.setRPY(0.0, 0.0, M_PI/2.0f);


    /*
     * RIGHT
     */
    const tf::Vector3 sensor_fr_0(sensor_fl_0.x(), -sensor_fl_0.y(), sensor_fl_0.z());
    tf::Quaternion sensor_fr_0_quat;
    sensor_fr_0_quat.setRPY(0.0, 0.0, -M_PI/2.0f);

    const tf::Vector3 sensor_fr_1(sensor_fl_1.x(), -sensor_fl_1.y(), sensor_fl_1.z());
    tf::Quaternion sensor_fr_1_quat;
    sensor_fr_1_quat.setRPY(0.0, 0.0, -M_PI/2.0f);

    /*
     * REAR 
     */
    const tf::Vector3 sensor_rl_0(-0.3, 0.2, lidar_front_z);
    tf::Quaternion sensor_rl_0_quat;
    sensor_rl_0_quat.setRPY(0.0, 0.0, M_PI);

    const tf::Vector3 sensor_rl_1(sensor_rl_0.x(), -sensor_rl_0.y(), sensor_rl_0.z());
    tf::Quaternion sensor_rl_1_quat;
    sensor_rl_1_quat.setRPY(0.0, 0.0, M_PI);

    while (n.ok()) {
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(lidar_front_quat, tf::Vector3(lidar_front_x, lidar_front_y, lidar_front_z)),
                        ros::Time::now(), "base_link", "laser_front"));
	/*
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(lidar_rear_quat, tf::Vector3(lidar_rear_x, lidar_rear_y, lidar_rear_z)),
                        ros::Time::now(), "base_link", "laser_rear"));
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(lidar_merged_quat, tf::Vector3(0.0, 0.0, lidar_front_z)),
                        ros::Time::now(), "base_link", "laser_merged"));
	*/

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fl_0_quat, sensor_fl_0),
                                                       ros::Time::now(), "base_link", "sonar_frame_6"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fl_1_quat, sensor_fl_1),
                                                       ros::Time::now(), "base_link", "sonar_frame_5"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fr_0_quat, sensor_fr_0),
                                                       ros::Time::now(), "base_link", "sonar_frame_1"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fr_1_quat, sensor_fr_1),
                                                       ros::Time::now(), "base_link", "sonar_frame_2"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rl_0_quat, sensor_rl_0),
                                                       ros::Time::now(), "base_link", "sonar_frame_4"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rl_1_quat, sensor_rl_1),
                                                       ros::Time::now(), "base_link", "sonar_frame_3"));

        r.sleep();
    }

    return 0;
}




