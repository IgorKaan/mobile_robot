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
     * FRONT LEFT
     */
    const tf::Vector3 sensor_fl_0(0.2307, 0.0638, 0.0f);
    tf::Quaternion sensor_fl_0_quat;
    const float sensor_fl0_angle = 0.0;
    sensor_fl_0_quat.setRPY(0.0, 0.0, sensor_fl0_angle);

    const tf::Vector3 sensor_fl_1(0.2175, 0.1047, 0.0f);
    tf::Quaternion sensor_fl_1_quat;
    const float sensor_fl1_angle = std::atan2(sensor_fl_1.y(), sensor_fl_1.x());
    sensor_fl_1_quat.setRPY(0.0, 0.0, sensor_fl1_angle);

    const tf::Vector3 sensor_fl_2(0.1987, 0.1347, 0.0f);
    tf::Quaternion sensor_fl_2_quat;
    const float sensor_fl2_angle = std::atan2(sensor_fl_2.y(), sensor_fl_2.x());
    sensor_fl_2_quat.setRPY(0.0, 0.0, sensor_fl2_angle);

    /*
     * FRONT RIGHT
     */
    const tf::Vector3 sensor_fr_0(sensor_fl_0.x(), -sensor_fl_0.y(), sensor_fl_0.z());
    tf::Quaternion sensor_fr_0_quat;
    sensor_fr_0_quat.setRPY(0.0, 0.0, -sensor_fl0_angle);

    const tf::Vector3 sensor_fr_1(sensor_fl_1.x(), -sensor_fl_1.y(), sensor_fl_1.z());
    tf::Quaternion sensor_fr_1_quat;
    sensor_fr_1_quat.setRPY(0.0, 0.0, -sensor_fl1_angle);

    const tf::Vector3 sensor_fr_2(sensor_fl_2.x(), -sensor_fl_2.y(), sensor_fl_2.z());
    tf::Quaternion sensor_fr_2_quat;
    sensor_fr_2_quat.setRPY(0.0, 0.0, -sensor_fl2_angle);

    /*
     * REAR LEFT
     */
    const tf::Vector3 sensor_rl_0(-sensor_fl_0.x(), sensor_fl_0.y(), sensor_fl_0.z());
    tf::Quaternion sensor_rl_0_quat;
    sensor_rl_0_quat.setRPY(0.0, 0.0, M_PI-sensor_fl0_angle);

    const tf::Vector3 sensor_rl_1(-sensor_fl_1.x(), sensor_fl_1.y(), sensor_fl_1.z());
    tf::Quaternion sensor_rl_1_quat;
    sensor_rl_1_quat.setRPY(0.0, 0.0, M_PI-+sensor_fl1_angle);

    const tf::Vector3 sensor_rl_2(-sensor_fl_2.x(), sensor_fl_2.y(), sensor_fl_2.z());
    tf::Quaternion sensor_rl_2_quat;
    sensor_rl_2_quat.setRPY(0.0, 0.0, M_PI-sensor_fl2_angle);

    /*
     * REAR RIGHT
     */
    const tf::Vector3 sensor_rr_0(-sensor_fl_0.x(), -sensor_fl_0.y(), sensor_fl_0.z());
    tf::Quaternion sensor_rr_0_quat;
    sensor_rr_0_quat.setRPY(0.0, 0.0, M_PI+sensor_fl0_angle);

    const tf::Vector3 sensor_rr_1(-sensor_fl_1.x(), -sensor_fl_1.y(), sensor_fl_1.z());
    tf::Quaternion sensor_rr_1_quat;
    sensor_rr_1_quat.setRPY(0.0, 0.0, M_PI+sensor_fl1_angle);

    const tf::Vector3 sensor_rr_2(-sensor_fl_2.x(), -sensor_fl_2.y(), sensor_fl_2.z());
    tf::Quaternion sensor_rr_2_quat;
    sensor_rr_2_quat.setRPY(0.0, 0.0, M_PI+sensor_fl2_angle);

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

	/*
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fl_0_quat, sensor_fl_0),
                                                       ros::Time::now(), "base_link", "sensor_frame_fl_0"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fl_1_quat, sensor_fl_1),
                                                       ros::Time::now(), "base_link", "sensor_frame_fl_1"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fl_2_quat, sensor_fl_2),
                                                       ros::Time::now(), "base_link", "sensor_frame_fl_2"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fr_0_quat, sensor_fr_0),
                                                       ros::Time::now(), "base_link", "sensor_frame_fr_0"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fr_1_quat, sensor_fr_1),
                                                       ros::Time::now(), "base_link", "sensor_frame_fr_1"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_fr_2_quat, sensor_fr_2),
                                                       ros::Time::now(), "base_link", "sensor_frame_fr_2"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rl_0_quat, sensor_rl_0),
                                                       ros::Time::now(), "base_link", "sensor_frame_rl_0"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rl_1_quat, sensor_rl_1),
                                                       ros::Time::now(), "base_link", "sensor_frame_rl_1"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rl_2_quat, sensor_rl_2),
                                                       ros::Time::now(), "base_link", "sensor_frame_rl_2"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rr_0_quat, sensor_rr_0),
                                                       ros::Time::now(), "base_link", "sensor_frame_rr_0"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rr_1_quat, sensor_rr_1),
                                                       ros::Time::now(), "base_link", "sensor_frame_rr_1"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_rr_2_quat, sensor_rr_2),
                                                       ros::Time::now(), "base_link", "sensor_frame_rr_2"));
	*/
        r.sleep();
    }

    return 0;
}




