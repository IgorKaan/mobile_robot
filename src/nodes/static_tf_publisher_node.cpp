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
    constexpr float lidar_front_x = 0.220;
    constexpr float lidar_front_y = 0.0;
    constexpr float lidar_front_z = 0.301f;
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

    constexpr float sonar_z = 0.2375f;
    constexpr float sonar_r = 0.2685f;

    auto make_sonar_vec = [=](const float angle_rad) -> tf::Vector3 {
	    return tf::Vector3(std::cos(angle_rad) * sonar_r, std::sin(angle_rad) * sonar_r, sonar_z);
    };

    auto make_sonar_quat = [=](const float angle_rad) -> tf::Quaternion {
	    tf::Quaternion quat;
	    quat.setRPY(0.0, 0.0, angle_rad);
	    return quat;
    };


    constexpr float sensor_0_angle = 0.0f;
    constexpr float sensor_1_angle = M_PI / 2.0f;
    constexpr float sensor_2_angle = sensor_1_angle + M_PI/4.0f;
    constexpr float sensor_3_angle = sensor_2_angle + M_PI/4.0f;
    constexpr float sensor_4_angle = sensor_3_angle + M_PI/4.0f;
    constexpr float sensor_5_angle = sensor_4_angle + M_PI/4.0f;

    const tf::Vector3 sensor_0 = make_sonar_vec(sensor_0_angle);
    tf::Quaternion sensor_0_quat = make_sonar_quat(sensor_0_angle);

    const tf::Vector3 sensor_1 = make_sonar_vec(sensor_1_angle);
    tf::Quaternion sensor_1_quat = make_sonar_quat(sensor_1_angle);

    const tf::Vector3 sensor_2 = make_sonar_vec(sensor_2_angle);
    tf::Quaternion sensor_2_quat = make_sonar_quat(sensor_2_angle);

    const tf::Vector3 sensor_3 = make_sonar_vec(sensor_3_angle);
    tf::Quaternion sensor_3_quat = make_sonar_quat(sensor_3_angle);

    const tf::Vector3 sensor_4 = make_sonar_vec(sensor_4_angle);
    tf::Quaternion sensor_4_quat = make_sonar_quat(sensor_4_angle);

    const tf::Vector3 sensor_5 = make_sonar_vec(sensor_5_angle);
    tf::Quaternion sensor_5_quat = make_sonar_quat(sensor_5_angle);

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

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_1_quat, sensor_1),
                                                       ros::Time::now(), "base_link", "sonar_frame_1"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_2_quat, sensor_2),
                                                       ros::Time::now(), "base_link", "sonar_frame_2"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_3_quat, sensor_3),
                                                       ros::Time::now(), "base_link", "sonar_frame_3"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_4_quat, sensor_4),
                                                       ros::Time::now(), "base_link", "sonar_frame_4"));

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_5_quat, sensor_5),
                                                       ros::Time::now(), "base_link", "sonar_frame_5"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(sensor_0_quat, sensor_0),
                                                       ros::Time::now(), "base_link", "sonar_frame_6"));

        r.sleep();
    }

    return 0;
}




