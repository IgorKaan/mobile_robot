#ifndef MOBILE_ROBOT_COMMS_BASE_CONTROLLER_H
#define MOBILE_ROBOT_COMMS_BASE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

#include "kinematics/differential_drive.h"


class base_controller
{
public:
    base_controller(std::string rpm_topic, std::string vel_topic);

    void twist_cb(const geometry_msgs::Twist::ConstPtr& twist_msg);
    void update();
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher left_pub, right_pub;

    float m_lin_vel {0.0f};
    float m_ang_vel {0.0f};

    differential_drive::parameters m_robot_params;
    float m_max_lin_vel;
    float m_max_ang_vel;
    ros::Time m_last_cmd_time;
};

#endif //MOBILE_ROBOT_COMMS_BASE_CONTROLLER_H
