#ifndef MOBILE_ROBOT_COMMS_BASE_CONTROLLER_H
#define MOBILE_ROBOT_COMMS_BASE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

#include "kinematics/omniwheel_base.h"


class base_controller
{
public:
    base_controller(std::string vel_topic);

    void twist_cb(const geometry_msgs::Twist::ConstPtr& twist_msg);
    void update();
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher top_left_pub, bottom_left_pub, bottom_right_pub, top_right_pub;

    float m_lin_vel_x {0.0f};
    float m_lin_vel_y {0.0f};
    float m_ang_vel {0.0f};

    omniwheel_base::parameters m_robot_params;
    float m_max_lin_vel;
    float m_max_ang_vel;
    ros::Time m_last_cmd_time;
};

#endif //MOBILE_ROBOT_COMMS_BASE_CONTROLLER_H
