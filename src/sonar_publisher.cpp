#include "sonar_publisher.h"

#include <functional>

sonar_publisher::sonar_publisher(const float sensor_fov, const float min_range, const float max_range)
    : m_nh("sonar_publisher"), m_sensor_fov{sensor_fov}, m_min_range{min_range}, m_max_range{max_range},
      m_sonar_data(NUM_SONARS, 0)
{
    for (int idx = 0; idx < NUM_SONARS; idx++) {
        SonarArrayRange rangeType;
        std::string sub_topic_str;
        int arr_sub = 0;
        if (idx < 3) {
            rangeType = SonarArrayRange::SONAR_1_3;
            sub_topic_str = "/sensors1_3_data";
            arr_sub = 0;
        } else if (idx < 5) {
            rangeType = SonarArrayRange::SONAR_4_6;
            sub_topic_str = "/sensors4_6_data";
            arr_sub = 1;
        } else if (idx < 9) {
            rangeType = SonarArrayRange::SONAR_7_9;
            sub_topic_str = "/sensors7_8_data";
            arr_sub = 2;
        }

        boost::function<void(const geometry_msgs::Vector3&)> fn = boost::bind(&sonar_publisher::sonar_vector_cb, this, _1, rangeType);

        std::stringstream ss;
        ss << "/sonar_" << idx + 1;
        std::string pub_topic_str(ss.str());

        m_array_subs[arr_sub] = m_nh.subscribe<std_msgs::UInt8MultiArray>(sub_topic_str, 10, fn);
        m_sonar_pubs[idx] = m_nh.advertise<sensor_msgs::Range>(pub_topic_str, 10);
    }

    m_last_cmd_time = ros::Time::now();

    ROS_INFO("Initialized sonar_publisher_node");
}

void sonar_publisher::update()
{
    float cb_dt = (ros::Time::now() - m_last_cmd_time).toSec();

    for (int idx = 0; idx < NUM_SONARS; idx++) {
        sensor_msgs::Range range_msg;

	std::stringstream ss;
	ss << "sonar_frame_" << idx + 1;
	range_msg.header.frame_id = ss.str();

        range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        range_msg.field_of_view = m_sensor_fov;
        range_msg.min_range = m_min_range;
        range_msg.max_range = m_max_range;

        if (cb_dt > 0.5f) {
	    // handle delayed cb (maybe)
            range_msg.range = m_sonar_data[idx];
        } else {
            range_msg.range = m_sonar_data[idx];
        }

        m_sonar_pubs[idx].publish(range_msg);
    }
}

void sonar_publisher::sonar_array_cb(const std_msgs::UInt8MultiArray& sonars,
                                     const sonar_publisher::SonarArrayRange range)
{
    switch (range) {
        case SonarArrayRange::SONAR_1_3:
            publish_array_range(sonars, 0, 2);
            break;
	case SonarArrayRange::SONAR_4_6:
            publish_array_range(sonars, 3, 5);
            break;
	case SonarArrayRange::SONAR_7_9:
            publish_array_range(sonars, 6, 8);
            break;
    }

    m_last_cmd_time = ros::Time::now();
}

void sonar_publisher::sonar_vector_cb(const geometry_msgs::Vector3& sonars, const SonarArrayRange range) {
    switch (range) {
        case SonarArrayRange::SONAR_1_3:
            publish_vector_range(sonars, 0, 2);
            break;
	case SonarArrayRange::SONAR_4_6:
            publish_vector_range(sonars, 3, 5);
            break;
	case SonarArrayRange::SONAR_7_9:
            publish_vector_range(sonars, 6, 8);
            break;
    }

    m_last_cmd_time = ros::Time::now();
}

void sonar_publisher::publish_array_range(const std_msgs::UInt8MultiArray &sonars, const int start_id, const int end_id) {
    for (size_t idx = start_id; idx <= end_id; idx++) {
        m_sonar_data[idx] = sonars.data[idx-start_id] / 100.0f;
    }
}

void sonar_publisher::publish_vector_range(const geometry_msgs::Vector3& sonars, const int start_id, const int end_id) {
	m_sonar_data[start_id] = sonars.x / 100.0f;
	m_sonar_data[start_id+1] = sonars.y / 100.0f;
	m_sonar_data[start_id+2] = sonars.z / 100.0f;
}

