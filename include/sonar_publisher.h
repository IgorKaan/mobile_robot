#pragma onc

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Range.h>

class sonar_publisher
{
public:
    static const int NUM_ARRAY_SUBS = 4;
    static const int NUM_SONARS_IN_ARRAY = 4;
    static const int NUM_SONARS = 16;

    enum class SonarArrayRange {
        SONAR_0_3,
        SONAR_4_7,
        SONAR_8_11,
        SONAR_12_15
    };
public:
    sonar_publisher(const float sensor_fov, const float min_range, const float max_range);

    void sonar_array_cb(const std_msgs::UInt8MultiArray& sonars, const SonarArrayRange range);

    void update();
private:
    void publish_array_range(const std_msgs::UInt8MultiArray& sonars, const int start_id, const int end_id);
private:
    float m_sensor_fov{}, m_min_range{}, m_max_range{};
    ros::NodeHandle m_nh;
    ros::Subscriber m_array_subs[NUM_ARRAY_SUBS];
    ros::Publisher m_sonar_pubs[NUM_SONARS];
    std::vector<uint8_t> m_sonar_data;

    ros::Time m_last_cmd_time;
};

