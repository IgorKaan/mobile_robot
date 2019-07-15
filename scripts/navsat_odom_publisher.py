#!/usr/bin/python
import rospy
import math
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

WGS84 = 6378137, 298.257223563

def geodetic_to_geocentric(ellipsoid, latitude, longitude, height):
    phi = math.radians(latitude)
    lambd = math.radians(longitude)
    sin_phi = math.sin(phi)
    a, rf = ellipsoid           # semi-major axis, reciprocal flattening
    e2 = 1 - (1 - 1 / rf) ** 2  # eccentricity squared
    n = a / math.sqrt(1 - e2 * sin_phi ** 2) # prime vertical radius
    r = (n + height) * math.cos(phi)   # perpendicular distance from z axis
    x = r * math.cos(lambd)
    y = r * math.sin(lambd)
    z = (n * (1 - e2) + height) * sin_phi
    return x, y, z


class NavSatOdometry:
    def __init__(self, name='nmea_odom_publisher'):
        self.pub = rospy.Publisher('/gps/fix', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('/fix', NavSatFix, self.navsatfix_callback)

    def navsatfix_callback(self, msg):
        if msg.status.status == -1:
            return

        lat, lon, height = msg.latitude, msg.longitude, msg.altitude
        x, y, z = geodetic_to_geocentric(WGS84, lat, lon, height)
        cov_x = msg.position_covariance[0]
        cov_y = msg.position_covariance[4]
        cov_z = msg.position_covariance[8]

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation.w = 1
        odom_msg.pose.covariance = (
            cov_x, 0, 0, 0, 0, 0,
            0, cov_y, 0, 0, 0, 0,
            0, 0, cov_z, 0, 0, 0,
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            0, 0, 0, 0, 0, 99999
        )

        self.pub.publish(odom_msg);


if __name__ == "__main__":
    rospy.init_node('nmea_odom_publisher')

    pubsub = NavSatOdometry()

    rospy.spin()

