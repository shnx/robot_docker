#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class SonarToLaserScan:
    def __init__(self):
        rospy.init_node('sonar_to_laserscan')
        self.sonar_sub = rospy.Subscriber('/Sonar_front_10_left', LaserScan, self.sonar_callback)
        self.laser_pub = rospy.Publisher('/sonar_to_laserscan', LaserScan, queue_size=10)

    def sonar_callback(self, sonar_data):
        # Create a new LaserScan message
        laser_scan = LaserScan()
        laser_scan.header = sonar_data.header
        laser_scan.angle_min = -0.139626  # Assuming the same angles as the sonar
        laser_scan.angle_max = 0.139626
        laser_scan.angle_increment = sonar_data.angle_increment
        laser_scan.time_increment = 0.0
        laser_scan.range_min = sonar_data.range_min
        laser_scan.range_max = sonar_data.range_max

        # Fill the ranges array with zeros except for the sonar data
        num_ranges = int((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1
        laser_scan.ranges = [0.0] * num_ranges
        sonar_index = int((0.139626 - sonar_data.angle_min) / sonar_data.angle_increment)
        if 0 <= sonar_index < num_ranges:
            laser_scan.ranges[sonar_index] = sonar_data.ranges[0]

        # Publish the LaserScan message
        self.laser_pub.publish(laser_scan)

if __name__ == '__main__':
    try:
        sonar_to_laserscan = SonarToLaserScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
