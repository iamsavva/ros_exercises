#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def talker():
    pub_topic = rospy.get_param("/fake_scan_publisher/pub_topic", "fake_scan")
    pub_rate = rospy.get_param("/fake_scan_publisher/pub_rate", 20)
    angle_min = rospy.get_param("/fake_scan_publisher/angle_min", -2.0/3.0 * np.pi)
    angle_max = rospy.get_param("/fake_scan_publisher/angle_max", 2.0/3.0 * np.pi)
    range_min = rospy.get_param("/fake_scan_publisher/range_min", 1.0)
    range_max = rospy.get_param("/fake_scan_publisher/range_max", 10.0)
    angle_increment = rospy.get_param("/fake_scan_publisher/angle_increment", 1.0/300.0*np.pi)

    rospy.init_node('fake_scan_publisher', anonymous=False)

    pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)

    rate = rospy.Rate(pub_rate)
    prev_time_scan = rospy.Time.now()

    while not rospy.is_shutdown():
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.scan_time = (scan_msg.header.stamp - prev_time_scan).to_sec()
        prev_time_scan = scan_msg.header.stamp
        scan_msg.range_min = range_min
        scan_msg.range_max = range_max

        k = int( np.rint( (scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment ) + 1 )

        scan_msg.ranges = list( np.random.uniform(scan_msg.range_min, 
                                                  scan_msg.range_max, 
                                                  k ) )
        


        # rospy.loginfo("published %s", scan_msg)
        pub.publish(scan_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass