#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def talker():
    rospy.init_node('fake_scan_publisher', anonymous=False)

    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)

    rate = rospy.Rate(20)
    prev_time_scan = rospy.Time.now()

    while not rospy.is_shutdown():
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = -2.0/3.0 * np.pi
        scan_msg.angle_max = 2.0/3.0*np.pi
        scan_msg.angle_increment = 1.0/300.0*np.pi
        scan_msg.scan_time = (scan_msg.header.stamp - prev_time_scan).to_sec()
        prev_time_scan = scan_msg.header.stamp
        scan_msg.range_min = 1.0
        scan_msg.range_max = 10.0

        scan_msg.ranges = list( np.random.uniform(scan_msg.range_min, 
                                                  scan_msg.range_max, 
                                                  401 ) )
        


        # rospy.loginfo("published %s", scan_msg)
        pub.publish(scan_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass