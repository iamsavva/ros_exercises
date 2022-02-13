#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

import numpy as np
from ros_exercises.msg import OpenSpace

def callback(data):
    os_msg = OpenSpace()
    mi = 0
    for i in range(len(data.ranges)):
      if data.ranges[i] > data.ranges[mi]:
        mi = i

    os_msg.angle = 2.0/3.0*np.pi + 1.0/300.0*np.pi*mi
    os_msg.distance = data.ranges[mi]
    # print(os_msg)
    os_pub.publish(os_msg)


def listener():
    rospy.init_node('open_space_publisher', anonymous=False)
    rospy.Subscriber(sub_topic, LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    # print(rospy.get_param_names())
    pub_topic = rospy.get_param("/open_space_publisher/pub_topic", "open_space")
    sub_topic = rospy.get_param("/open_space_publisher/sub_topic", "fake_scan")

    os_pub = rospy.Publisher(pub_topic, OpenSpace, queue_size=10)
    listener()