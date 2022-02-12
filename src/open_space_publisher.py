#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np

def callback(data):
    float_msg = Float32()
    mi = 0
    for i in range(len(data.ranges)):
      if data.ranges[i] > data.ranges[mi]:
        mi = i

    float_msg.data = data.ranges[mi]
    dist_pub.publish(float_msg)
    
    float_msg.data = 2.0/3.0*np.pi + 1.0/300.0*np.pi*mi
    angle_pub.publish(float_msg)
    
    


def listener():
    rospy.init_node('open_space_publisher', anonymous=False)
    rospy.Subscriber("fake_scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    angle_pub = rospy.Publisher('open_space/angle', Float32, queue_size=10)
    dist_pub = rospy.Publisher('open_space/distance', Float32, queue_size=10)
    listener()