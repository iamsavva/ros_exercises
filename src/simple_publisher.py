#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

def talker():
    float_pub = rospy.Publisher('my_random_float', Float32, queue_size=10)
    rospy.init_node('simple_publisher', anonymous=False)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        float_msg = Float32()
        float_msg.data = np.random.uniform(0,10,1)[0]

        rospy.loginfo(float_msg)
        float_pub.publish(float_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass