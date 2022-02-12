#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo("received %s", data.data)
    float_log_pub.publish( data )

def listener():
    rospy.init_node('simple_subscriber', anonymous=False)

    rospy.Subscriber("my_random_float", Float32, callback)
    
    rospy.spin()

if __name__ == '__main__':
    float_log_pub = rospy.Publisher('random_float_log', Float32, queue_size=10)
    listener()