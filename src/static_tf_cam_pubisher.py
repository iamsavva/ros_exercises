#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped

import numpy as np

import tf2_ros

def broadcast_tf(p, frame):
  t = TransformStamped()

  t.header.stamp = rospy.Time()
  t.header.frame_id = "base_link_gt"
  t.child_frame_id = frame
  t.transform.translation.x = p[0]
  t.transform.translation.y = p[1]
  t.transform.translation.z = p[2]
  t.transform.rotation.x = 0.0
  t.transform.rotation.y = 0.0
  t.transform.rotation.z = 0
  t.transform.rotation.w = 1

  return t
   

if __name__ == '__main__':
  rospy.init_node('static_tf_cam_publisher')

  tfBuffer = tf2_ros.Buffer()
  tb = tf2_ros.StaticTransformBroadcaster()
  listener = tf2_ros.TransformListener(tfBuffer)

  t_bl_2_lc = np.array( [-0.05, 0, 0] )
  t_bl_2_rc = np.array( [0.05, 0, 0] )

  t1 = broadcast_tf( t_bl_2_lc, "left_cam")  
  t2 = broadcast_tf( t_bl_2_rc, "right_cam") 

  tb.sendTransform( [t1,t2] )

   

  rospy.spin()