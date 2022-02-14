#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped

import numpy as np

import tf2_ros
import tf


def transform_to_pq(transform):
    p = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    q = np.array([transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w])
    return p, q

def transform_to_se3(transform):
  p, q = transform_to_pq(transform)
  norm = np.linalg.norm(q)
  if np.abs(norm - 1.0) > 1e-3:
    raise ValueError(
      "un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(str(q), np.linalg.norm(q)))
  elif np.abs(norm - 1.0) > 1e-6:
      q = q / norm
  g = tf.transformations.quaternion_matrix(q)
  g[0:3, -1] = p
  return np.array(g)


def broadcast_tf(tfmatrix, frame, stamp):
  q = tf.transformations.quaternion_from_matrix(tfmatrix)
  p = tfmatrix[0:3, 3]

  t = TransformStamped()

  t.header.stamp = stamp
  t.header.frame_id = "world"
  t.child_frame_id = frame
  t.transform.translation.x = p[0]
  t.transform.translation.y = p[1]
  t.transform.translation.z = p[2]
  t.transform.rotation.x = q[0]
  t.transform.rotation.y = q[1]
  t.transform.rotation.z = q[2]
  t.transform.rotation.w = q[3]

  tfBroadcaster.sendTransform(t)


def callback():
  # get current transform of robot wrt world
  try:
    transform = tfBuffer.lookup_transform( "world", "left_cam", rospy.Time())
  except:
    print("couldn't get transform from left_cam to world")
    return 0

  stamp = transform.header.stamp

  # convert transform into 4x4 array:
  w_2_lc = transform_to_se3(transform.transform)

  # base link to left cam
  # i could just invert the matrix i was using before, but this transform can be trivially written by hand
  lc_2_bl = np.array( [[1,0,0,0.05],[0,1,0,0],[0,0,1,0],[0,0,0,1]] )

  # dot the two to get world to left cam
  w_2_bl = np.dot( lc_2_bl, w_2_lc )
  broadcast_tf(w_2_bl, "base_link_gt_2", stamp)


    

if __name__ == '__main__':
  rospy.init_node('dtfc')

  tfBuffer = tf2_ros.Buffer()
  tfBroadcaster = tf2_ros.TransformBroadcaster()
  listener = tf2_ros.TransformListener(tfBuffer)

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    callback()
    rate.sleep()