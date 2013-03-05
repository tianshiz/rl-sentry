#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_sentry')
import rospy
import tf

if __name__ == '__main__':
  rospy.init_node('tf_listener')

  listener = tf.TransformListener()

  rate = rospy.Rate(10.0)
  listener.waitForTransform("/openni_depth_frame", "/torso_1", rospy.Time(), rospy.Duration(4.0))
  print "Detected user, begin tracking torso"
  while not rospy.is_shutdown():
    try:
      now = rospy.Time.now()
      listener.waitForTransform("/openni_depth_frame", "/torso_1", now, rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform("/openni_depth_frame", "/torso_1", rospy.Time(0))
      print "Torso XYZ:", trans
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
      
    rate.sleep()
