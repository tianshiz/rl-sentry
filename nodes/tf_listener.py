#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_sentry')
import rospy
import tf

from PyKDL import *

JOINTS = ["head", "neck", "torso", "left_shoulder", "left_elbow", "right_shoulder", "right_elbow", "left_hip", "left_knee", "right_hip", "right_knee", "left_hand", "right_hand", "left_foot", "right_foot"]

def save_frame(frame_d):
  frame_string = "%d,"%frame_d
  now = rospy.Time(0)
  for j in JOINTS:
    #(position,quaternion) = listener.lookupTransformFull("/"+j+"_1", now, "/openni_depth_frame", now, "/world")
    (position,quaternion) = listener.lookupTransform("/"+j+"_1", "/openni_depth_frame", now)
    x,y,z = position
    Qx,Qy,Qz,Qw = quaternion
    f = Frame(Rotation.Quaternion(Qx,Qy,Qz,Qw),Vector(x,y,z))    
    
    frame_string += "%f,%f,%f,%f,%f,%f,%f,%f,%f,1"%(f.M[0,0], f.M[0,1], f.M[0,2], f.M[1,0], f.M[1,1], f.M[1,2], f.M[2,0], f.M[2,1], f.M[2,2])
    frame_string += "%f,%f,%f,1"%(position)
  print frame_string

def record():
  ## Start
    
  ## Record frame
    ## Sampling rate - duration...
  ## Save to disk

  ## Repeat
  pass

if __name__ == '__main__':
  rospy.init_node('tf_listener')

  listener = tf.TransformListener()
  frame = 0
  rate = rospy.Rate(10.0)
  try:
    listener.waitForTransform("/torso_1", "/openni_depth_frame", rospy.Time(), rospy.Duration(4.0))
    print "Detected user, begin tracking torso"
  except (tf.Exception):
    print "Unable to detect user"
  while not rospy.is_shutdown():
    try:
      now = rospy.Time.now()
      listener.waitForTransform("/torso_1", "/openni_depth_frame", now, rospy.Duration(4.0))
      ## (trans,rot) = listener.lookupTransform("/torso_1", "/openni_depth_frame", rospy.Time(0))
      ##print "Torso XYZ:", trans
      save_frame(frame)
      frame = frame + 1      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print "Lost user"

      
    rate.sleep()


