#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_sentry')
import rospy
import tf
import fof
import sys
import numpy
sys.path.insert(0,'../libsvm-3.17/python')
from svmutil import *

from PyKDL import *

# array of joints to iterate over
JOINTS = ["head", "neck", "torso", "left_shoulder", "left_elbow", "right_shoulder", "right_elbow", "left_hip", "left_knee", "right_hip", "right_knee", "left_hand", "right_hand", "left_foot", "right_foot"]
joint_n = (11,4)

# array of states robot can be in
STATES = ["Neutral", "Friendly", "Hostile", "HOSTILE"]

global user

#-------------------------------------------------------------
# grab an individual frame of joint information
#-------------------------------------------------------------
# params:
#   listener - used to find joint positions and orientations
#-------------------------------------------------------------
# returns:
#   frame_string - string of joint data 
#-------------------------------------------------------------
def grab_frame(listener):
  global user
  frame_string = "0"
  now = rospy.Time(0)

  # iterate over joints to find their position and orientation in current frame
  i = 1
  for j in JOINTS:
    (position,quaternion) = listener.lookupTransform("/openni_depth_frame", "/"+j+"_"+str(user), now)
    x,y,z = position
    Qx,Qy,Qz,Qw = quaternion
    f = Frame(Rotation.Quaternion(Qx,Qy,Qz,Qw),Vector(x,y,z))    
    
    # append orientation and position data for current joing to frame_string
    if i < 12:
      frame_string += ",%f,%f,%f,%f,%f,%f,%f,%f,%f,1"%(f.M[0,0], f.M[0,1], f.M[0,2], f.M[1,0], f.M[1,1], f.M[1,2], f.M[2,0], f.M[2,1], f.M[2,2])

    frame_string += ",%f,%f,%f,1"%(position)
    i += 1 

  return frame_string
 
#-------------------------------------------------------------
# Classify an individual frame as either friendly or hostile
# output confidence values 
#-------------------------------------------------------------
# params:
#   listener - used to find joint positions and orientations
#   m        - model used to classify frames
#-------------------------------------------------------------
# returns:
#   p_val    - confidence value of classification 
#-------------------------------------------------------------
def classify_frame(listener,m):

  f = grab_frame(listener)  
 
  frames = []
  joints = [[Rotation(1,0,0,0,1,0,0,0,1),Vector(0,0,0)]]*15
  data = numpy.array([float(v) for v in f.split(',')])

  index = 1
  for j in xrange(joint_n[0]):
    joints[j]=[Rotation(*data[index:index+9]),                # rotation matrix
               Vector(*data[index+10:index+13])]              # position
    index = index + 14
  for j in xrange(joint_n[0], joint_n[0]+joint_n[1]):
    joints[j]=[Rotation(1,0,0,0,1,0,0,0,1),Vector(*data[index:index+3])] # position
    index = index + 4
  
  frames.append(joints)  

  xp = []
  vect = fof.extractPoseFeature(frames)
  me=numpy.mean(vect)
  se=numpy.std(vect)
  vect[:]=[(p-float(me)/float(se)) for p in vect]
  xp.extend([vect])
  p_label, p_acc, p_val = svm_predict([0]*len(xp), xp, m, '-q')
 
#  print p_label 
#  print p_val
#  if p_label[0]==1.0:
#    print 'FRIENDLY'
#  else:
#    print 'HOSTILE'

  return p_val[0][0]


# MAIN METHOD
if __name__ == '__main__':
  global user
  
  user = 1
  # initialize tf_listener
  rospy.init_node('tf_listener')
  listener = tf.TransformListener()
  rate = rospy.Rate(10.0)

  # load classification model
  m = svm_load_model('heart_scale.model')
  
  # PR2 Sentry States
  NEUTRAL  = 0
  FRIENDLY = 1
  WHOSTILE = 2  
  SHOSTILE = 3
  # Initial state is neutral  
  state = NEUTRAL  
  pval = [0]*10
  s_pval = 0
  detected = False
  while not detected:
    try:
      listener.waitForTransform("/torso_"+str(user), "/openni_depth_frame", rospy.Time(), rospy.Duration(4.0))
      detected = True
      print 'Detected user, begin classifying'
    except (tf.Exception):
      print 'Unable to detect user ' + str(user) + ' searching for user ' + str(user+1)
      user = (user%10)+1
      continue 
  while not rospy.is_shutdown():
    try: 
      print 'Current State: ', STATES[state], 'pval = ', s_pval
      now = rospy.Time.now()
      listener.waitForTransform("/torso_"+str(user), "/openni_depth_frame", now, rospy.Duration(4.0))
      
      # constantly shift in confidence values, use their sums to determine state transitions
      pval[:-1] = pval[1:]
      pval[-1] =  classify_frame(listener,m)
      s_pval = sum(pval)
      if state == NEUTRAL:
        if s_pval > 9:
          state = FRIENDLY
          pval = [0]*10
        elif s_pval < -9:
          state = WHOSTILE
          pval = [0]*10
      elif state == FRIENDLY:
        if s_pval < -8:
          state = WHOSTILE
          pval = [0]*10
      elif state == WHOSTILE:     
        if s_pval > 9:
          state = NEUTRAL
          pval = [0]*10
        elif s_pval < -12:
          state = SHOSTILE
          pval = [0]*10
      else:
        if s_pval > 9:
          state = WHOSTILE 
          pval = [0]*10
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
      user = (user%10)+1

      print 'Lost user, now searching for user ' + str(user)
      continue
    
    rate.sleep()
