#!/usr/bin/env python
""" frameExtract

Extracts requested information from a given frame
"""
import roslib; roslib.load_manifest('ros_sentry')
import rospy
import tf
import sys
import PyKDL
from PyKDL import *

# array of joints to iterate over
JOINTS = ["head", "neck", "torso", "left_shoulder", "left_elbow", "right_shoulder", "right_elbow", "left_hip", "left_knee", "right_hip", "right_knee", "left_hand", "right_hand", "left_foot", "right_foot"]

## first rot + pos, only pos
joint_n = (11,4)
torso_j = 2

def grabFrame(listener, user, time = None):
  """ 
  Return a frame of joint positions and orientations

  ORI(1),P(1),ORI(2),P(2),...,P(11),J(11),P(12),...,P(15)
    
  ORI(i) => orientation of ith joint
	0 1 2
	3 4 5
	6 7 8
  3x3 matrix is stored as follows
  0,1,2,3,4,5,6,7,8

  P(i)   => position of ith joint
  x,y,z """

  if time is None:
      time = rospy.Time(0)

  #frame = [[Rotation(1,0,0,0,1,0,0,0,1), Vector(0,0,0)]]*15
  frame = []
  i = 1

  for j in JOINTS:
    (position,quaternion) = listener.lookupTransform("/openni_depth_frame", "/"+j+"_"+str(user), time)
    x,y,z = position
    Qx,Qy,Qz,Qw = quaternion
    f = Frame(Rotation.Quaternion(Qx,Qy,Qz,Qw),Vector(x,y,z)) 

    if i<12:
      frame.append([Rotation(f.M[0,0], f.M[0,1], f.M[0,2], f.M[1,0], f.M[1,1], f.M[1,2], f.M[2,0], f.M[2,1], f.M[2,2]), Vector(x,y,z)])
    else:
      frame.append([Rotation(1,0,0,0,1,0,0,0,1), Vector(x,y,z)])

    i += 1

  return frame
def whiten(frame):
  """ Translate the frame to local axis and orientate
  """
  r0,p0 = extractRelPosition(frame)

  new_frame = [0]*len(frame)
  for j,(j_r,j_p) in enumerate(frame):
    new_frame[j] = [j_r, r0*(j_p - p0)]
  return new_frame

def extractRelPosition(frame):
  """ Extract the relative position of the frame
    
  Uses Torso (could be more complex)
  """
  return frame[torso_j]

def extractXYPosition(frame):
  """ Extract the relative position of the frame
    
  Uses Torso (could be more complex)
  """
  return frame[torso_j][1][0], frame[torso_j][1][1]


def extractPoseFeatures(frame):
  """ Extract the pose features 

  Returns an array of the positions of each joint"""
  feature = []

  ## TODO possibly slow assignment
  for j_r, j_p in whiten(frame):
    feature.extend( (j_p[0], j_p[1], j_p[2]) )

  return feature

def test():
  f2 = extractPoseFeature(f[:2])
  print f[:2]

