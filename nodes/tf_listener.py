#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_sentry')
import rospy
import tf
import curses

from PyKDL import *

# array of joints to iterate over
JOINTS = ["head", "neck", "torso", "left_shoulder", "left_elbow", "right_shoulder", "right_elbow", "left_hip", "left_knee", "right_hip", "right_knee", "left_hand", "right_hand", "left_foot", "right_foot"]

# global tail variables for files
f_tail = 1
h_tail = 1

#-------------------------------------------------------------
# append an individual frame of joint information to file
#-------------------------------------------------------------
# params:
#   behav    - type of behavior, hostile or friendly
#   frame    - current frame
#   win      - window for displaying text
#   listener - used to find joint positions and orientations
#   tail     - appended to filename to distinguish files
#-------------------------------------------------------------
def save_frame(behav, frame, win, listener, tail):
  frame_string = "%d"%frame
  now = rospy.Time(0)
  
  # iterate over joints to find their position and orientation in current frame
  for j in JOINTS:
    (position,quaternion) = listener.lookupTransform("/"+j+"_1", "/openni_depth_frame", now)
    x,y,z = position
    Qx,Qy,Qz,Qw = quaternion
    f = Frame(Rotation.Quaternion(Qx,Qy,Qz,Qw),Vector(x,y,z))    
    
    # append orientation and position data for current joing to frame_string
    frame_string += ",%f,%f,%f,%f,%f,%f,%f,%f,%f,1"%(f.M[0,0], f.M[0,1], f.M[0,2], f.M[1,0], f.M[1,1], f.M[1,2], f.M[2,0], f.M[2,1], f.M[2,2])
    frame_string += ",%f,%f,%f,1\n"%(position)
  
  # append frame_string to end of file
  f = open('%s_%d.txt'%(behav,tail), 'a')
  f.write(frame_string)
  f.close()
  win.clear()
  win.addstr(0,0, "Recording %s behavior, press <space> to stop recording"%behav)
  win.addstr(1,0, frame_string)

#-------------------------------------------------------------
# record a sequence of frames for a selected type of behavior
#-------------------------------------------------------------
# params:
#   key      - pressed key, determines type of behavior
#   win      - window for displaying text
#   listener - used to find joint positions and orientations
#   rate     - sampling rate
#-------------------------------------------------------------
def record(key, win, listener, rate):
  global f_tail
  global h_tail
  frame = 0

  # depending on pressed key, set and increment corresponding tail to allow for multiple recordings
  if key == 'h':
    behav = "hostile"
    tail = h_tail
    h_tail += 1
  else:
    behav = "friendly"
    tail = f_tail
    f_tail += 1

  # initialize a new file to write joint information to
  f = open('%s_%d.txt'%(behav,tail), 'w')
  f.close()
  
  # write frames of joint information to file until recording is stopped or user is lost
  while not rospy.is_shutdown():
    try:
      now = rospy.Time.now()
      listener.waitForTransform("/torso_1", "/openni_depth_frame", now, rospy.Duration(4.0))
      save_frame(behav,frame,win,listener,tail)
      frame += 1      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      win.clear()
      win.addstr(0,0,"Lost user")
      break
    
    # detect keypress
    try:
      key = win.getkey()
    except:
      key = None
    # if <space> is pressed, break out of loop
    if key == " ":
      win.clear()
      break
    rate.sleep()

#-------------------------------------------------------------
# main loop, allow user to make multiple recordings of either
# hostile or friendly behavior
#-------------------------------------------------------------
# params:
#   win - window for displaying text
#-------------------------------------------------------------
def main(win):
  win.nodelay(True)
    
  win.clear() 
  win.addstr(0,0,'starting tf_listener...')

  # initialize tf_listener
  rospy.init_node('tf_listener')
  listener = tf.TransformListener()
  rate = rospy.Rate(10.0)
  
  # main loop
  while True:
    try:
      listener.waitForTransform("/torso_1", "/openni_depth_frame", rospy.Time(), rospy.Duration(4.0))
      win.addstr(0,0,"Detected user, begin tracking")
      win.addstr(1,0,"USAGE:                             ")
      win.addstr(2,3, "press <h> to begin recording hostile behavior") 
      win.addstr(3,3, "press <f> to begin recording friendly behavior") 
      win.addstr(4,3, "press <q> to quit tf_listener") 
    except (tf.Exception):
      win.clear()
      win.addstr(0,0,"Unable to detect user")
      win.addstr(1,3, "press <q> to quit tf_listener") 

    # detect keypress 
    try:
      key = win.getkey()
    except:
      key = None
    # if <f> or <h> pressed, begin recording frames for corresponding behavior
    if key == "f" or key == "h":
      record(key, win, listener, rate)
    # if <q> pressed, exit
    elif key == "q":
      win.clear()
      win.addstr(0,0,"exiting tf_listener...")
      break

if __name__ == '__main__':
  curses.wrapper(main)

