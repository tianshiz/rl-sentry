""" Tracking predictor

Using Kalman Filter, predicts the position of a target in the future
we dont know the current input(u), only have measurement z,
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#      print 'Lost user'

TODO Evaluate A,Q,H,R

TODO A should be learn rather then set by hand
TODO Does state need momentum (probably yes NdAF)?
"""
import roslib
roslib.load_manifest('ros_sentry')
import rospy
import tf
import os
import sys
import numpy
import time
import fof
from numpy import *
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

global past,dt

# array of joints to iterate over
JOINTS = ["head", "neck", "torso", "left_shoulder", "left_elbow", "right_shoulder", "right_elbow", "left_hip", "left_knee", "right_hip", "right_knee", "left_hand", "right_hand", "left_foot", "right_foot"]

def KalmanStep(x,p, z_past= [], step_future = 0):
    """ Perform Kalman filtering

    x,p are the intial conditions
    z_past is the sequence of measurement
    step_future is the number of iteration in the future
    """
    I = eye(4)

    A = KalmanStep.A
    Q = KalmanStep.Q 
    H = KalmanStep.H 
    R = KalmanStep.R 

    x_old=x[:,numpy.newaxis]
    p_old=p
    #for z in z_past:
        ## Time Update

    for z in z_past:
#        print z
        x_ = dot(A, x_old)
        p_ = dot(dot(A, p_old), A.T) + Q
        
    
            ## Measurement Update
        k = dot(dot(p_, H.T), linalg.inv(dot( dot(H, p_),H.T)+R) )
        p = dot(I - dot(k,H),p_)
#        z=numpy.array(z)[:,numpy.newaxis]
        
            #print dot(k,(z.T-dot(H,x_)))
            #z = dot(H,x_old) 
        x = x_ + dot(k,(z[:,numpy.newaxis]-dot(H,x_)))
        
        x_old=x
        p_old=p
    
    
    for t in xrange(step_future):
        ## Time Update
        
        x = dot(A, x_old)
        p = dot(dot(A, p_old), A.T) + Q
        
        x_old=x
        p_old=p


    return x[:,0],p

def predict(trajectory, t):
    """ Return predicted position at time t

    TODO what's the role of time? How to set the reprate?

    :trajectory: [time, (x,y,z)]
    :t:  float
    """
    ## Init kalman
    x = trajectory[0][1]
    p = diag((1,1,3))     ## TODO How we decide it?
    
    n_steps = (t - x[-1][0])*sample_rate

    x,p = KalmanStep(x,p, z_past = trajectory[:][1], step_future = n_steps)
    return x

def init():
    ## Init the parameters
    sample_rate = 10 

    KalmanStep.A = numpy.array([[1,0 ,.1, 0],[0, 1 ,0 ,.1],[0, 0 ,1 ,0],[0, 0 ,0 ,1]])

    KalmanStep.Q = eye(4) * 0.01
    KalmanStep.H = eye(4)
    KalmanStep.R = diag((0.1,0.1,.9,.9)) 

def testAnimation(fname):
  dt = .1
  frames = fof.loadPose(fname)
  trajectory = zeros((10,4))
  z_past=[]
  # Run it
#  with open(fname,'r') as f:
#    read_data=f.readlines()
#    for line in read_data:
#      line=line[:-1] #remove /n char
#      line=line.split(',') #convert toist
#      z_past.extend([[float(line[0]) ,float(line[1]) ,float(line[2]), float(line[3])]])
  
  for f in xrange(len(frames)):
    t_p = fof.extractRelPosition(frames[f])[1]
    if f==0:
      z_past.extend([[t_p[0],t_p[1],0,0]])
    else:
      t_p_ = fof.extractRelPosition(frames[f-1])[1]
      z_past.extend([[t_p[0],t_p[1],(t_p[0]-t_p_[0])/dt,(t_p[1]-t_p_[1])/dt]])

  # First set up the figure, the axis, and the plot element we want to animate
  fig = plt.figure()
  ax = plt.axes(xlim=(0, 4), ylim=(-2, 2))
  line, = ax.plot(trajectory[0], trajectory[1], lw=2)
  line2, = ax.plot(trajectory[0], trajectory[1],'r',lw=2)

  # initialization function: plot the background of each frame
  def init():
    line.set_data([], [])
    line2.set_data([], [])
    return line,line2,

  # animation function.  This is called sequentially
  def animate(i):
#    trajectory = z_past[i:i+10] 
    trajectory[:-1,:] = trajectory[1:,:]
    trajectory[-1,0:2] = z_past[0][0:2]
    trajectory[-1,2:] = z_past[0][2:]
    z_past.append(z_past.pop(0))
    # KEEP
    x = []
    for i in xrange(4,0,-1):
        x.append(KalmanStep(trajectory[-1], eye(4), trajectory[:-i])[0])
    for i in xrange(0,10):
        x.append(KalmanStep(trajectory[-1], eye(4), trajectory, i)[0])
    x=array(x)

    line.set_data(trajectory[:,0],trajectory[:,1])
    line2.set_data(x[:,0],x[:,1])
    return line,line2,

  # call the animator.  blit=True means only re-draw the parts that have changed.
  anim = animation.FuncAnimation(fig, animate, init_func=init, interval=100, blit=True)

#  anim.save('track_path.mp4', fps=10)

  plt.show()

def test3D(fname):
  frames = fof.loadPose(fname)
  poses=[]  
  pose = zeros((26,3))

  for f in frames:
    p=[]
    for i in [0,1,2,3,4,11,4,3,1,5,6,12,6,5,2,7,8,13,8,7,2,9,7,9,10,14]: 
      p.extend([[f[i][1][0], f[i][1][1], f[i][1][2]]])
    poses.extend([array(p)])
  
  # Attaching 3D axis to the figure
  fig = plt.figure()
  ax = p3.Axes3D(fig)
  line, = ax.plot(pose[:,0], pose[:,1], pose[:,2], lw=2)

  # initialization function: plot the background of each frame
  def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,

  def animate(i):
    pose = poses[0]
    line.set_data(pose[:,0],pose[:,1])
    line.set_3d_properties(pose[:,2])
    poses.append(poses.pop(0))  
    
    return line,
 
  # Setting the axes properties
  ax.set_xlim3d([0, 4.0]) 
  ax.set_ylim3d([-2.0, 2.0])
  ax.set_zlim3d([-1.0, 1.0])
  ax.view_init(0,-180)
  
  # Creating the Animation object
  anim = animation.FuncAnimation(fig, animate, init_func=init, interval=100, blit=True)
  
#  anim.save('track_skeleton.mp4', fps=10)
#  plt.axis('off')
  plt.show()

def getXY():
  global past, dt
  now = rospy.Time(0)
  ctime = time.time()
  dt = ctime - past
  past = ctime
  (position,quaternion) = listener.lookupTransform("/openni_depth_frame", "/torso_1", now)
  x,y,z = position
  return (x,y)

def getPose():
  jpos = []
  now = rospy.Time(0)
  for j in JOINTS:
    (position,quaternion) = listener.lookupTransform("/openni_depth_frame", "/"+j+"_1", now)
    x,y,z = position
    jpos.extend([[x, y, z]])
  p = []
  for i in [0,1,2,3,4,11,4,3,1,5,6,12,6,5,2,7,8,13,8,7,2,9,7,9,10,14]: 
    p.extend([[jpos[i][0], jpos[i][1], jpos[i][2]]])

  return array(p)

def trackPath():
  trajectory = zeros((10,4))
  
  # First set up the figure, the axis, and the plot element we want to animate
  fig = plt.figure()
#  ax = plt.axes(xlim=(0, 4), ylim=(-2, 2))
  ax = plt.axes(xlim=(-2, 2), ylim=(-4, 0))
  line, = ax.plot(trajectory[0], trajectory[1], lw=2)
  line2, = ax.plot(trajectory[0], trajectory[1],'r',lw=2)

  # initialization function: plot the background of each frame
  def init():
    global past, dt
    past = 0
    dt = 0
    line.set_data([], [])
    line2.set_data([], [])
    return line,line2,

  # animation function.  This is called sequentially
  def animate(i):
    global dt

    try:
      now = rospy.Time.now()
      listener.waitForTransform("/torso_1", "/openni_depth_frame", now, rospy.Duration(4.0))
      coords = getXY()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Lost User'

    trajectory[:-1,:] = trajectory[1:,:]
    if i==1:   
#      trajectory[-1,0] = coords[0]
#      trajectory[-1,1] = coords[1]
      trajectory[-1,0] = coords[1]
      trajectory[-1,1] = -coords[0]
      trajectory[-1,2] = 0
      trajectory[-1,3] = 0
    else:
#      trajectory[-1,0] = coords[0]
#      trajectory[-1,1] = coords[1]
#      trajectory[-1,2] = (coords[0]-trajectory[-2,0])/dt
#      trajectory[-1,3] = (coords[1]-trajectory[-2,1])/dt
      trajectory[-1,0] = coords[1]
      trajectory[-1,1] = -coords[0]
      trajectory[-1,2] = (coords[1]-trajectory[-2,0])/dt
      trajectory[-1,3] = (-coords[0]-trajectory[-2,1])/dt
    
    # KEEP
    x = []
    for j in xrange(4,0,-1):
        x.append(KalmanStep(trajectory[-1], eye(4), trajectory[:-j])[0])
    for j in xrange(0,10):
        x.append(KalmanStep(trajectory[-1], eye(4), trajectory, j)[0])
    x=array(x)

    line.set_data(trajectory[:,0],trajectory[:,1])
    line2.set_data(x[:,0],x[:,1])
    return line,line2,

  # call the animator.  blit=True means only re-draw the parts that have changed.
  anim = animation.FuncAnimation(fig, animate, init_func=init, interval=100, blit=True)

  plt.show()

def trackSkeleton():
  pose = zeros((26,3))

  # Attaching 3D axis to the figure
  fig = plt.figure()
  ax = p3.Axes3D(fig)
  line, = ax.plot(pose[:,0], pose[:,1], pose[:,2], lw=2)

  # initialization function: plot the background of each frame
  def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,

  def animate(i):
    try:
      now = rospy.Time.now()
      listener.waitForTransform("/torso_1", "/openni_depth_frame", now, rospy.Duration(4.0))
      pose = getPose()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Lost User'
    line.set_data(pose[:,0],pose[:,1])
    line.set_3d_properties(pose[:,2])

    return line,
 
  # Setting the axes properties
  ax.set_xlim3d([0, 4.0]) 
  ax.set_ylim3d([-2.0, 2.0])
  ax.set_zlim3d([-1.0, 1.0])
  ax.view_init(0,-180)
  
  # Creating the Animation object
  anim = animation.FuncAnimation(fig, animate, init_func=init, interval=100, blit=True)
  
  plt.show()

if __name__ == '__main__':
  # Init node
  init()
  testAnimation('../data/lookaround2.txt')    
#  test3D('../data/creep1.txt')
#  rospy.init_node('tf_listener')
#  listener = tf.TransformListener()
#  rate = rospy.Rate(10.0)
#  try:
#    listener.waitForTransform("/torso_1", "/openni_depth_frame", rospy.Time(), rospy.Duration(4.0))
#    print 'Detected user, begin tracking'
#  except (tf.Exception):
#    print 'Unable to detect user'
#  trackPath()
#  trackSkeleton()
