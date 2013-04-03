#!/usr/bin/env python
""" Friend or Foe

Behaviour predictor
- trains using given set

"""
import roslib
roslib.load_manifest('ros_sentry')

from numpy import array
import PyKDL
from matplotlib import pyplot
import pylab
from mpl_toolkits.mplot3d import Axes3D

## first rot + pos, only pos
joint_n = (11,4)
torso_j = 2
l_shoulder = 3
r_shoulder = 5
l_hip = 7
r_hip = 9


def loadPose(pose_filename):
    """ 
    Frame#,ORI(1),P(1),ORI(2),P(2),...,P(11),J(11),P(12),...,P(15)
    
    Frame# => integer starting from 1
    ORI(i) => orientation of ith joint
                0 1 2
                3 4 5
                6 7 8
              3x3 matrix is stored as followed by CONF
                0,1,2,3,4,5,6,7,8,CONF
    P(i)   => position of ith joint followed by CONF
                x,y,z,CONF  """

    pose_file = open(pose_filename,'r')

    frames = []

    for line in pose_file.readlines():
        if line == "END":
            break
        joints = [[PyKDL.Rotation(1,0,0,0,1,0,0,0,1),
                   PyKDL.Vector(0,0,0)]]*15
        data = array([float(v) for v in line.strip("\n").split(',')])
        
        index = 1
        for j in xrange(joint_n[0]):
            joints[j]=[PyKDL.Rotation(*data[index:index+9]),  # rotation matrix
                       PyKDL.Vector(*data[index+10:index+13])]              # position
            index = index + 14
        for j in xrange(joint_n[0], joint_n[0]+joint_n[1]):
            joints[j]=[PyKDL.Rotation(1,0,0,0,1,0,0,0,1),
                       PyKDL.Vector(*data[index:index+3])]              # position
            index = index + 4

        frames.append(joints)  ## int(data[0])
    return frames

def whiten(frame):
    """ Translate the frame to local axis and orientate
    """
    r0,p0 = extractRelPosition(frame)

    new_frame = [0]*len(frame)
    for j,(j_r,j_p) in enumerate(frame):
        new_frame[j] = [j_r, r0*(j_p - p0)]
    return new_frame

def extractRelPosition(frame):
    """ Extract the relative position and orientation of the frame
    
    Uses Torso (could be more complex)
    """
    return frame[torso_j]

def extractAvgPosition(frame):

    x_avg = (frame[torso_j][1][0]+frame[l_shoulder][1][0]+frame[r_shoulder][1][0]+frame[l_hip][1][0]+frame[r_hip][1][0])/5.0
    y_avg = (frame[torso_j][1][1]+frame[l_shoulder][1][1]+frame[r_shoulder][1][1]+frame[l_hip][1][1]+frame[r_hip][1][1])/5.0
    z_avg = (frame[torso_j][1][2]+frame[l_shoulder][1][2]+frame[r_shoulder][1][2]+frame[l_hip][1][2]+frame[r_hip][1][2])/5.0

    return (x_avg, y_avg, z_avg)

def extractPoseFeature(frames):
    """ Extract the pose feature 

    Returns an array of the positions of each joint"""
    feature = []

    ## TODO possibly slow assignment
    for f in frames:
        for j_r, j_p in whiten(f):
            feature.extend( (j_p[0], j_p[1], j_p[2]) )

    return feature


def training(train_set):
    pass

def addSkeleton(frame):
    x = []
    y = []
    z = []
    for i in [0,1,2,3,4,11,4,3,1,5,6,12,6,5,2,7,8,13,8,7,2,9,7,9,10,14]: 
      print frame[i][1][0], frame[i][1][1], frame[i][1][2] 
      x.append(frame[i][1][0])
      y.append(frame[i][1][1])
      z.append(frame[i][1][2])
     
    pylab.gca().plot(x, y, z, marker='o')

def showSkeletons(frames):
    fig = pylab.figure()
    ax = Axes3D(fig)

    for f in frames:
      addSkeleton(f)

    ax.set_xlim(0,3)
    ax.set_ylim(-1,1)
    ax.view_init(0,-180)
    pyplot.show()

def addPath(frames):
    x = []
    y = []
    z = []
    for f in frames:
      t_p = extractRelPosition(f)[1]
      x.append(t_p[0])
      y.append(t_p[1])
      z.append(t_p[2])

    pylab.gca().plot(x, y, z, marker='o')

def trackPaths(frames):

    fig = pylab.figure()
    ax = Axes3D(fig)

    for f in frames:
      addPath(f)  
 
    ax.set_xlim(0,3)
    ax.set_ylim(-1,1)
    ax.view_init(90,-180)
    pyplot.show()

def saveTrajectory(pose_filename):
    dt = .1
    frames = loadPose(pose_filename)
    
    traj_filename = pose_filename[:-4]+'_path.txt'
    traj = open(traj_filename,'w')
    traj.close()
    traj = open(traj_filename,'a') 

    for f in xrange(len(frames)):
      t_p = extractRelPosition(frames[f])[1]
      if f==0:
        traj.write("%f,%f,%f,%f\n"%(t_p[0],t_p[1],0,0))
      else:
        t_p_ = extractRelPosition(frames[f-1])[1]
        traj.write("%f,%f,%f,%f\n"%(t_p[0],t_p[1],(t_p[0]-t_p_[0])/dt,(t_p[1]-t_p_[1])/dt))

def test():
    f = loadPose('../data/approach1.txt')
    f2 = loadPose('../data/approach2.txt')
    f3 = loadPose('../data/approach3.txt')
    f4 = loadPose('../data/approach4.txt')
    f5 = loadPose('../data/approach5.txt')
    f6 = loadPose('../data/approach6.txt')
    #f2 = extractPoseFeature(f[:2])
    #print f2
    #print len(f2)
    showSkeletons([f[00],f2[0],f3[0],f4[0],f5[0],f6[0]])
#    saveTrajectory('../data/stand1.txt')
#    trackPaths([f,f2,f3,f4,f5,f6])
#    trackPaths([f])

test()

