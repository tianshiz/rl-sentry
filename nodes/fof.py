#!/usr/bin/env python
""" Friend or Foe

Behaviour predictor
- trains using given set

"""
import roslib
roslib.load_manifest('rl-sentry')
import sys
from array import array
import PyKDL

## first rot + pos, only pos
joint_n = (11,4)
torso_j = 2

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

        data = array('f',[float(v) for v in line[:-3].split(',')])
        
        index = 1
        for j in xrange(joint_n[0]):
            joints[j]=[PyKDL.Rotation(*data[index:index+9]),  # rotation matrix
                       PyKDL.Vector(*data[index+10:index+13])]              # position
            index = index + 14
        for j in xrange(joint_n[0], joint_n[1]):
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

def test():
    f = loadPose('friendly_1.txt')

    f2 = extractPoseFeature(f[:2])
    print f[:2]
    #print len(f2)

#test()
