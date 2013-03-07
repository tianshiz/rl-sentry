#!/usr/bin/env python
""" Friend or Foe

Behaviour predictor
- trains using given set

"""

from numpy import array

## first rot + pos, only pos
joint_n = (11,4)


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
        joints = [[0,0]]*15
        data = array([float(v) for v in line[:-2].split(',')])
        
        index = 1
        for j in xrange(joint_n[0]):
            joints[j][0] = data[index:index+9].reshape((3,3))  # rotation matrix
            joints[j][1] = data[index+9:index+13]              # position
            index = index + 14
        for j in xrange(joint_n[0], joint_n[1]):
            joints[j][1] = data[index:index+3]              # position
            index = index + 4

        frames.append(joints)  ## int(data[0])
    return frames

def training(train_set):
    pass

## Test
f = loadPose('../data/0510175431.txt')
print f[0][1] 