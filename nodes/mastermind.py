#!/usr/bin/env python
""" Mastermind.py

sentrybot
"""
import roslib; roslib.load_manifest('ros_sentry')
import rospy
from std_msgs.msg import String
import actionlib
import time
import tf
import sys
import poseClassify
import frameExtract
import trackPredict
import numpy
sys.path.insert(0,'../libsvm-3.17/python')
from svmutil import *

## RViz
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

def createMarkerLine(pos_list, color = (.1, .1, 1), ID = 0, alpha = 1., size = 0.5):
    marker = Marker()
    marker.header.frame_id = "/openni_depth_frame"
    marker.id = ID
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = alpha

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    for p in pos_list:
        marker.points.append( Point(p[0],p[1],0) )

    return marker

#state machine method that makes high level decisions
def stateMachine(state):
    global fof #state flags
    #state definitions
    START=0
    END=1
    STANDBY=2
    WAVE=3
    SHOOT=4
    AIM=5
    #in this state machine only fof should be modified outside the state machine!!
    if state==START:
        #brief startup state to init some flags, we never come back here
        state=STANDBY #go to standby
        fof=1 #fof can be 0 for friendly, 1 for neutral, 2 for foe
    elif state==END:
        pass
    elif state==STANDBY:
        #observation mode to identify people
        if fof>=1:
            #if not friendly, aim gun
            state=AIM    
        else:
            #friendly, go to wave to id
            state=WAVE
            #waveHand()

    elif state==WAVE:
        #when the robot beckons you to id yourself. stand still and face the robot
        if fof==2:
            state=AIM
        #nothing happens as long as u remain friendly or neutral

    elif state==SHOOT:
        if fof==0:
            #friendly
            state=STANDBY
        elif fof==1:
            #target is neutral, but do not let your guard down!
            state=AIM
        else:
           # shoot(trajectory,1) #publish trajector and shoot command(1) to arm_node
            state=AIM #go back to aiming
    elif state==AIM:
        #keep aiming at the person
        if fof==2:
            #shoot if foe!
            state=SHOOT
        elif fof==0:
            #go on standby knowing the person is friendly
            state=STANDBY
        else:
            #shoot(trajectory,0)
            pass

        #otherwise we remain aiming if neutral
    else:
        raise ValueError, "unexpected state "+state

    return state

def init():
    global listener
    global trajectory_topic, pred_trajectory_topic
    global m
    global base_frame
    global user

    # initialize tf_listener
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()

    # initialize trajectory publishers
    trajectory_topic = rospy.Publisher("trajectory_target", Marker)
    pred_trajectory_topic = rospy.Publisher("pred_trajectory_target", Marker)

    # load SVM classification model
    m = svm_load_model('heart_scale.model')

    # set base frame
    base_frame = "/openni_depth_frame"

    # set user to track
    user = 1

    # sleep
    rospy.sleep(1)

if __name__ == '__main__':
    global fof 
    global listener
    global trajectory_topic, pred_trajectory_topic
    global m
    global base_frame
    global user

    init()

    START=0
    END=1
    STANDBY=2
    WAVE=3
    SHOOT=4
    AIM=5

    state=START #initial state
    targetTraj=[]
    trajHistory=numpy.zeros((10,4))
    start=1

    # Initial state is neutral  
    posestate = 1 
    pval = [0]*10
    s_pval = 0
    sampling_time = .1
    past = rospy.Time.now()# + rospy.Duration(sampling_time)
    init = True
    detected = False
    while not detected:
        try:
            listener.waitForTransform("/torso_"+str(user), base_frame, past, rospy.Duration(2.0))
            detected = True
            print 'Detected user ' + str(user) + ', begin running'
            past = rospy.Time.now()
        except (tf.Exception):
            print 'Unable to detect user, searching for user ' + str((user%10)+1)
            past = rospy.Time.now()# + rospy.Duration(sampling_time)
            user = (user%10)+1
            continue 

    #forever loop
    while True:
        if start==0:
        #mastermind should not run
            state=END
        else:
        #run likenormal
            
            ## Mainloop
            now = rospy.Time.now()
#            print (now-past).to_sec()
            while (now-past).to_sec() > sampling_time:
                try:
                    #grab a frame periodically
                    listener.waitForTransform("/torso_"+str(user), base_frame, past, rospy.Duration(2.0))

                    #call this getFrames function, made from parts of tf-listener
                    frame=frameExtract.grabFrame(listener, user, time = past) 
                    past = past + rospy.Duration(sampling_time)
                    #print time, past, dt
                    #past = time
                    x,y = frameExtract.extractXYPosition(frame)
                

                    trajHistory[:-1,:] = trajHistory[1:,:]
                    if init:
                        trajHistory[-1,0] = x
                        trajHistory[-1,1] = y            
                        trajHistory[-1,2] = 0
                        trajHistory[-1,3] = 0
                        init = False
                    else:
                        trajHistory[-1,0] = x
                        trajHistory[-1,1] = y            
                        trajHistory[-1,2] = (x-trajHistory[-2,0])/sampling_time
                        trajHistory[-1,3] = (y-trajHistory[-2,1])/sampling_time

                    ## Visualize trajectory
                    marker = createMarkerLine( pos_list = trajHistory, color = (1., 0.1, 0.),
                                  ID = i, size = 0.02 )
                    trajectory_topic.publish(marker)

                    #with frames, get predicted trajectory, a string of points in trajector
                    targetTraj = trackPredict.predict(trajHistory,10)

                    # Visualize predicted trajectory
                    marker2 = createMarkerLine( pos_list = targetTraj, color = (0., 0.1, 1.),
                                  ID = i, size = 0.02 )
                    pred_trajectory_topic.publish(marker2)

                    #update current pose state
                    posestate, pval, s_pval=poseClassify.getClass(frame,m,posestate,pval,s_pval)

                    #there are two returns for hostile, we count them the same
                    if posestate==3 or posestate==2:
                        fof=2
                    else:
                        #update global fof variable for statemachine
                        fof=posestate
                except tf.Exception as e:
                    #print e
                    past = rospy.Time.now()
                    now = rospy.Time.now()
                    user = (user%10)+1

                    print 'Lost user, now searching for user ' + str(user)
                    continue
            
#            print 'Current State'
#            print trajHistory
#            print 'Predcition'
#            print targetTraj

            #publish trajectory to rostopics

            #get next state
            state=stateMachine(state)

#            rospy.sleep(.05)