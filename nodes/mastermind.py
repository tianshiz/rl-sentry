#!/usr/bin/env python
""" Mastermind.py

sentrybot
"""
import roslib; roslib.load_manifest('rl-sentry')
import rospy
from std_msgs.msg import String
import actionlib
import time
import tf
import sys
import poseClassify
sys.path.insert(0,'../libsvm-3.17/python')
#subscribe to rostopic to start/stop the master
#mastermind node responds once receiving message from publisher
def callback(data):
    global start
    #rospy.loginfo("I heard %s",data.data)
    if data.data=="1":
        start=1
        #print 'done'
    else:
        start=0
    
    #returns start flag

#listener for topic subsciber    
def listener():
    rospy.init_node('master_node')
    pub = rospy.Publisher('arm_node', String)
    rospy.Subscriber("start_master", String, callback)
    # spin() simply keeps python from exiting until this node is stopped

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
            waveHand()

    elif state==WAVE:
        #when the robot beckons you to id yourself. stand still and face the robot
        if fof==2:
            state=AIM
        #nothing happens as long as u remain friendly or neutral

    elif state==SHOOT:
        if fof==0:m
            #friendly
            state=STANDBY
        elif fof==1:
            #target is neutral, but do not let your guard down!
            state=AIM
        else:
            shoot(trajectory,1) #publish trajector and shoot command(1) to arm_node
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
            shoot(trajectory,0)

        #otherwise we remain aiming if neutral
    else:
        raise ValueError, "unexpected state "+state

    return state

def init():
  # initialize tf_listener
  rospy.init_node('tf_listener')
  listener = tf.TransformListener()
  rate = rospy.Rate(10.0)	

  # load classification model
  m = svm_load_model('heart_scale.model')

if __name__ == '__main__':
    global fof 
    #load svm model
    m = svm_load_model('heart_scale.model')
    START=0
    END=1
    STANDBY=2
    WAVE=3
    SHOOT=4
    AIM=5

    state=START #initial state

    start=0
    #intialize subscriber
    listener() 
    #record current time
    last_time=time.time()
    #forever loop
    while True:
        if start==0:
        #mastermind should not run
            print 'nope!'
            state=END
        else:
        #run likenormal
            print 'ok!'
            
            ## Mainloop
            
            #tf-listener update periodically(once every secs) and get sequence of frames
          
            last_time=curr_time
            #call this getFrames function, made from parts of tf-listener
            frames=getFrames() #most recent frame is 

            #with the frames, get gesture prediction(sentry states)
            #fof can be 0 for friendly, 1 for neutral, 2 for foe
            fof=poseClassify.classify_frame(frames,m) 
            #there are two returns for hostile, we count them the same
            if fof==3:
                fof==2


            #with frames, get predicted trajectory, a string of points in trajector
            trajectory=predictPath(frames)
            #get next state
            state=stateMachine(state)
