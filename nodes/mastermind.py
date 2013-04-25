#!/usr/bin/env python
""" Mastermind.py

sentrybot
"""

import roslib; roslib.load_manifest('rl_sentry')
import rospy
from std_msgs.msg import String
import actionlib

start=0
#subscribe to rostopic to start/stop the master
#mastermind node responds once receiving message from publisher
def callback(data):
    rospy.loginfo("I heard %s",data.data)
    if data.data=='1':
        start=1
    else:
        start=0
    end
    #returns start flag
    return start
#listener for topic subsciber    
def listener():
    rospy.init_node('start_command')
    rospy.Subscriber("mastermind", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

while True:
    if start==0:
    #initially the mastermind does not start
        listener() 
    else:
        print 'ok!'
    end
    ## Mainloop
    
    #tf-listener update periodically and get sequence of frames
    #frames=getFrames()

    #
    ## If not alerted
        ## track humans
        ## choose one still neutral and predict fof on him
        ## if friend identify him
        ## if foe get allerted
        ## if neutral invite or idle
    ## if alerted
        ## shoot and verify change of behaviour

if __name__ == '__main__':
    ## Init ros node
    ## Init all the systems
    pass

