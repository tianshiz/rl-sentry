#!/usr/bin/env python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('mastermind', String)
rospy.init_node('start_command')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
   pub.publish("1")
   r.sleep()