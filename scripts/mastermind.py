#!/usr/bin/env python
""" Mastermind.py

sentrybot
"""

import roslib; roslib.load_manifest('ros_sentry')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)

def create_goal(joints_p, arm_side = 'r'):
    """ Create a goal for a given set of joints angles """
    goal = JointTrajectoryGoal()
    # Populates trajectory with joint names.
    goal.trajectory.joint_names.append(arm_side + "_shoulder_pan_joint")
    goal.trajectory.joint_names.append(arm_side + "_shoulder_lift_joint")
    goal.trajectory.joint_names.append(arm_side + "_upper_arm_roll_joint")
    goal.trajectory.joint_names.append(arm_side + "_elbow_flex_joint")
    goal.trajectory.joint_names.append(arm_side + "_forearm_roll_joint")
    goal.trajectory.joint_names.append(arm_side + "_wrist_flex_joint")
    goal.trajectory.joint_names.append(arm_side + "_wrist_roll_joint")
    
    goal.trajectory.points = []

    # Positions
    for ind in xrange(len(joints_p)):
        point = JointTrajectoryPoint()

        point.positions = joints_p[ind]
        point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        goal.trajectory.points.append(point)
        #To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = rospy.Duration(1.5)*ind

    goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(.5)        
    return goal


## Encode wave
wave = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        [0.1, 0.3,  0.0, -1.8,  0.0,  0.0,   0.3],
        [0.1, 0.2, 0.0, -0.8, 0.0, 0.0, 0.0],
        [0.1, 0.3,  0.0, -1.8,  0.0,  0.0,   0.3],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]


def waver():
    rospy.init_node('mastermind', anonymous=True)
    l_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    l_client.wait_for_server()
    l_client.send_goal(create_goal(wave, 'l'))
    #l_client.send_goal(create_goal([-0.3,0.2,-0.1,-1.2,1.5,-0.3,0.5]))
    print l_client
    # Goal
   

    #rospy.Subscriber("chatter", String, callback)
    
    #rospy.spin()


if __name__ == '__main__':
    ## Get List of Arms Joints
    print rospy.get_param('/r_arm_controller/joints')


    waver()