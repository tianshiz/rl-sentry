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

def create_goal(joints_p):
    """ Create a goal for a given set of joints angles """
    goal = JointTrajectoryGoal()
    # Populates trajectory with joint names.
    goal.trajectory.joint_names.append("r_shoulder_pan_joint")
    goal.trajectory.joint_names.append("r_shoulder_lift_joint")
    goal.trajectory.joint_names.append("r_upper_arm_roll_joint")
    goal.trajectory.joint_names.append("r_elbow_flex_joint")
    goal.trajectory.joint_names.append("r_forearm_roll_joint")
    goal.trajectory.joint_names.append("r_wrist_flex_joint")
    goal.trajectory.joint_names.append("r_wrist_roll_joint")
    
    # First trajectory point
    # Positions
    ind = 0
    #print goal.trajectory.points
    point1 = JointTrajectoryPoint()
    point2 = JointTrajectoryPoint()
    goal.trajectory.points = [point1]
    #goal.trajectory.points[ind].positions.resize(7)
    point1.positions = joints_p
    point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    
    #To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = rospy.Duration(2.0)

    goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(2.0)        
    return goal

def listener():
    rospy.init_node('mastermind', anonymous=True)
    l_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    l_client.wait_for_server()
    l_client.send_goal(create_goal([0.0,0.0,0.0,0.0,0.0,0.0,0.0]))
    #l_client.send_goal(create_goal([-0.3,0.2,-0.1,-1.2,1.5,-0.3,0.5]))
    print l_client
    # Goal
   

    #rospy.Subscriber("chatter", String, callback)
    
    #rospy.spin()


if __name__ == '__main__':
    ## Get List of Arms Joints
    print rospy.get_param('/r_arm_controller/joints')


    listener()