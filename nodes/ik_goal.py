#!/usr/bin/env python
import roslib; 	roslib.load_manifest('drive_base_tutorial')
import rospy
import actionlib

from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal, SimplePoseConstraint,\
                               PositionConstraint, OrientationConstraint


def poseConstraintToPositionOrientationConstraints(pose_constraint):
    position_constraint = PositionConstraint()
    orientation_constraint = OrientationConstraint()
    position_constraint.header = pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position
    position_constraint.constraint_region_shape.type = 0
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0
	 
    return (position_constraint, orientation_constraint)

def addGoalConstraintToMoveArmGoal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = poseConstraintToPositionOrientationConstraints(pose_constraint);
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

def createHandPose(pos, orientation, hand = 'r'):
    """ Create the final pose contraint for the hand
    """
    desired_pose = SimplePoseConstraint();
    desired_pose.header.frame_id = "torso_lift_link"
    desired_pose.link_name = hand + "_wrist_roll_link"

    desired_pose.pose.position.x = pos[0]
    desired_pose.pose.position.y = pos[1]
    desired_pose.pose.position.z = pos[2]

    desired_pose.pose.orientation.x = orientation[0]
    desired_pose.pose.orientation.y = orientation[1]
    desired_pose.pose.orientation.z = orientation[2]
    desired_pose.pose.orientation.w = orientation[3]

    desired_pose.absolute_position_tolerance.x = 0.02
    desired_pose.absolute_position_tolerance.y = 0.02
    desired_pose.absolute_position_tolerance.z = 0.02

    desired_pose.absolute_roll_tolerance = 0.04
    desired_pose.absolute_pitch_tolerance = 0.04
    desired_pose.absolute_yaw_tolerance = 0.04

    return desired_pose

def createIKGoal(pos, orientation, hand);
    """ Create the message for the IK service 
    """

    goalA = MoveArmGoal()
    goalA.motion_plan_request.group_name = hand + "_arm"
    goalA.motion_plan_request.num_planning_attempts = 1
    goalA.motion_plan_request.planner_id = ""
    goalA.planner_service_name = "ompl_planning/plan_kinematic_path"
    goalA.motion_plan_request.allowed_planning_time = rospy.Duration.from_sec(5.0);

    ## Set desired pose  
    desired_pose(pos, orientation, hand)
    ## Associate pose to goal
    addGoalConstraintToMoveArmGoal(desired_pose, goalA)
    return goalA


if __name__ == '__main__':
    rospy.init_node('move_arm_test')
    ## Start Movement Client
    move_arm = actionlib.SimpleActionClient('right_arm', MoveArmAction)
    move_arm.wait_for_server()
    move_arm.send_goal(goalA)

