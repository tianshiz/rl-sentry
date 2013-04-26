# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


import roslib
roslib.load_manifest('rl-sentry')

import rospy
import actionlib

from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.msg import OrientationConstraint
from actionlib_msgs.msg import GoalStatus


if __name__ == '__main__':

    rospy.init_node('arm_cartesian_goal_sender')

    move_arm = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
    move_arm.wait_for_server()
    rospy.logout('Connected to server')

    goalA = MoveArmGoal()
    goalA.motion_plan_request.group_name = 'right_arm'
    goalA.motion_plan_request.num_planning_attempts = 1
    goalA.motion_plan_request.planner_id = ''
    goalA.planner_service_name = 'ompl_planning/plan_kinematic_path'
    goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.)


    '''
    Each position constraints is specified with a header, a link name, 
    the goal position that we want the link to reach and a tolerance region specified 
    using a geometric_shapes/Shape message. In this case we are trying to move the end-effector 
    link (r_wrist_roll_link) to the position (0.75,-0.188,0) in the torso_lift_link frame. 
    Note also that we fill the header with the current time (ros::Time::now()).
    '''
    pc = PositionConstraint()
    pc.header.stamp = rospy.Time.now()
    pc.header.frame_id = 'base_footprint'
    pc.link_name = 'r_wrist_roll_link'
    pc.position.x = .5
    pc.position.y = .05
    pc.position.z = .9

    pc.constraint_region_shape.type = 0
    pc.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
    pc.constraint_region_orientation.w = 1.0
    
    goalA.motion_plan_request.goal_constraints.position_constraints.append(pc)


    oc = OrientationConstraint()
    oc.header.stamp = rospy.Time.now()
    oc.header.frame_id = 'base_footprint'
    oc.link_name = 'r_wrist_roll_link'
    oc.orientation.x = 0.
    oc.orientation.y = 0.
    oc.orientation.z = -.3
    
    oc.orientation.w = 1.

    oc.absolute_roll_tolerance = 0.01
    oc.absolute_pitch_tolerance = 0.01
    oc.absolute_yaw_tolerance = 0.04
    oc.weight = 1.

    goalA.motion_plan_request.goal_constraints.orientation_constraints.append(oc)

    move_arm.send_goal(goalA)
    finished_within_time = move_arm.wait_for_result()
    if not finished_within_time:
        move_arm.cancel_goal()
        rospy.logout('Timed out achieving goal A')
    else:
        state = move_arm.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.logout('Action finished with SUCCESS')
        else:
            rospy.logout('Action failed')
