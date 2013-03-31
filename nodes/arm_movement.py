#!/usr/bin/env python
""" Arm Movement

Controls robot arm and gripper
"""

import roslib; roslib.load_manifest('ros_sentry')
import rospy
import tf

import actionlib

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
    

SH_CONTROLLERS = {
    'r': '/r_arm_controller/joints',
    'l': '/l_arm_controller/joints',
}

def create_goal(joints_p, arm_side = 'r'):
    """ Create a goal for a given set of joints angles

    joints_p is a vector of angles
    """
    goal = JointTrajectoryGoal()
    # Populates trajectory with joint names.
    for joint_name in rospy.get_param(SH_CONTROLLERS[arm_side]):
        goal.trajectory.joint_names.append(joint_name)
    
    goal.trajectory.points = []

    # Positions
    for ind in xrange(len(joints_p)):
        point = JointTrajectoryPoint()

        point.positions = joints_p[ind]
        point.velocities = [0]*len(point.positions)
        goal.trajectory.points.append(point)
        #To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = rospy.Duration(1.5)*ind

    goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(.5)        
    return goal


def getHandPos(arm_side = 'r'):
    tf_listener.waitForTransform('/' + arm_side + '_wrist_roll_link', '/base_footprint', rospy.Time(0),rospy.Time(3))
    (trans,rot) = tf_listener.lookupTransform('/' + arm_side + '_wrist_roll_link', '/base_footprint', rospy.Time(0))
    return trans, rot

def moveGrip(p = 0.08, arm_side = 'r'):
    open = Pr2GripperCommandGoal()
    open.command.position = p
    open.command.max_effort = -1.0
    r_g_controller.send_goal(open)

def moveArmTo(p, arm_side = 'r'):
    """ Set arm to position 

    either use IK or inhouse IK
    """
    x0, r0 = getHandPos(arm_side)
    pass

## Sharpshooter
def bulletPhysics():
    """ Stocastic initial speed """
    f = (1,0,0)
    return f

def practiceShootGazebo(target, arm):
    """ Simulate Shooting

    set the gazebo Simulation
    run init
    return whatever the target is hit
    """
    ## Set Target

    ## Set arm/hand position

    ## Create bullet + physics
    # Rotate bullet to direction of hand

    # Start Simulation

    # Verify if hit

    # De-spawn target and bullet
    # Return hit (or minimal distance)
    pass

def createTrainingSet(n = 100):
    trainSet = []
    for i in xrange(n):
        ## Decide target position
        ## Decide hand position
        hit = practiceShootGazebo()
        ## Save trial
    return trainSet

def trainSharpshooter():
    """ Train at sharpshooting

    TODO decide learning method
    TODO save final state on disk

    """
    ## Gather Training set
    ## Update policy/model/decider/filter
    ## return/init model
    pass

def aimSharpshooter(target):
    ## Return joints / handposition / posegoal to hit the target
    pass

### Encoded Actions
## Encode wave
wave = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        [0.1, 0.3,  0.0, -1.8,  0.0,  0.0,   0.3],
        [0.1, 0.2, 0.0, -0.8, 0.0, 0.0, 0.0],
        [0.1, 0.3,  0.0, -1.8,  0.0,  0.0,   0.3],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]


def robotWave():
    l_controller.send_goal(create_goal(wave, 'l'))
    #l_client.send_goal(create_goal([-0.3,0.2,-0.1,-1.2,1.5,-0.3,0.5]))

def robotShoot():
    """ Press the trigger

    """
    moveGrip(0.08)
    r_g_controller.wait_for_result()
    moveGrip(0)

def shootTarget(target):
    ## Verify training works
    ## predict trajectory of the target using track_predictor
    ## decide where to aim
    goal = aimSharpshooter(target)
    ## schedule arm movement and trigger
    ## shoot

def test():
    ## Get List of Arms Joints
    print rospy.get_param('/r_gripper_controller/joint')

    print getHandPos(arm_side = 'r')
    rospy.loginfo(rospy.get_name() + ": I'm waving to you")
    robotWave()
    moveGrip(0.08)
    r_g_controller.wait_for_result()
    moveGrip(0)
    print getHandPos(arm_side = 'r')

if __name__ == '__main__':
    rospy.init_node('arm_movement', anonymous=True)
    ## Init servers
    global l_controller, r_controller, r_g_controller, tf_listener

    l_controller = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    l_controller.wait_for_server()
    r_controller = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    r_controller.wait_for_server()
    r_g_controller = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
    r_g_controller.wait_for_server()

    tf_listener = tf.TransformListener()

    
    