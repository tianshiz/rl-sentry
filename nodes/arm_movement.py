#!/usr/bin/env python
""" Arm Movement

Controls robot arm and gripper
"""
import os
from numpy import random
from numpy import *

from matplotlib import pylab

import roslib; roslib.load_manifest('ros_sentry')
import rospy
import PyKDL

import tf
import actionlib

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, Wrench, Point
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

from gazebo_msgs.srv import DeleteModel, ApplyBodyWrench, GetLinkState, GetModelState

import track_predictor


SPAWN_SCRIPT   ='python /opt/ros/groovy/stacks/simulator_gazebo/gazebo/scripts/spawn_model -file nodes/bullet.urdf -urdf'
WRENCH_SERVICE ='/opt/ros/groovy/stacks/simulator_gazebo/gazebo_msgs/srv/ApplyBodyWrench.srv'
    
    
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
    now = rospy.Time(0)  #.now()
    tf_listener.waitForTransform('/' + arm_side + '_wrist_roll_link', '/base_footprint', now, rospy.Duration(10))
    (trans,rot) = tf_listener.lookupTransform('/' + arm_side + '_wrist_roll_link', '/base_footprint', now)
    return trans, PyKDL.Rotation.Quaternion(*rot)

def moveGrip(p = 0.08, arm_side = 'r'):
    open = Pr2GripperCommandGoal()
    open.command.position = p
    open.command.max_effort = -1.0
    r_g_controller.send_goal(open)

def moveHandTo(p, arm_side = 'r'):
    """ Set arm to position 

    either use IK or inhouse IK
    """
    x0, r0 = getHandPos(arm_side)
    pass

## Sharpshooter
def bulletPhysics():
    """ Stocastic initial speed

    return the initial speed, as a vector, oriented along the x axis"""
    
    ## Simulation Parameters
    v0 = 17.8 ## ms
    angles = 0.017 ## rad
    
    theta = random.random() * 3.1416
    phi = random.normal(scale = angles)
    f = PyKDL.Vector(cos(phi)*v0, sin(phi)*cos(theta)*v0, sin(phi)*sin(theta)*v0)
    return f

def practiceShootGazebo(target):
    """ Simulate Shooting

    target is a Point / tuple
    
    shoots the bullet from the current configuration
    
    
    return the closes point of the trajectory and its time
    """

    ## Gazebo Services
    deleteModel = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    

    ## Set arm/hand position

    ## Create bullet + physics
    model_name = 'bullet1'
    # Rotate bullet to direction of hand
    x0, r0 = getHandPos('r')
    r0 = hand2gun * r0
    
    os.system(SPAWN_SCRIPT + ' -x %f -y %f -z %f'%x0 + ' -Y %f -R %f - P %f'%r0.GetRPY() +' -model '+model_name)
    
    wrench = Wrench() # Message to apply forces in Gazebo
    dt = 0.03         # impulse duration
    f = gun2hand * r0.Inverse() * bulletPhysics() # Generate initial speed
    f = projectile_m * f/dt                       # "convert" speed in a force
    
    wrench.force.x = f[0]
    wrench.force.y = f[1]
    wrench.force.z = f[2]
    
    try:
        resp1 = apply_wrench_server(model_name+'::my_box', '',
                                    Point(x0[0], x0[1], x0[2]), wrench,
                                    rospy.Time.now(), rospy.Duration(dt))
        
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
    
    ## Start Simulation
    start_t = rospy.Time(0) #.now()
    # measure trajectory
    trajectory = []
    try:
        for t in xrange(0,50):
            ## wait 0.1 secs
            p = get_model_server(model_name, '').pose.position
            trajectory.append((rospy.Time(0) - start_t, (p.x-target[0], p.y - target[1], p.z - target[2] )) ) 
    except Exception, e:
        print "Service did not process request: %s"%str(e)

    try:
        deleteModel(model_name) # De-spawn target and bullet
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)

    ## Find minimal distance
    dist_min = 9999
    t_min = 0

    for t,x in trajectory:
        dist = sqrt(x[0]**2 + x[1]**2 + x[2]**2 )
        if dist < dist_min:
            t_min = t
            dist_min = dist

    savetxt('trajectory.txt', [t[1] for t in trajectory ])
    t = array([t[1] for t in trajectory ]).T
    pylab.plot(t[0], t[1])
    pylab.show()
    return t_min, dist_min  # Return hit (or minimal distance)

def practiceShootTheory(target):
    """ Predict trajectory of bullet using physics filter 
    
    target is a position
    """
    
    x, r0 = getHandPos(arm_side)
    v = r0.Inverse() * gun2hand * bulletPhysics()   # Generate initial speed
    a = (0, 0, -9.8)
    dt = 0.1
    
    dist_min = 9999
    t_min = 0
    
    for i in xrange(300):
        x, v = x + v * dt, v + a * dt
        if sqrt(sum(x-target)**2) < dist_min:
            t_min = i*dt
            
    return t_min, dist_min
    
def createTrainingSet(n = 100):
    trainSet = []
    for i in xrange(n):
        ## Decide target position
        target = random.random(3)
        ## Decide hand position
        x0 = random.random(3)
        moveHandTo(x)
        hit = practiceShootGazebo(target)
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
    
def test_shoot():
    print practiceShootGazebo((20,20,20))

## Robot knowledge
hand2gun = PyKDL.Rotation.EulerZYX(0.8,0,0)
gun2hand = hand2gun.Inverse()
ikinematics = None

projectile_m = 0.0013  # From URDF file 

if __name__ == '__main__':
    rospy.init_node('arm_movement', anonymous=True)
    ## Init servers
    global l_controller, r_controller, r_g_controller, tf_listener, apply_wrench_server, get_model_server

    l_controller = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    l_controller.wait_for_server()
    r_controller = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    r_controller.wait_for_server()
    r_g_controller = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
    r_g_controller.wait_for_server()

    tf_listener = tf.TransformListener()

    apply_wrench_server = rospy.ServiceProxy('gazebo/apply_body_wrench', ApplyBodyWrench)
    get_model_server = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    #apply_wrench_server.wait_for_server()

    test_shoot()