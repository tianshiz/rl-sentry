#!/usr/bin/env python
""" Arm Movement

Controls robot arm and gripper
"""
import os

from numpy import random
from numpy import *
from scipy import optimize

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

## IK module
from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal

## Forward Kinematics
## roslaunch pr2_3dnav right_arm_navigation.launch
#from kinematics_msgs.msg import GetKinematicSolverInfo, GetPositionFK

## Grip action
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

## Gazebo
from gazebo_msgs.srv import DeleteModel, ApplyBodyWrench, GetLinkState, GetModelState

## RViz
from visualization_msgs.msg import Marker, MarkerArray

import track_predictor
from ik_goal import createIKGoal


SPAWN_SCRIPT   ='python /opt/ros/groovy/stacks/simulator_gazebo/gazebo/scripts/spawn_model -file nodes/bullet.urdf -urdf'
WRENCH_SERVICE ='/opt/ros/groovy/stacks/simulator_gazebo/gazebo_msgs/srv/ApplyBodyWrench.srv'
    
    
SH_CONTROLLERS = {
    'r': '/r_arm_controller/joints',
    'l': '/l_arm_controller/joints',
}

def createFKGoal(joints_p, arm_side = 'r'):
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
        goal.trajectory.points[ind].time_from_start = rospy.Duration(0.5)*ind

    goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(.5)        
    return goal

def getHandPos(arm_side = 'r'):
    """ Return current position of the gripper 

    in respect of the robot
    """
    now = rospy.Time(0)  #.now()
    tf_listener.waitForTransform('/' + arm_side + '_wrist_roll_link', '/base_footprint', now, rospy.Duration(10))
    (trans,rot) = tf_listener.lookupTransform('/' + arm_side + '_wrist_roll_link', '/base_footprint', now)
    return trans, PyKDL.Rotation.Quaternion(*rot)

def moveGrip(p = 0.08, arm_side = 'r'):
    """ Open/close the gripper 
    """
    open = Pr2GripperCommandGoal()
    open.command.position = p
    open.command.max_effort = -1.0
    r_g_controller.send_goal(open)

def gunpose(theta = 0, phi = 0):
    return [[ 0.3,.4,0,-2,pi + phi,-pi/2 + theta,0] ]

def moveHandTo(position,orientation, 
               arm_side = 'r',
               method = 'house'):
    """ Set arm to position 

    either use IK or inhouse IK
    
    for inhouse only 2 angles are moved.
    """
    if method == 'ik':
        x0, r0 = getHandPos(arm_side)
        ik_goal = create_IK_goal(position, orientation, arm_side)
        if arm_side == 'r':
            r_arm_ik.send_goal(ik_goal)
        if arm_side == 'l':
            l_arm_ik.send_goal(ik_goal)
    ## Rotate only
    if method == 'house':
        ## TODO verify that gun was initialized
        gun_orientation = hand2gun * orientation 
        __, phi, theta = gun_orientation.GetRPY()
        r_controller.send_goal(createFKGoal(gunpose(theta,phi), 'r'))
        r_controller.wait_for_result()


def initGunAngle():
    """ Put robot in pose
    rotates the arm holding the gun and collects
    training data"""
    #gunpose = [[ 0.3,.4,0,-2,pi,-pi/2,0] ]
    

    angle_in = []
    angle_out = []
    ## Create a set of joint angles/hand angle
    for phi in linspace(-pi/2,pi/2,0):
        for theta in linspace(0,pi/3,0):
            r_controller.send_goal(createFKGoal(gunpose(theta,phi), 'r'))
            r_controller.wait_for_result()
            __, rot = getHandPos()
            angle_in.append((phi,theta))
            angle_out.append(rot.GetRPY())

    ## Set to angle 0 and get use it as reference
    r_controller.send_goal(createFKGoal(gunpose(0,0), 'r'))
    r_controller.wait_for_result()
    rospy.sleep(1)
    pos, rot = getHandPos()

    global gun2hand, hand2gun, robot2gun
    gun2hand = rot
    hand2gun = gun2hand.Inverse()
    robot2gun = pos

    return angle_in, angle_out

# def getFK():
#     ros::service::waitForService("pr2_right_arm_kinematics/get_fk_solver_info");
#     ros::service::waitForService("pr2_right_arm_kinematics/get_fk");

#     ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>
#         ("pr2_right_arm_kinematics/get_fk_solver_info");
#     ros::ServiceClient fk_client = rh.serviceClient
#         <kinematics_msgs::GetPositionFK>("pr2_right_arm_kinematics/get_fk");

#     ## define the service messages
#     GetKinematicSolverInfo::Request request
#     GetKinematicSolverInfo::Response response
#     if query_client.call(request,response):
#         for(unsigned int i=0; 
#             i< response.kinematic_solver_info.joint_names.size(); i++)
#         ROS_DEBUG("Joint: %d %s", i,
#                     response.kinematic_solver_info.joint_names[i].c_str())

#     else
#         ROS_ERROR("Could not call query service");
#         ros::shutdown();
#         exit(1);
#     }
#     ## define the service messages
#     GetPositionFK.Request  fk_request;
#     GetPositionFK.Response fk_response;
#      fk_request.header.frame_id = "torso_lift_link";
#       fk_request.fk_link_names.resize(2);
#       fk_request.fk_link_names[0] = "r_wrist_roll_link";
#       fk_request.fk_link_names[1] = "r_elbow_flex_link";
#       fk_request.robot_state.joint_state.position.resize
#         (response.kinematic_solver_info.joint_names.size());
#       fk_request.robot_state.joint_state.name = 
#         response.kinematic_solver_info.joint_names;
#       for(unsigned int i=0; 
#           i< response.kinematic_solver_info.joint_names.size(); i++)
#       {
#         fk_request.robot_state.joint_state.position[i] = 0.5;
#       }
#       if(fk_client.call(fk_request, fk_response))
#       {
#         if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
#         {
#           for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
#           {
#             ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
#             ROS_INFO_STREAM("Position: " << 
#               fk_response.pose_stamped[i].pose.position.x << "," <<  
#               fk_response.pose_stamped[i].pose.position.y << "," << 
#               fk_response.pose_stamped[i].pose.position.z);
#             ROS_INFO("Orientation: %f %f %f %f",
#               fk_response.pose_stamped[i].pose.orientation.x,
#               fk_response.pose_stamped[i].pose.orientation.y,
#               fk_response.pose_stamped[i].pose.orientation.z,
#               fk_response.pose_stamped[i].pose.orientation.w);
#           } 
#         }
#         else
        
## Sharpshooter
def getGunPos():
    ## TODO!!!!!!!!!!!
    p, r = getHandPos()
    return p, hand2gun*r


def bulletPhysics(n=1, r0= PyKDL.Rotation.Identity()):
    """ Stocastic initial speed

    can generate more than one 

    return the initial speed, as a vector, oriented along the x axis"""
    
    theta = random.random(n) * 3.1416
    phi = random.normal(size = n, scale = bullet_d_angle)
    v = c_[cos(phi)*bullet_v0,
           sin(phi)*cos(theta)*bullet_v0,
           sin(phi)*sin(theta)*bullet_v0]
    
    if n == 1:
        v = r0 * PyKDL.Vector(v[0],v[1],v[2])
    if n > 1:
        r0 = array( [[r0[0,0], r0[0,1], r0[0,2]],
                     [r0[1,0], r0[1,1], r0[1,2]],
                     [r0[2,0], r0[2,1], r0[2,2]]]).T
        #v = r_[v[0],v[1],v[2]]
        for i in xrange(n):
            v[i] = dot(r0,v[i])      
    return v

def inverseAngle(dx, v0, a = -9.8):
    """ Find shooting angle 

    dx is the distance vector (3d)
    """
    phi = arctan2(dx[0],dx[1])
    dz = sqrt(dx[0]**2 + dx[1]**2)
    dy = dx[2]
    
    f = lambda t: cos(t)**2 *dy - sin(t)*cos(t)*dz - a*0.5*dz**2/v0**2
    
    theta = optimize.brentq(f,-pi/6.,pi/3)  ## Avoid trajectory with gun going higher than 45
    dt = dz/(cos(theta)*v0)
    
    return phi,theta,dt
 
def bulletTrajectory(x0, v0, dt, a = -9.8*r_[0,0,1]):
    return x0 + v0*dt + 1./2 *a *dt**2

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
    l_controller.send_goal(createFKGoal(wave, 'l'))
    #l_client.send_goal(create_goal([-0.3,0.2,-0.1,-1.2,1.5,-0.3,0.5]))

def robotShoot():
    """ Press the trigger

    """
    moveGrip(0.08)
    r_g_controller.wait_for_result()
    moveGrip(0)


    
    
def shootTarget(target_trajectory, debug_plot = False):
    """ Target contains the trajectory.. """
    ## Get next few points in target trajectory
    ## TODO
    target_trajectory = [(2, (3,0,1)),
                         (3, (4,0,1)),
                         (4, (5,0,1))]  ## Placeholder

    ## for each of trajectory points calculate angle
    coarse_aims = [0]*len(target_trajectory)
    gun_pos, gun_rot = getGunPos() 
    for i, (time, target_pos) in enumerate(target_trajectory):
        dx = array(target_pos) - array(gun_pos) # Relative position in absolute ref frame
        coarse_aims[i] = inverseAngle(dx, bullet_v0)
    
    ## sample proj traj for each angle at the given time
    samples_n = 10
    samples_p = []
    for i, (phi, theta, dt) in enumerate(coarse_aims):
        r0 = PyKDL.Rotation.RPY(0,  theta, -pi/2+phi)
        v0 = bulletPhysics(samples_n, r0)
        samples_p.append( bulletTrajectory(gun_pos, v0, dt))
        
        #ax.scatter(samples_p[0],samples_p[1],samples_p[2], 'b')
    
    ## Publish target trajectory
    ## Possibly on mastermind

    markerArray = MarkerArray()
    for i, (time, target_pos) in enumerate(target_trajectory):
            marker = createMarker( pos = target_pos, color = (1., 0.1, 0.),
                                     ID = i, size = 0.2 )
            markerArray.markers.append(marker)

    # Publish the MarkerArray
    trajectory_topic.publish(markerArray)

    ## Publish bullets on topic
    markerArray = MarkerArray()
    count = 0
    for p in samples_p:
        for b in p:
            marker = createMarker( pos = b, ID = count, size = 0.05)
            marker.id = count
            markerArray.markers.append(marker)

            # Renumber the marker IDs
            count += 1

    # Publish the MarkerArray
    bullets_topic.publish(markerArray)

    #rospy.sleep(0.01)
    # if debug_plot:
    #     import matplotlib.pylab as pl
    #     from mpl_toolkits.mplot3d import Axes3D

    #     fig = pl.figure()
    #     ax = fig.add_subplot(111, projection='3d')

    #     for t,p in target_trajectory:
    #         ax.scatter(p[0],p[1],p[2], color= 'r')

    #     for p in samples_p:
    #         ax.scatter(p.T[0],p.T[1],p.T[2], 'b')

    #     pl.show()  

    #sample some man traj at that time
    #count how many close points we get
    #forward kinematics

    
    
    
    #goal = aimSharpshooter(target)
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

def test_various():
    print bulletPhysics(10)

def test_IK():
    p0,r0 = getHandPos(arm_side = 'r')
    p1 = (.4,.4,.4)
    r1 = PyKDL.Rotation.RPY(0,0.4,0.4)

    moveHandTo(position= p1, orientation = rot.GetQuaternion(), arm_side = 'r' )
    moveHandTo(position= p0, orientation = r0, arm_side = 'r' )


## Robot knowledge
hand2gun = PyKDL.Rotation.RPY(0,0.3,pi/2)
gun2hand = hand2gun.Inverse()

bullet_m = 0.0013  # From URDF file 
## Simulation Parameters
bullet_v0 = 17.8 ## ms
bullet_d_angle = 0.017 ## rad


def initROScom():
    """ Init all server connections """
    
    global l_controller, r_controller, r_g_controller
    global tf_listener
    global apply_wrench_server, get_model_server
    global r_arm_ik, l_arm_ik
    global trajectory_topic, bullets_topic

    l_controller = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    l_controller.wait_for_server()
    r_controller = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
    r_controller.wait_for_server()
    r_g_controller = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
    r_g_controller.wait_for_server()

    tf_listener = tf.TransformListener()

    apply_wrench_server = rospy.ServiceProxy('gazebo/apply_body_wrench', ApplyBodyWrench)
    get_model_server = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    trajectory_topic = rospy.Publisher("targets", MarkerArray)
    bullets_topic = rospy.Publisher("bullets", MarkerArray)

    # r_arm_ik = actionlib.SimpleActionClient('right_arm', MoveArmAction)
    # l_arm_ik = actionlib.SimpleActionClient('left_arm', MoveArmAction)
    # r_arm_ik.wait_for_server()
    # l_arm_ik.wait_for_server()
    
def testShootAngle():
    a1, a2 = initGunAngle()
    pylab.plot([a[0] for a in a1], unwrap([a[2] for a in a2]),'.')
    pylab.plot([a[1] for a in a1], unwrap([a[1] for a in a2]),'.')

    pylab.show()

## Visualization ##########
def createMarker(pos, color = (.1, .1, 1), ID = 0, alpha = 1., size = 0.5):
    marker = Marker()
    marker.header.frame_id = "/base_footprint"
    marker.id = ID
    marker.type = marker.SPHERE
    marker.action = marker.ADD

    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = alpha

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    return marker

if __name__ == '__main__':
    ## Create a node
    rospy.init_node('arm_movement', anonymous=True)
    ## Init servers
    initROScom()
    #robotWave()
    #initGunAngle()
    #r0 = getHandPos('r')[1]
    #moveHandTo((0,0,0), r0 * PyKDL.Rotation.RPY(0,-0.4,0))
    #print getHandPos('r')[1].GetRPY()
    shootTarget([], True)
     
    #r_controller.send_goal(createFKGoal([[ 0.3,.4,0,-2,pi,-pi/2,0] ], 'r'))
    #l_controller.send_goal(createFKGoal([[ 0.3,.4,0,-2,pi,-pi/2,0] ], 'l'))
    #apply_wrench_server.wait_for_server()
    #test_various()