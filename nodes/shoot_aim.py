#!/usr/bin/env python
#how to run this testcode: python shoot_aim.py .79 -.188 .9  2.4 bullet30 0.4
# shoots bullet in yaw direction(2.4 rad) with initial pt[.79,-.188,.9]. bullet30 is the
#name of the bullet, can be anything, but must be unique for each bullet!
#0.4 is the speed
#iMPORTANT: in actualy application, just import this python file and create new
#instances like so: bullet1=shoot_aim(xpos,ypos,zpos,speed,bulletNum)
import roslib
roslib.load_manifest('gazebo')
import rospy,os,sys
direc='/opt/ros/groovy/stacks/simulator_gazebo/gazebo/scripts/spawn_model -file bullet.urdf -urdf'
service_type='/opt/ros/groovy/stacks/simulator_gazebo/gazebo_msgs/srv/ApplyBodyWrench.srv'
from gazebo import gazebo_interface
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
import tf.transformations as tft
import math

class shoot_aim():
    def __init__(self,x,y,z,yaw,speed,name):
        point= Point()
        point.x=x
        point.y=y
        point.z=z
        self.y=yaw
        self.initial=point
        self.speed=speed
        self.name=name
        bulletNum=name

        spawn(x,y,z,yaw,bulletNum)
        shoot_server(bulletNum+'::my_box',speed,point,yaw)


#inserts model at exact place on right wrist
def spawn(xpos,ypos,zpos,yaw,bulletNum):
    os.system('python '+ direc+' -x '+str(xpos)+' -y '+str(ypos)+' -z '+str(zpos)+ ' -Y '+str(yaw)+' -model '+bulletNum)

#calls services to apply wrench, we only care about force in x direction. 
def shoot_server(bullet_name,force,point,yaw):

    apply_wrench_server = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    wrench=Wrench() #intialized wrench object
    #apply force, determined by force and yaw of object
    wrench.force.x=force*math.cos(yaw)
    wrench.force.y=force*math.sin(yaw)
    
    start_time=rospy.Time(10)

    duration=rospy.Duration(1)
    try:
      resp1 = apply_wrench_server(bullet_name,'',point,wrench,start_time,duration)
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)


if __name__ == "__main__":
    xpos=float(sys.argv[1])
    ypos=float(sys.argv[2])
    zpos=float(sys.argv[3])
    yaw=float(sys.argv[4]) #(rad)we only care about yaw since we assume shot is straight initially and roll is irrevelant
    bulletNum=sys.argv[5] #name given to bullet, any string
    speed=float(sys.argv[6]) #speed in m/s
    
    #xpos=.7697
    #ypos=-.188
    #zpos=.9
    #create new instance of bullet easily like this
    bullet1=shoot_aim(xpos,ypos,zpos,yaw,speed,bulletNum)




    
    
