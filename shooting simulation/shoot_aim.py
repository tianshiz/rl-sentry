#!/usr/bin/env python
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
import spawn_model


#inserts model at exact place on right wrist
def spawn(xpos,ypos,zpos,bulletNum):
	#os.system('python '+ direc+' -x '+xpos+' -y '+ypos+' -z '+zpos+' -model '+bulletNum)
	#return 0
    sm=spawn_model.SpawnModel()
    sm.inital_xyz=[xpos,ypos,zpos]
    sm.file_name='bullet.urdf'
    sm.model_name=bulletNum
    sm.urdf_format=True
    sm.callSpawnService()
    return
         
def shoot_server(bullet_name,xforce,point):

    apply_wrench_server = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    print 'server started'
    wrench=Wrench()
    wrench.force.x=xforce
    
    start_time=rospy.Time(10)

    duration=rospy.Duration(10)
    try:
      resp1 = apply_wrench_server(bullet_name,'',point,wrench,start_time,duration)
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)


if __name__ == "__main__":
	#xpos=sys.argv[0]
	#ypos=sys.argv[1]
	#zpos=sys.argv[2]
	#bulletNum=sys.argv[3]
    
	xpos='.7697'
	ypos='-.188'
	zpos='.9'
	point= Point()
	point.x=.7697
	point.y=-.188
	point.z=.9

	bulletNum='bullet12'
	spawn(xpos,ypos,zpos,bulletNum)
	shoot_server('bullet12::my_box',0.4,point)
	#apply_wrench_server()

	
	
