#! /usr/bin/env python
#Tres tortugas

import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from std_srvs.srv import Empty


from turtlesim.msg import *
from turtlesim.srv import *
from std_srvs.srv import *



x = 0
y = 0
z = 0
theta = 0

x2 = 0
y2 = 0

x3 = 0
y3 = 0

x4 = 0
y4 = 0

#Estos poseCallbacks permiten guardar en variables X y Y las pocisiones de las tortugas. En especial PoseCallBack tiene pose_message.theta que permite determinar la orientacion de nuestra tortuga a mover e identificar a las demas.

def poseCallback(pose_message):
    global x
    global y
    global theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
  

def poseCallback2(pose_message):
    global x2
    global y2

    x2 = pose_message.x
    y2 = pose_message.y
    

def poseCallback3(pose_message):
    global x3
    global y3

    x3 = pose_message.x
    y3 = pose_message.y


def poseCallback4(pose_message):
    global x4
    global y4
	
    x4 = pose_message.x
    y4 = pose_message.y

#la funcion go to goal permite a partir de coordenadas dadas en el programa trasladarse a una pocision particular en X y Y. En este caso la tortuga que mueve es la tortuga 3.

def go_to_goal (xgoal, ygoal):

    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle3/cmd_vel'
    #cmd_vel_topic permite defnir que tortuga es la que tiene movimiento en este nodo de python.

    while(True):
        kv = 2.5
        distance = abs(math.sqrt(((xgoal - x) ** 2) + ((ygoal - y) ** 2)))
        linear_speed = kv * distance
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal - y, xgoal - x)
    	dtheta = math.fmod((desired_angle_goal-theta),(2*math.pi))
	if(dtheta<0.0):
		if( abs(dtheta)>((2*math.pi)+dtheta)):
			dtheta = (2*math.pi)+dtheta
	else:
		if(dtheta> abs(dtheta-(2*math.pi))):
			dtheta = dtheta-(2*math.pi)  
	
	angular_speed = ka * (dtheta)
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        

        if (distance < 0.01):
            break

#La funcion orientate permite como su nombre lo dice dirigir a la tortuga hacia una direccion especifica. Es importante siempre orientar la tortuga antes de un go to goal o hara curvas.

def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle3/cmd_vel'

    while(True):
        ka = 5
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
	dtheta=math.fmod((desired_angle_goal-theta),(2*math.pi))
	if(dtheta<0.0):
		if( abs(dtheta)>((2*math.pi)+dtheta)):
			dtheta=(2*math.pi)+dtheta
	else:
		if(dtheta> abs(dtheta-(2*math.pi))):
			dtheta=dtheta-(2*math.pi)
	angular_speed = ka * (dtheta)

        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break


#Main del codigo
if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous = True)
        #Aqui se determina que la torttuga que se va a mover es la 3 (la nuestra)
        cmd_vel_topic = '/turtle3/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
	
        #En esta parte del codigo se obtiene informacion sobre la pocision de las tortugas
	position_topic3 = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic3, Pose, poseCallback3)
        time.sleep(2)
    
        position_topic1 = "/turtle3/pose"
        pose_subscriber = rospy.Subscriber(position_topic1, Pose, poseCallback)
        time.sleep(2)

        position_topic2 = "/turtle2/pose"
        pose_subscriber = rospy.Subscriber(position_topic2, Pose, poseCallback2)
        time.sleep(2)

	position_topic4 = "/turtle4/pose"
        pose_subscriber = rospy.Subscriber(position_topic4, Pose, poseCallback4)
        time.sleep(2)
	
	rate=rospy.Rate(30)
 
    
	c=0;
	yg=y
	xg=x
	f=0

	while(y<10 and c<7):
		
		dx1=abs(x-x3)
		dy1=abs(y-y3)
		dx2=abs(x-x2)
		dy2=abs(y-y2)
		dx4=abs(x-x4)
		dy4=abs(y-y4)
			
		if((dy1>1.5 or dx1>1.5) and (dx2>1.5 or dy2>1.5) and (dx4>1.5 or dy4>1.5) or f>4):
			orientate(1.00,10.00)
			go_to_goal(xg,yg)
			yg=yg+0.8
			xg=xg-0.8
			f=0
			c=c+1
		else:
			time.sleep(1)
			f=f+1
			
		if(xg<1):
			xg=xg+0.8
	
	orientate(1,10)
	go_to_goal(1,10)
	time.sleep(1)
	
	yg=y
	xg=x
	f=0

	while(y>1):

		dx1=abs(x-x3)
		dy1=abs(y-y3)
		dx2=abs(x-x2)
		dy2=abs(y-y2)
		dx4=abs(x-x4)
		dy4=abs(y-y4)
			
		if((dy1>1.5 or dx1>1.5) and (dx2>1.5 or dy2>1.5) and (dx4>1.5 or dy4>1.5) or f>4):
			orientate(4.0,1.0)
			go_to_goal(xg,yg)
			yg=yg-0.8
			xg=xg+0.8
			f=0
		else:
			time.sleep(1)
			f=f+1
			
		if(xg>4):
			xg=xg-0.8
	
	 
	orientate(4,1)
	go_to_goal(4,1)
	time.sleep(1)

	orientate(10,10)
	time.sleep(1)
	
	yg=y
	xg=x
	f=0
	while(y<10):

		dx1=abs(x-x3)
		dy1=abs(y-y3)
		dx2=abs(x-x2)
		dy2=abs(y-y2)
		dx4=abs(x-x4)
		dy4=abs(y-y4)
			
		if((dy1>1.5 or dx1>1.5) and (dx2>1.5 or dy2>1.5) and (dx4>1.5 or dy4>1.5) or f>4):
			orientate(10.0,10.0)
			go_to_goal(xg,yg)
			yg=yg+0.8
			xg=xg+0.8
			f=0
		else:
			time.sleep(1)
			f=f+1
			
		if(xg>10):
			xg=xg-0.8

	orientate(10,10)
	go_to_goal(10,10)
	
    except rospy.ROSInterruptException:
	pass
