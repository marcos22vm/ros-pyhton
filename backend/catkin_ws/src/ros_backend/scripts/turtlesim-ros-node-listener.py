#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose

import sys
import time
import math

global actual_pose_x
global actual_pose_y
global actual_pose_theta
global end_all_process

def callback_move_to_point(pose) :
    global actual_pose_x
    global actual_pose_y
    global actual_pose_theta
    global end_all_process

    end_all_process=False

    goal_x=pose.x
    goal_y=pose.y

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
  
    prop_cons_vel_linear=0.5
    prop_cons_vel_angular=12
    
    max_angle_tolerance=2.9
    min_angle_tolerance=0.01
   
    while(True):

        if end_all_process : return
        vel = Twist()
        
        #DISTANCE = CALCULATE HYPOTENUSE sqrt(dX**2 + dY**2) 
        distance_linear = abs(math.sqrt(((goal_x-actual_pose_x) ** 2 + (goal_y-actual_pose_y) ** 2)))

        #SPEED IS A PORTION OF DISTANCE 
        linear_speed=distance_linear *prop_cons_vel_linear

        #THE OBJECTIVE ANGLE IS THE ARCOTANGENT ATAN2(dY, dX) 
        angul_to_objetive = math.atan2(goal_y-actual_pose_y, goal_x-actual_pose_x)

        #THE ANGULAR VELOCITY IS MULTIPLIED BY AN ACCELERATION CONSTANT 
        angular_speed = (angul_to_objetive - actual_pose_theta) * prop_cons_vel_angular 
        
        
        #IF THE ANGLE IS LESS THAN THE MIN TOLERANCE, THE ANGULAR SPEED IS NOT MODIFIED
        if abs(angul_to_objetive - actual_pose_theta)<min_angle_tolerance :
                vel.linear.x= linear_speed
                vel.angular.z=0.0
        else : 
            #FOR ANGLES GREATER THAN THE MAX TOLERANCE, A FIRST APPROACH IS MADE 
            if(abs(angul_to_objetive-actual_pose_theta)>max_angle_tolerance ) : 
                vel.linear.x= 2.0
                vel.angular.z=5.0
            else :
                vel.linear.x= linear_speed
                vel.angular.z=angular_speed
                
        pub.publish(vel)
           
        if distance_linear<0.01 :
            break

        time.sleep(0.01)
        
        
def callback_refresh_pose(pose) : 
    global actual_pose_x
    global actual_pose_y
    global actual_pose_theta

    actual_pose_x=pose.x
    actual_pose_y=pose.y
    actual_pose_theta=pose.theta


def callback_stop_operation(msg):
    global end_all_process
    rospy.loginfo("Comando recibito en go to: "+str(msg))
    operation = msg.data

    if operation=="pause" or operation=="clear" or operation=="reset": 
        end_all_process=True
        rospy.loginfo("go to PAUSANDO: "+str(end_all_process))


if __name__ == '__main__':
    global end_all_process
    end_all_process=False

    
    rospy.init_node('turtlesim_listener')
    
    #MOVE ROBOT TO POINT 
    rospy.Subscriber("turtlesim_move_to_point", Pose2D , callback_move_to_point)

    #UPDATE ROBOT POSITION 
    rospy.Subscriber("/turtle1/pose", Pose, callback_refresh_pose)
    
    #STOPS ANY OPERATION BEING PERFORMED 
    rospy.Subscriber("/turtlesim_commands", String, callback_stop_operation)

    rospy.spin()
