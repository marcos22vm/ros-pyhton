#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys
import time
from ros_backend.srv import turtle_command_service 
from turtlesim.msg import Pose
from std_srvs.srv import Empty
velocity_linear = 1
velocity_angular = 1
global end_all_process

def move_up(pub):
    vel = Twist()
    vel.linear.x = velocity_linear
    pub.publish(vel)

def move_down(pub):
    vel = Twist()
    vel.linear.x = -velocity_linear
    pub.publish(vel)

def move_left(pub):   
    vel = Twist()
    vel.linear.y = velocity_linear
    pub.publish(vel) 

def move_rigth(pub):
    vel = Twist()
    vel.linear.y = -velocity_linear
    pub.publish(vel)

def rorate_right(pub):
    vel = Twist()
    vel.angular.z = -velocity_angular
    pub.publish(vel)

def rotate_leftl(pub):
    vel = Twist()
    vel.angular.z = velocity_angular
    pub.publish(vel)    

def move_rigth(pub):
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = -velocity_linear
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    pub.publish(vel)

def draw_circle(pub):
    global end_all_process
    rate = rospy.Rate(11)
    radius = 2
    maxIterations=59
    countIterations=0
    vel = Twist()

    while countIterations<maxIterations :
        if end_all_process :
            return ""
        vel.linear.x = radius
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 1
        pub.publish(vel)
        countIterations+= 1
        rate.sleep()

def draw_star(pub):
    global end_all_process
    size=2
    vel_rotation = 1.247
    sllep_time=1.1    


    vel = Twist()
    
    
    for d in range(5):
        if end_all_process :
            return ""
        vel = Twist()
        vel.linear.x = size
        pub.publish(vel)
        
        time.sleep(sllep_time)
        if end_all_process :
            return ""

        vel = Twist()
        vel.angular.z = (-vel_rotation*2)
        pub.publish(vel)
       
        time.sleep(sllep_time)
        if end_all_process :
            return ""

        vel = Twist()
        vel.linear.x = size
        pub.publish(vel)

        time.sleep(sllep_time)
        if end_all_process :
            return ""
                
        vel = Twist()
        vel.angular.z = (vel_rotation)
        pub.publish(vel)

        time.sleep(sllep_time)

def reset():
    clear_bg = rospy.ServiceProxy('reset', Empty)
    clear_bg()
    time.sleep(1)

def clear():
    clear_bg = rospy.ServiceProxy('clear', Empty)
    clear_bg()
    time.sleep(1)    


def callback_receive_commands(msg):
    global end_all_process
    end_all_process=False
    
    order = msg.command

    pub =rospy.Publisher("/turtlesim_commands", String, queue_size=10)
    pub.publish(order)

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    if order == 'w' :
        move_up(pub)
        return  ""
    if order == 's' :
        move_down(pub)
        return  ""
    if order == 'a' :
        move_left(pub)
        return ""
    if order == 'd' :
        move_rigth(pub)
        return ""
    if order == 'spinr' :
        rorate_right(pub)
        return ""
    if order == 'spinl' :
        rotate_leftl(pub)
        return ""
    if order == 'draw_circle' :
        draw_circle(pub)
        return ""
    if order == 'draw_star' :
        draw_star(pub)
        return ""
    return ""

def callback_receive_stop_commands(msg):
    
    global end_all_process
    end_all_process=True
    time.sleep(0.1)
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    pub.publish(vel) 

    order = msg.command
    rospy.loginfo("Comando stop: "+str(order))
    pub =rospy.Publisher("/turtlesim_commands", String, queue_size=10)
    pub.publish(order)
    
    if order == 'clear' :
        clear()
       
    if order == 'reset' :
        reset()
         
    return ""        

if __name__ == "__main__":
    global stack_position
    global copy_process
    copy_process = False
  
    rospy.init_node('turtlesim_service')

    #PROCESS MOVEMENT COMMAND 
    rospy.Service('turtle_command_service', turtle_command_service, callback_receive_commands)

    #PROCESS STOP COMMAND  
    rospy.Service('turtle_command_stop_service', turtle_command_service, callback_receive_stop_commands)
    
    rospy.spin()


