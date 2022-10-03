#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			2339
# Author List:		Dhruva Shaw
# Filename:			task_0.py
# Functions:
# 					main, update_pose, equationroots, make_semicircle, give_pos_y
# Nodes:		    turtlebot_controller


####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pi
from typing import Union, Tuple
##############################################################

class TurtleBot:

    def __init__(self) -> None:
        """
        Initializes the important variables required in the whole class
        """        
        
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)
 
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)

        #Storing the position of the turtle
        self.pose = Pose()
        
        #The frequency of the turle
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        
        #Final position of y required to make a semicircle
        self.final_pos_circle_y = 0
        
        #Initial startin point of the turtle
        self.start_point_y = self.pose.y
        self.start_point_x = self.pose.x
    
    def update_pose(self, data: Pose) -> None:
        """
        Callback function which is called when a new message of type Pose is
        received by the subscriber.
        """
        #saving the individual data in the Pose class
        self.pose = data
        
        #Storing the rounded data up to two decimal place as it is easier to calculate
        self.pose.x = round(self.pose.x,2)
        self.pose.y = round(self.pose.y,2)
    
    def equationroots(self, a: int, b: int, c: int) -> Tuple[Union[int, float]]:
        """Takes a,b and c coefficients of the root and calculates the roots using shreedharacharya's formula

        Returns:
            Tuple[Union[int, float]]: The Root of the equation
        """
        # calculating discriminant using formula
        dis = b * b - 4 * a * c 
        #returning both the values using shreedharacharya's formula
        return (-b + sqrt(dis))/(2*a) , (-b - sqrt(dis))/(2*a)
    
    def give_pos_y(self, y_pos: int, radius: int) -> Union[int, float]:
        """Gives the required position of the coordinate that the turtle needs to move to form a semicircle from initial y axis and radius

        Args:
            y_pos (int): The initial y position of the tutrle at the start
            radius (int): THe radius of the semicircle

        Returns:
            Union[int, float]: Required position of the coordinate that the turtle needs to move to form a semicircle
        """        
        y1, y2 = self.equationroots(1, -2*y_pos, -(((radius*2)**2)-(y_pos**2)))
        return y1 if y1 > 0 else y2
    
    def make_semicircle(self, radius: int) -> None:
        """Make a D shape in teh turtlesim

        Args:
            radius (int): The radius of 'D' or semicircle
        """
        
        #semicircle        
        while not rospy.is_shutdown():
            #Initialization of the velocity variables
            self.vel.linear.x: int = radius #Since x coordinate needs to be moved to make a D shape
            self.vel.linear.y: int = 0
            self.vel.linear.z: int = 0
            self.vel.angular.x: int = 0
            self.vel.angular.y: int = 0
            self.vel.angular.z: int = radius #also to make a curve the angulare z needs to move
            
            #Logging
            rospy.loginfo("Radius = %f",radius)
            rospy.loginfo("x = %f",self.pose.x)
            rospy.loginfo("y = %f",self.pose.y)
            rospy.loginfo("Final position for the semicircle without the bottom horizontal line = %f",self.final_pos_circle_y)
            
            #Getting some impt values just at the startup
            if self.pose.x != 0 and self.pose.y != 0 and self.final_pos_circle_y == 0:
                self.final_pos_circle_y = self.give_pos_y(self.pose.y, radius) #final position of y to make semicircle
                self.start_point_y = self.pose.y #inital position of y
                self.start_point_x = self.pose.x #inital position of x
                
            #While loop breaking logic to make the semicircle    
            if self.pose.y >= self.final_pos_circle_y and self.final_pos_circle_y != 0:
                self.vel.linear.x: int = 0
                self.vel.angular.z = 0
                self.velocity_publisher.publish(self.vel) #Publishes the velocity to the turtle
                break
            self.velocity_publisher.publish(self.vel)
            self.rate.sleep() #This function properply controles the frequency of the turtle
        
        #Rotation
        self.vel.linear.x = 0 #Seeting this to 0 as it is not required
        self.vel.angular.z=abs(90*2*pi/360) #angles to radians
        self.velocity_publisher.publish(self.vel)
        self.rate.sleep() 
        
        #Linear motion to complete the D
        while not rospy.is_shutdown():
            self.vel.linear.x = 0
            self.vel.linear.y = sqrt((self.pose.y - self.start_point_y)**2 + (self.pose.x - self.start_point_x)**2) #Using the distance formula to calculate the exact linear distance that needs to be cover
            self.vel.linear.z = 0
            self.vel.angular.x = 0
            self.vel.angular.y = 0
            self.vel.angular.z = 0
            
            #Breaking logic of the linear motion
            if self.pose.y <= self.start_point_y:
                self.vel.linear.y = 0
                self.velocity_publisher.publish(self.vel)
                break
            
            rospy.loginfo("x = %f",self.pose.x)
            rospy.loginfo("y = %f",self.pose.y)
            
            self.velocity_publisher.publish(self.vel)
            
            self.rate.sleep()
        
        rospy.spin()


def main() -> None:
    """The main function which runs the whole program
    """
    #Init of the class    
    turtlebot = TurtleBot()
    
    #Making the D shape of radius 1
    turtlebot.make_semicircle(1)



######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
