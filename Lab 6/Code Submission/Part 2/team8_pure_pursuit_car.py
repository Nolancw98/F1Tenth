#!/usr/bin/env python
# Authors: Nicholas LaRosa, Nolan Sornson
# Team 8
# ECE 350/450 Section 010/011 (Intro to Robotics)
# Lab 6 Pure Pursuit (Car Version)

import rospy
import tf
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from tf.transformations import euler_from_quaternion
from numpy import linalg as LA
from os.path import expanduser

waypoints = np.array([])
speeds = np.array([])
TIME_INCREMENT = 0.1
start_distance=1000

#PID controller parameters
kp = 0.5
ki = 0
kd = 0
prev_error = 0.0 
error = 0.0
integral = 0.0

goal_index = 0 #index, increment that for new goal
lookahead_threshold = 0.5

# TODO: import ROS msg types and libraries

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        #rospy.Subscriber("scan", LaserScan, self.scan_callback)

        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        odom_topic = '/vesc/odom'

        rospy.Subscriber(odom_topic, Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=0)

        global waypoints
        global speeds
        home = expanduser('~')
        filepath = home+'/rcws/targets/target_waypoints.csv'
        array_from_file = np.genfromtxt(filepath, delimiter=',')
        waypoints = array_from_file[:,0:2]
        speeds = array_from_file[:,2]

    
    #ASSUMPTIONS: 
    # We always have a point within lookahead distance
    # 
    def pose_callback(self, data):
        
        global waypoints
        global speeds
        global goal_index
        global lookahead_threshold
        global start_distance
        heading=0
        position=0

        quaternion = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        heading = euler[2];

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        position = np.array([x,y])
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        # Translate all waypoints
        new_waypoints = waypoints-position
        
        # Rotate all waypoints
        rotate = np.array([[math.cos(-heading), math.sin(-heading)],[-math.sin(-heading), math.cos(-heading)]])
        new_waypoints = np.matmul(new_waypoints,rotate)

        # Determine waypoint that is closest to look-ahead distance
        # TODO: transform goal point to vehicle frame of reference
        
        
        
        #check if we have to update the goal point
        #sqrt(x-x^2 + y-y^2)
        #if(magn(position and new_waypoints[:,goal_index + 1])) < lookahead_threshold):
        #  goal_index += 1
        distances = new_waypoints**2; #[n,2]
        distances = distances[:,0]+distances[:,1] #[n,1]
        distances = np.sqrt(distances)
        if(distances[goal_index] < lookahead_threshold):
          goal_index += 1
          if(goal_index == distances.size):
            goal_index = 0
	    start_distance = distances[goal_index]
        # x = 0 y = 0
        # x = 1 y = 1
        # |x = 0 y = 0|
        # |x = 1 y = 1|
        # TODO: calculate curvature/steering angle
        r = distances[goal_index]**2 / (2.0 * new_waypoints[goal_index,1])
        curvature = 1.0 / r

        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians

        #Steering angle is proportional to curvature
        global integral
        global prev_error
        global kp
        global ki
        global kd
        
        error = curvature
        P = kp * error # proportional component
        I = integral + ki * error * TIME_INCREMENT # integral component
        D = kd * (error - prev_error) / TIME_INCREMENT # derivative component
        angle = P + I + D #angle output is sum of components
        
        #Limit to max steering angle
        steeringLimit = 0.4189 #radians
        if(angle > steeringLimit):
          angle = steeringLimit
        elif(angle < -steeringLimit):
          angle = -steeringLimit


        #Speed control (TO-DO)
        global prev_targetspeed
        
        # Linear interpolation of speed points
        if goal_index is 0:
          targetspeed = ((distances[goal_index]/start_distance)*(speeds[goal_index]-speeds[distances.size-1]))+speeds[distances.size-1]
        else:
          targetspeed = ((distances[goal_index]/start_distance)*(speeds[goal_index]-speeds[goal_index-1]))+speeds[goal_index-1]
              #targetspeed = 2
        
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = targetspeed
        self.drive_pub.publish(drive_msg)

        print("-----------------------------------")
        print("Current Position: ", position)
        print("Goal Point (Global Frame): ", waypoints[goal_index, :])
        print("Goal Point (Car Frame): ", new_waypoints[goal_index, :])
        print("L: ", distances[goal_index])
        print("Steering Angle: ", angle)
        print("Heading: ", heading)
        print("Target Speed: ", targetspeed)

def main():
    rospy.init_node('pure_pursuit_node')
    #import waypoints from CSV
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
