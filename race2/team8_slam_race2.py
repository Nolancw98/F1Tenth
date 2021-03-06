#!/usr/bin/env python
# Authors: Nicholas LaRosa, Nolan Sornson
# Team 8
# ECE 350/450 Section 010/011 (Intro to Robotics)
# Race 2 Pure Pursuit + Particle Filter Localization (Car Version)
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
kp = 0.30  # we found kp did not need to be large when we are driving fast
ki = 0
kd = 0
prev_error = 0.0 
error = 0.0
integral = 0.0

goal_index = 0 #index, increment that for new goal
lookahead_threshold = 1.0 # lookahead threshold is large for smooth driving

# TODO: import ROS msg types and libraries

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.

        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' #'/nav'
        # This topic is from Particle Filter, it allows us to localize as we are driving, referencing the map we created in cartographer
        odom_topic = 'pf/viz/inferred_pose' #'/odom'

        rospy.Subscriber(odom_topic, PoseStamped, self.pose_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=0)

        global waypoints
        global speeds
        home = expanduser('~')
        filepath = home+'/rcws/targets/race2_waypoints.csv'
        array_from_file = np.genfromtxt(filepath, delimiter=',')
        
        # Waypoints are (x, y) pairs with respect to the global map
        waypoints = array_from_file[:,0:2]
	
        # speeds are float values assigned to each waypoint.  They set the target speed up until the waypoint for with they are paired with. 
        speeds = array_from_file[:,2]

    def pose_callback(self, data):
        
        global waypoints
        global speeds
        global goal_index
        global lookahead_threshold
        global start_distance
        heading=0
        position=0
	
        # grab quaternions from particle filter's inferred pose, convert to euler angles and extract heading.
        quaternion = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,data.pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        heading = euler[2];

        # store current position in a numpy array as a pair
        x = data.pose.position.x
        y = data.pose.position.y
        position = np.array([x,y])
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        # Translate all waypoints to car frame
        new_waypoints = waypoints-position
        
        # Rotate all waypoints to car frame
        rotate = np.array([[math.cos(-heading), math.sin(-heading)],[-math.sin(-heading), math.cos(-heading)]])
        new_waypoints = np.matmul(new_waypoints,rotate)

        # compute distances away from all waypoints
        distances = new_waypoints**2; #[n,2]
        distances = distances[:,0]+distances[:,1] #[n,1]
        distances = np.sqrt(distances)
        
        #check if we have to update the goal point.  We are only checking one point ahead because we only switch when a waypoint enters our lookahead threshold.  Instead of having the car look for waypoints around it, we are making the waypoints large and if the car enters the area of a waypoint, we consider that good enough
        if(distances[goal_index] < lookahead_threshold):
              goal_index += 1	  
          if(goal_index == distances.size):
            goal_index = 0
        # this helps mitigate skipping points and to some extent the kidnapped robot problem
        if(new_waypoints[goal_index,0] < 0):
          goal_index += 1	  
          if(goal_index == distances.size):
            goal_index = 0
          
        start_distance = distances[goal_index]

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

        #print("-----------------------------------")
        #print("Current Position: ", position)
        #print("Goal Point (Global Frame): ", waypoints[goal_index, :])
        #print("Goal Point (Car Frame): ", new_waypoints[goal_index, :])
        #print("L: ", distances[goal_index])
        #print("Steering Angle: ", angle)
        #print("Target Speed: ", targetspeed)

def main():
    rospy.init_node('pure_pursuit_node')
    #import waypoints from CSV
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
