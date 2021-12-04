#!/usr/bin/env python
# Authors: Nicholas LaRosa, Nolan Sornson
# Team 8
# ECE 350/450 Section 010/011 (Intro to Robotics)
# Lab 4 Gap Follow Algorithm (Car Version)
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

DISPARITY_THRESHOLD=0.6
CAR_WIDTH = 0.7
TIME_INCREMENT = 0.1

#PID controller parameters
kp = 1.0
ki = 0.00001
kd = 0.1
prev_error = 0.0 
error = 0.0
integral = 0.0
prev_targetspeed = 1

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=0)
    
    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        #process range data using numpy
        ranges = data.ranges
        range_max = data.range_max
        range_min = data.range_min
        valid_ranges = np.clip(ranges, range_min, range_max) #get rid of bad data

        #instead of using a for loop, make an array of indexes
        index = np.arange(valid_ranges.size)

        #create a copy of valid_ranges but shift all elements over by 1
        rolled_ranges = np.roll(valid_ranges,1)
        np.put(rolled_ranges,0,valid_ranges[0])

        #find the change between two indexes
        diff_ranges = valid_ranges - rolled_ranges
        
        # determine the locations of disparities compared to a constant threshold
        pos_disparities = np.where(diff_ranges > DISPARITY_THRESHOLD)
        neg_disparities = np.where(diff_ranges < -DISPARITY_THRESHOLD)

        # HANDLE POSITIVE DISPARITIES 

        #for each actual disparity index, create an array of lengh REPLACEMENT_THRESHOLD and 
        #fill it with the value at the index from valid_ranges
        #then add to the previous empty array at count created above and increment count
        for index in pos_disparities[0]:
          try:
            pos_temp = valid_ranges[index-1] #handle the exceptions caused at the endpoints
          except (IndexError):
            pos_temp = valid_ranges[index]

          #determine indices to mask out dynamically using distance to obstacle
          theta = math.atan((CAR_WIDTH)/pos_temp)
          replacement_threshold = math.ceil(theta / data.angle_increment)
          indices = np.arange(index, index + int(replacement_threshold))
          np.put(valid_ranges, indices, pos_temp,'clip')
        

        # HANDLE NEGATIVE DISPARITIES
        
        #for each actual disparity index, create an array of lengh REPLACEMENT_THRESHOLD and 
        #fill it with the value at the index from valid_ranges
        #then add to the previous empty array at count created above and incrememnt count
        for index in neg_disparities[0]:
          try:
            neg_temp = valid_ranges[index+1] #[1, 1, 1, 1, 1]
          except(IndexError):
            neg_temp = valid_ranges[index]

          #determine indices to mask out dynamically using distance to obstacle
          theta = math.atan((CAR_WIDTH)/neg_temp)
          replacement_threshold = math.ceil(theta / data.angle_increment)
          #print("Neg", index, int(replacement_threshold))
          indices = np.arange(index - int(replacement_threshold), index)
          np.put(valid_ranges, indices, neg_temp,'clip')
        
        #print("After Processing\t", valid_ranges, "\n")
        proc_ranges = valid_ranges
        return proc_ranges
    
    def find_best_point(self, proc_ranges, data):
        """ Find the largest range value in our dataset
        Return the angle to target
        """
        angle_indices = np.arange(proc_ranges.size)
        angle_array = data.angle_min + angle_indices * data.angle_increment
        
        neg_limit_idx = (np.abs(angle_array + math.radians(90))).argmin()
        pos_limit_idx = (np.abs(angle_array - math.radians(90))).argmin()
        front_ranges = proc_ranges[neg_limit_idx:pos_limit_idx]
        #What is the longest distance in the scans from -90, +90
        index = np.argmax(front_ranges)
        angle = angle_array[neg_limit_idx] + index * data.angle_increment
        return {"angle": angle, "range": front_ranges[index]}

    def check_corners(self, proc_ranges, data):
        """ Keep the car away from the corners (we ultimately did not use this method) """
        '''
        angle_indices = np.arange(proc_ranges.size)
        angle_array = data.angle_min + angle_indices * data.angle_increment

        left_front_idx = (np.abs(angle_array + math.radians(90))).argmin()
        left_rear_idx = (np.abs(angle_array + math.radians(135))).argmin()
        
        right_front_idx = (np.abs(angle_array - math.radians(90))).argmin()
        right_rear_idx = (np.abs(angle_array - math.radians(135))).argmin()

        left_ranges = proc_ranges[left_rear_idx:left_front_idx]
        right_ranges = proc_ranges[right_front_idx:right_rear_idx]

        threshold = 0.3

        left_warn = np.any(left_ranges < threshold)
        right_warn = np.any(right_ranges < threshold)

        return {"left_warn": left_warn, "right_warn": right_warn}
	      '''
        return {"left_warn": 0, "right_warn": 0}


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        #Cleaned range data
        #Extended Disparities 
        proc_ranges = self.preprocess_lidar(data)

        #Find the deepest point in valid_ranges with ext disparities 
        best_point = self.find_best_point(proc_ranges, data)

        #Check corners
        warnings = self.check_corners(proc_ranges, data)
        
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_targetspeed
        error = best_point["angle"]
        P = kp * error # proportional component
        I = integral + ki * error * TIME_INCREMENT # integral component
        D = kd * (error - prev_error) / TIME_INCREMENT # derivative component
        angle = P + I + D #angle output is sum of components
        
	drive_msg.drive.steering_angle = angle
        prev_error = error
        integral = I
        
	topspeed = 4 #maximum speed on straightaways
        targetspeed = topspeed
	speedDecisionDist = proc_ranges[proc_ranges.size/2] #decide speed based on distance straight ahead
        if(speedDecisionDist > 1.75): # if distance is more than threshold, set to top speed
          targetspeed = topspeed
        else: # otherwise derate speed linearly based on distance, and limit acceleration back to top speed
          targetspeed = min(prev_targetspeed+0.1, topspeed - ((5.5/topspeed) * (1.8 - speedDecisionDist)))
          prev_targetspeed = min(prev_targetspeed+0.1, targetspeed)
        drive_msg.drive.speed = targetspeed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rgf = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
