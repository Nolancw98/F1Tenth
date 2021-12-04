#!/usr/bin/env python
# Authors: Nicholas LaRosa, Nolan Sornson
# ECE 350/450 Section 010/011 (Intro to Robotics)
# Lab 2 Step 4.3

import rospy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float32

# TODO: import ROS msg types and libraries

class Safety(object):

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        #print("Current Speed", odom_msg.twist.twist.linear.x)

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC


        #set limit of TTC value
        threshold = 0.5

        #process range data using numpy
        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        range_min = scan_msg.range_min
        valid_ranges = np.clip(ranges, range_min, range_max) #get rid of bad data
        
        #create ttc array to all values above threshold
        ttc = np.full((1,valid_ranges.size),threshold + 1)

        #instead of using a for loop, make an array of indexes
        index = np.arange(valid_ranges.size)

        #determine angle from the process we determined before 
        angle = scan_msg.angle_min + index * scan_msg.angle_increment
        
        #only do if moving to handle div by zero
        if(abs(self.speed) > 0):
          #vector projection (is this right? can we use dot product)
          projection = abs(self.speed) * np.cos(angle)

          #if denominator is ever negative, just make it zero
          denominator = abs(projection)

          #calculate ttc
          ttc = valid_ranges / denominator

        #create blank messages
        ack_msg = AckermannDriveStamped()
        bool_msg = Bool()

        #if the smallest value in ttc is less than threshold
        if(np.nanmin(ttc) < threshold):
          print("AEB Triggered", np.nanmin(ttc))
          ##rospy.loginfo("AEB Triggered: %s", np.nanmin(ttc))

          #populate the AckermannDriveStamped message
          ack_msg.header.stamp = rospy.Time.now()
          ack_msg.header.frame_id = 'AEB'
          ack_msg.drive.speed = 0.0

          #publish the AckermannDriveStamped message
          self.brake_pub.publish(ack_msg)

          #populate the brake bool message
          bool_msg.data = True;  
        else:
          #when its not AEBing, should publish false
          bool_msg.data = False;
        
        #publish the brake bool message here so we are always sending up to date info
        self.brake_bool_pub.publish(bool_msg)
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.brake_pub = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub = rospy.Publisher('brake_bool', Bool, queue_size=10)

def main():
    rospy.init_node('safety_node',anonymous=True)
    sn = Safety()
    while not rospy.is_shutdown():
      rospy.spin()
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass