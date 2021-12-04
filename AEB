#!/usr/bin/env python
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

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        range_min = scan_msg.range_min
        valid_ranges = []
        for x in ranges:
          if x <= range_max and x >= range_min:
            valid_ranges.append(x)     
        
        ttc = []
        for i in range(len(valid_ranges)):
          angle = scan_msg.angle_min + i * scan_msg.angle_increment
          projection = abs(self.speed) * math.cos(angle)
          denominator = max(-projection,0.0)
          if(self.speed > 0):
            ttc[i] = valid_ranges[i] / denominator

        # TODO: publish brake message and publish controller bool
        for time in ttc:
          if(time < 10):
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.header.frame_id = 'AEB'
            ack_msg.drive.speed = 0.0
            self.brake_pub.publish(ack_msg)
            bool_msg = Bool()
            bool_msg.data = True;
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
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
