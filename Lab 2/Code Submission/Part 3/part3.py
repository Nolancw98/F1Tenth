#!/usr/bin/env python
# Authors: Nicholas LaRosa, Nolan Sornson
# ECE 350/450 Section 010/011 (Intro to Robotics)
# Lab 2 Step 3.4
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

def callback(data):
  pub1 = rospy.Publisher('closestpoint', Float32, queue_size=10)
  pub2 = rospy.Publisher('farthestpoint', Float32, queue_size=10)
  rate = rospy.Rate(10)
  ranges = data.ranges
  range_max = data.range_max
  range_min = data.range_min
  nearestPoint = range_max
  farthestPoint = range_min
  valid_ranges = []
  for x in ranges:
    if x <= range_max and x >= range_min:
      valid_ranges.append(x)     
  
  for x in valid_ranges:
    if x < nearestPoint:
      nearestPoint = x
    if x > farthestPoint:
      farthestPoint = x
    
  rospy.loginfo(rospy.get_caller_id() + ": New data received")
  rospy.loginfo("nearestPoint: %s", nearestPoint)
  rospy.loginfo("farthestPoint: %s", farthestPoint)
  pub1.publish(nearestPoint)
  pub2.publish(farthestPoint)
  rate.sleep()

def lidar_process():
  rospy.init_node('lidar_process', anonymous=True)
  rospy.Subscriber("scan", LaserScan, callback)
  rospy.spin()

if __name__ == '__main__':
  try:
    lidar_process()
  except rospy.ROSInterruptException:
    pass