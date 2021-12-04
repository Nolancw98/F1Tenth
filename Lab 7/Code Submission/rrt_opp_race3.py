#!/usr/bin/env python
"""
ESE 680
RRT assignment
Author: Hongrui Zheng

This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
import tf

import rospy
import random
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
#from tf import transform_listener
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

# TODO: import as you need

#PID controller parameters
kp = 0.10
ki = 0
kd = 0.00005
prev_error = 0.0 
error = 0.0
integral = 0.0

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.child = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(object):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pf_topic = 'opp_id/odom' #'/pf/pose/odom' #'/pf/viz/inferred_pose' #rospy.get_param('pose_topic')
        scan_topic = 'opp_id/scan' #rospy.get_param('scan_topic') 
        drive_topic = 'opp_id/drive'

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        rospy.Subscriber(pf_topic, Odometry, self.pf_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.mkpub = rospy.Publisher("/omap", Marker, queue_size = 5)
        self.pathpub = rospy.Publisher("/path", Path, queue_size = 5)
        self.goalpub = rospy.Publisher("/goal_point", PointStamped, queue_size = 5)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=5)
        self.wp_pub = rospy.Publisher("/wp", Marker, queue_size = 5)

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.heading = 0
        self.x = 0.001
        self.y = 0
        self.radial_ogrid = 0
        self.cartesian_ogrid = 0


        self.valid_ranges = None
        self.angle_array = None
        self.goal_threshold = 0.5 #constant for what is close enough to the goal
        self.exploration_bias = 0.5
        self.safety_buffer = 2
        self.max_expand = 1
        self.n = 10
        self.goal_x = 1
        self.goal_y = 0
        
        #Gap Follow Relevant Things
        self.neg_limit_idx = 0;
        self.pos_limit_idx = 0;
        self.DISPARITY_THRESHOLD = 1
        self.CAR_WIDTH = 0.6
        self.angle_increment = 0
        
        #Pure Pursuit Constants
        self.lookahead_threshold = 1
        self.start_distance = 1000
        self.TIME_INCREMENT = 0.1

        self.tree = [];

        self.count = 0

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        self.angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        range_min = scan_msg.range_min
        valid_ranges = np.clip(ranges, range_min, range_max) #get rid of bad data

        angle_indices = np.arange(valid_ranges.size)
        angle_array = scan_msg.angle_min + angle_indices * scan_msg.angle_increment

        self.valid_ranges = valid_ranges
        self.angle_array = angle_array

        self.neg_limit_idx = (np.abs(self.angle_array + math.radians(90))).argmin()
        self.pos_limit_idx = (np.abs(self.angle_array - math.radians(90))).argmin()

        #self.radial_ogrid = np.array([[valid_ranges], [angle_array]])
        #self.radial_ogrid = np.transpose(self.radial_ogrid)
        if(self.count > 100):
          self.update_goal_point()
          self.count = 0
        self.count += 1

        goal_point = PointStamped()
        goal_point.header.frame_id = "/laser"
        goal_point.header.stamp = rospy.Time.now()
        goal_point.point.x = self.goal_x
        goal_point.point.y = self.goal_y
        goal_point.point.z = 0
        self.goalpub.publish(goal_point)
        return None

    def update_goal_point(self):
        '''
        Copied from Gap follow
        '''

        #instead of using a for loop, make an array of indexes
        index = np.arange(self.valid_ranges.size)

        #create a copy of valid_ranges but shift all elements over by 1
        rolled_ranges = np.roll(self.valid_ranges,1)
        np.put(rolled_ranges,0,self.valid_ranges[0])

        #find the change between two indexes
        diff_ranges = self.valid_ranges - rolled_ranges
        
        # determine the locations of disparities compared to a constant threshold
        pos_disparities = np.where(diff_ranges > self.DISPARITY_THRESHOLD)
        neg_disparities = np.where(diff_ranges < -self.DISPARITY_THRESHOLD)

        # **** HANDLE POSITIVE DISPARITIES ****

        #for each actual disparity index, create an array of lengh REPLACEMENT_THRESHOLD and 
        #fill it with the value at the index from valid_ranges
        #then add to the previous empty array at count created above and increment count
        for index in pos_disparities[0]:
          try:
            pos_temp = self.valid_ranges[index-1] #handle the exceptions caused at the endpoints
          except (IndexError):
            pos_temp = self.valid_ranges[index]

          #determine indices to mask out dynamically using distance to obstacle
          theta = math.atan((self.CAR_WIDTH)/pos_temp)
          replacement_threshold = math.ceil(theta / self.angle_increment)
          indices = np.arange(index, index + int(replacement_threshold))
          np.put(self.valid_ranges, indices, pos_temp, 'clip')
        
        # **** HANDLE NEGATIVE DISPARITIES ****
        
        #for each actual disparity index, create an array of lengh REPLACEMENT_THRESHOLD and 
        #fill it with the value at the index from valid_ranges
        #then add to the previous empty array at count created above and incrememnt count
        for index in neg_disparities[0]:
          try:
            neg_temp = self.valid_ranges[index+1] 
          except(IndexError):
            neg_temp = self.valid_ranges[index]

          #determine indices to mask out dynamically using distance to obstacle
          theta = math.atan((self.CAR_WIDTH)/neg_temp)
          replacement_threshold = math.ceil(theta / self.angle_increment)
          #print("Neg", index, int(replacement_threshold))
          indices = np.arange(index - int(replacement_threshold), index)
          np.put(self.valid_ranges, indices, neg_temp, 'clip')
        
        proc_ranges = self.valid_ranges
        
        theta, r = self.find_best_point(proc_ranges)
        self.goal_x = r * math.cos(theta)
        self.goal_y = r * math.sin(theta)

        return None

    def find_best_point(self, proc_ranges):
        """ Find the largest range value in our dataset
        Return the angle to target
        """        
        front_ranges = proc_ranges[self.neg_limit_idx:self.pos_limit_idx]
        #What is the longest distance in the scans from -90, +90
        index = np.argmax(front_ranges)
        angle = self.angle_array[self.neg_limit_idx] + index * self.angle_increment
        return (angle, front_ranges[index])

    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        self.quaternion = np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w])
        self.euler = tf.transformations.euler_from_quaternion(self.quaternion)
        self.heading = self.euler[2]

        if pose_msg.pose.pose.position.x == 0 or pose_msg.pose.pose.position.y == 0:
          self.x = 0.0000001
          self.y = 0.0
        else:
          self.x = pose_msg.pose.pose.position.x
          self.y = pose_msg.pose.pose.position.y

        #RRT Main Loop
        self.tree = []
        starting_node = Node()
        starting_node.x = self.x
        starting_node.y = self.y
        starting_node.is_root = True
        self.tree.append(starting_node)

        points = []; #visualization #array of Point Objects
        
         #number of points to sample for the tree
        reached_goal = False
        for i in range(self.n):
          xy_rand = self.sample()
          nearest_node = self.nearest(self.tree, xy_rand)
          new_node = self.steer(nearest_node, xy_rand)
          pt = Point()
          pt.x = new_node.x
          pt.y = new_node.y
          pt.z = 0
          points.append(pt)

          #check for collisions
          collision = self.check_collision(nearest_node,new_node)
          if not collision:
            self.tree.append(new_node)
          
          #shortcut if goal is near, maybe needs to change for RRT*
          if(self.is_goal(new_node, self.goal_x, self.goal_y)):
            path = self.find_path(self.tree, new_node)
            reached_goal = True
            break
        
        #if don't reach goal, still need to find a path
        if not reached_goal:    
          optimal_node = self.nearest(self.tree, (self.goal_x, self.goal_y))
          path = self.find_path(self.tree, optimal_node)


        #visualize all sampled points
        mk = Marker()
        mk.header.frame_id = "/laser";
        mk.header.stamp = rospy.Time.now()
        mk.type = 8#CUBE_LIST
        mk.action = 0
        mk.pose.position.x = self.x * 0.05#* self.map_msg.info.resolution;
        mk.pose.position.y = self.y * 0.05#* self.map_msg.info.resolution;
        mk.pose.position.z = 0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        #mk.points = map(self.npArray_to_Point, global_xyz)
        mk.points = points
        #print(map(self.npArray_to_Point, global_xyz))
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;
        mk.color.a = 1.0;
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        self.mkpub.publish(mk)
        
        self.pathpub.publish(path[1])

        goal_index = 0

        position = np.array([self.x, self.y])
        waypoints = np.flip(path[2],0)
        
        #new_waypoints = waypoints

        
        rotate = np.array([[math.cos(-self.heading), math.sin(-self.heading)],[-math.sin(-self.heading), math.cos(-self.heading)]])
        new_waypoints = np.matmul(waypoints,rotate)
        new_waypoints = new_waypoints-position
        
        # compute distances away from all waypoints
        distances = new_waypoints**2; #[n,2]
        distances = distances[:,0]+distances[:,1] #[n,1]
        distances = np.sqrt(distances)
  
        #check if we have to update the goal point.  We are only checking one point ahead because we only switch when a waypoint enters our lookahead threshold.  Instead of having the car look for waypoints around it, we are making the waypoints large and if the car enters the area of a waypoint, we consider that good enough        
        if(distances[goal_index] < self.lookahead_threshold):
          goal_index += 1 
          if(goal_index == distances.size):
            goal_index = 0
        
        # this helps mitigate skipping points and to some extent the kidnapped robot problem
        if(new_waypoints[goal_index,0] < 0):
          goal_index += 1	  
          if(goal_index == distances.size):
            goal_index = 0
          
        self.start_distance = distances[goal_index]

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
        I = integral + ki * error * self.TIME_INCREMENT # integral component
        D = kd * (error - prev_error) / self.TIME_INCREMENT # derivative component
        angle = P + I + D #angle output is sum of components
        
        #Limit to max steering angle
        steeringLimit = 0.4189 #radians
        if(angle > steeringLimit):
          angle = steeringLimit
        elif(angle < -steeringLimit):
          angle = -steeringLimit


        global prev_targetspeed
        
        # Linear interpolation of speed points
        if goal_index == 0:
          targetspeed = 2 #((distances[goal_index]/self.start_distance)*(speeds[goal_index]-speeds[distances.size-1]))+speeds[distances.size-1]
        else:
          targetspeed = 2 #((distances[goal_index]/self.start_distance)*(speeds[goal_index]-speeds[goal_index-1]))+speeds[goal_index-1]
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
        #print("Goal Point (Car Frame): ", waypoints[goal_index, :])
        #print("Goal Point (Global Frame): ", new_waypoints[goal_index, :])
        #print("L: ", distances[goal_index])
        #print("Steering Angle: ", angle)
	      #print("Target Speed: ", targetspeed)
        
        return path[0]

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        #determine angles ahead of car ***I MADE THESE OBJECT VARIABLES***
        #neg_limit_idx = (np.abs(self.angle_array + math.radians(90))).argmin()
        #pos_limit_idx = (np.abs(self.angle_array - math.radians(90))).argmin()
        #front_ranges = self.valid_ranges[neg_limit_idx:pos_limit_idx]
        
        #Choose random LIDAR index [-90, 90]
        rand_angle_idx = random.randint(self.neg_limit_idx,self.pos_limit_idx)

        #Choose random range [0, max dist along actual LIDAR ray]
        rand_range = random.uniform(self.exploration_bias,self.valid_ranges[rand_angle_idx] - self.safety_buffer)

        #convert r, theta to cartesian x, y
        x = rand_range * math.cos(self.angle_array[rand_angle_idx])
        y = rand_range * math.sin(self.angle_array[rand_angle_idx])

        #x = None
        #y = None
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        dlist = [(node.x - sampled_point[0])**2 + (node.y - sampled_point[1])**2 for node in tree]
        minind = dlist.index(min(dlist))

        nearest_node = tree[minind]
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        #randNode = Node()
        #randNode.x = sampled_point[0]
        #randNode.y = sampled_point[1]

        dx = sampled_point[0] - nearest_node.x
        dy = sampled_point[1] - nearest_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)

        if(d > self.max_expand):
          d = self.max_expand

        newNode = Node()
        newNode.x = nearest_node.x + d * math.cos(theta)
        newNode.y = nearest_node.y + d * math.sin(theta)
        newNode.parent = nearest_node
        return newNode

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        # Find slope of line segment
        m = (new_node.x-nearest_node.x)/(new_node.y-nearest_node.y)
        # Use one point to solve for b
        b = new_node.x-(m*new_node.y)

        # Get first and last LIDAR rays desired
        desired_theta_a = math.atan(nearest_node.y/nearest_node.x)
        desired_theta_b = math.atan(new_node.y/new_node.x)

        # Find actual theta_a and theta_b indices
        theta_a_idx = np.argmin(self.angle_array - desired_theta_a)
        theta_b_idx = np.argmin(self.angle_array - desired_theta_b)
        x_intercept = np.array([])
        y_intercept = np.array([])
        #r_intercept = np.array([])
        #theta_intercept = np.array([])

        for i in range(theta_a_idx, theta_b_idx, 10):
          # Get ray/line segment intercepts
          theta_slope = math.tan(self.angle_array[i]+(math.pi/2))
          np.append(x_intercept,[(b/(theta_slope-m))])
          np.append(y_intercept,[(theta_slope*x_intercept)])
          
        r_intercept = np.sqrt((x_intercept**2)+(y_intercept**2))
        theta_intercept = np.tan(x_intercept/y_intercept)
        theta_intercept_idx = []
        for i in theta_intercept:
          theta_intercept_idx.append(np.argmin(self.angle_array-theta_intercept[i]))
        
        for i in theta_intercept_idx:
          if r_intercept[i] > self.valid_ranges[i]:
            return True

        return False

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        dx = goal_x - latest_added_node.x
        dy = goal_y - latest_added_node.y
        if(math.hypot(dx, dy) < self.goal_threshold): #if within goal threshold
          return True
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        pathViz = []
        reverse_waypoints = np.array([])
        x = []
        y = []
        
        # Start with last node (that is close to goal)
        node = latest_added_node

        #if this node has a parent
        while node.parent is not None:
            
            #for visualizing
            pose = PoseStamped()
            pose.header.frame_id = "/laser";
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pathViz.append(pose)

            x.append(node.x)
            y.append(node.y)
            #np.append(reverse_waypoints, ([node.x, node.y]))
            
            #traverse up graph
            path.append(node) #add it to the list
            node = node.parent #traverse upwards to the parent
        
        path.append(node) #add the initial node to the list (maybe we don't want to do this given our pure pursuit algorithim)
        
        #for visualizing
        pose = PoseStamped()
        pose.header.frame_id = "/laser";
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = node.x * 0.05
        pose.pose.position.y = node.y * 0.05
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pathViz.append(pose)

        x.append(node.x)
        y.append(node.y)
        #np.append(reverse_waypoints, [[node.x, node.y]])

        path_msg = Path()
        path_msg.header.frame_id = "/laser"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = pathViz

        xnp = np.asarray(x)
        ynp = np.asarray(y)

        reverse_waypoints = np.vstack((xnp,ynp))
        reverse_waypoints = np.transpose(reverse_waypoints)

        #print(reverse_waypoints)
        
        return (path, path_msg, reverse_waypoints)

    #def calc_distance_and_angle(from_node, to_node):
    #    dx = to_node.x - from_node.x
    #    dy = to_node.y - from_node.y
    #    d = math.hypot(dx, dy)
    #    theta = math.atan2(dy, dx)
    #    return d, theta



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()
