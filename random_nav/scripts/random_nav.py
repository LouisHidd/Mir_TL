#!/usr/bin/env python

"""This node repeatedly selects random map positions until it finds
  one that can be navigated to.
  It then navigates to the random goals using the ROS navigation stack.
"""
import numpy as np
import rospy
import map_utils
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees, pi
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,  Point, Twist, Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu
from random import randint
import sys
import time


class RandomNavNode(object):
    def __init__(self):
        
        rospy.init_node('random_nav')
        self.map_msg = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
    def run(self):

        while not rospy.is_shutdown():
            


            if self.map_msg is None:
                rospy.loginfo("Waiting for map...")
                rospy.sleep(.1)
            else:
                self.map = map_utils.Map(self.map_msg)
                self.demo_map()

    def demo_map(self):
        """ Illustrate how to interact with a loaded map object. """
        
        x_pos = randint(-8, 8)
        y_pos = randint(-8, 8)
        

        if self.map.get_cell(x_pos, y_pos) == 0:
            message = "clear"
        elif self.map.get_cell(x_pos, y_pos) == -1:
            message = "unknown"
            message = "Position ({}, {}) is ".format(x_pos, y_pos) + message
            rospy.loginfo(message)


            ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

            #wait for the action server to come up
            while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")

            goal = MoveBaseGoal()

            #set up the frame parameters
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            # moving towards the goal*/
            goal.target_pose.pose.position =  Point(x_pos,y_pos,0)
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

            start = rospy.get_time()  #get the current time

            rospy.loginfo("Sending goal location ...")
            ac.send_goal(goal)
                
            ac.wait_for_result(rospy.Duration(120))

            ##################-- goal arrived --#################
            if(ac.get_state() ==  GoalStatus.SUCCEEDED):
                rospy.loginfo("You have reached the destination")

                ###########-- calculate the time of the path --##########
                end = rospy.get_time()  #get the current time
                duration = end - start
                rospy.loginfo("Duration: %.3f s"%duration)
            
            else:
                rospy.loginfo("The robot failed to reach the destination")

                move_cmd = Twist()
                rate = 20
                #Set the equivalent ROS rate variable
                r = rospy.Rate(rate)

                # Set the forward linear speed to 0.2 meters per second
                linear_speed= 0.5
                # Set the travel distance to 1.0 meters
                goal_distance= 0.5
                # How long should it take us to get there?
                linear_duration=goal_distance/linear_speed

                #Move forward for a time to go 1 meter
                move_cmd.linear.x = (-1) * linear_speed
                ticks = int(linear_duration * rate)
                for t in range(ticks):
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                    rospy.loginfo("Ich bin hier2")
        elif np.isnan(self.map.get_cell(x_pos, y_pos)):
            message = "unknown (out of bounds)"
        else:
            message = "occupied"
        message = "Position ({}, {}) is ".format(x_pos, y_pos) + message
        rospy.loginfo(message)

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg
        
if __name__ == "__main__":
    randomNavNode = RandomNavNode()
    randomNavNode.run()