#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,  Point, Twist, Pose
#from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, TwistWithCovariance
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
import time
import numpy as np

class map_navigation():

  def choose(self):

    choice='q'

    rospy.loginfo("|-----------------------------|")
    rospy.loginfo("|PRESSE A KEY:")
    rospy.loginfo("|'1': Goal_1: ( 0.0 , 8.0)")
    rospy.loginfo("|'2': Goal_2: (-8.0 , 1.5) ")
    rospy.loginfo("|'3': Goal_3: ( 0.0 , 0.0) ")
    #rospy.loginfo("|'4': Goal_4: (0.0 , 0.0) ")
    rospy.loginfo("|'0': Quit ")
    rospy.loginfo("|-----------------------------|")
    rospy.loginfo("|WHERE TO GO?")
    choice = input()
    return choice

  def __init__(self):

    # declare the coordinates of interest
    ###################################################################
    ############ get the position of the goal x and y #################
    ############ run command: rostopic echo /amcl_pose ################
    ##### change the position of the robot and check the x and y ######
    ###################################################################
    self.xGoal_1 = 10.0#3.0
    self.yGoal_1 = 10.0 #1.5

    self.xGoal_2 = -8.0
    self.yGoal_2 = 1.5

    self.xGoal_3 = 0.0
    self.yGoal_3 = 0.0

    #self.xGoal_4 = 0.0
    #self.yGoal_4 = 0.0
    g = 0
    h = 0
    self.goalReached = False

    self.k = 0 
    
    self.xy = []
    self.sumax = []
    self.sumay = []

    # initiliaze
    rospy.init_node('map_navigation', anonymous=False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
    sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback) # get the messages of the robot pose in frame
    #sub_vec = rospy.Subscriber('/odom_comb', TwistWithCovariance, self.vec_callback) 
    sub_imu = rospy.Subscriber('/imu_data', Imu, self.imu_callback)

    rate = rospy.Rate(0.5)

    choice = self.choose()

    if (choice == 1):
      self.k = 1
      while ((not rospy.is_shutdown and not ac.wait_for_server(rospy.Duration.from_sec(5.0))) or h != self.xGoal_1):
        h = h +2
        #self.moveToGoal(self.xGoal_1 - (self.xGoal_1 - h), self.yGoal_1)

        while ((not rospy.is_shutdown and not ac.wait_for_server(rospy.Duration.from_sec(5.0))) or g != self.yGoal_1):
          g = g +2
          self.moveToGoal(self.xGoal_1 - (self.xGoal_1 - h), self.yGoal_1 -(self.yGoal_1 - g))
        g = 0
        
    elif (choice == 2):
      self.k = 1
      self.goalReached = self.moveToGoal(self.xGoal_2, self.yGoal_2)
    elif (choice == 3):
      self.k = 1
      self.goalReached = self.moveToGoal(self.xGoal_3, self.yGoal_3)

    elif (choice == 4):
      self.goalReached = self.moveToGoal(self.xGoal_4, self.yGoal_4)
    elif (choice == 0):
      exit(0)

    if (choice!='q'):
        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          #rospy.spin()
        else:
           rospy.loginfo("Hard Luck!")

    while choice != 'q':
      choice = self.choose()
      if (choice == 1):
        self.k = 1
        self.goalReached = self.moveToGoal(self.xGoal_1, self.yGoal_1)
      elif (choice == 2):
        self.k = 1
        self.goalReached = self.moveToGoal(self.xGoal_2, self.yGoal_2)
      elif (choice == 3):
        self.k = 1
        self.goalReached = self.moveToGoal(self.xGoal_3, self.yGoal_3)
      #elif (choice == 4):
        #self.goalReached = self.moveToGoal(self.xGoal_4, self.yGoal_4)
      elif (choice == 0):
        exit(0)

      if (choice!='q'):
        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          #rospy.spin()
        else:
          rospy.loginfo("Hard Luck!")

    while not rospy.is_shutdown():
      move = Twist()
      pub.publish(move)
      rate.sleep() 

  def shutdown(self):
    # stop MIR
    rospy.loginfo("Quit program")
    #rate.sleep()

################ -- get accleration of robot from imu -- ###############
  def imu_callback(self, imsg):

    ax = imsg.linear_acceleration.x
    ay = imsg.linear_acceleration.y

    if self.k == 1:
      #print ("------------------------------------------------")
      #print (ax)
      #print (ay)

      if ax > 0:
        self.sumax.append(ax)     
      if ay > 0:
        self.sumay.append(ay)

############ -- get the current pose of the robot -- #################
  def odom_callback(self, msg):
    #print ("------------------------------------------------")
    #print ("pose vx = " + str(msg.twist.twist.linear.x))
    #print ("pose vy = " + str(msg.twist.twist..y))
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    self.xy.append(x)
    self.xy.append(y)
    #print(self.xy)

##### Task: How to save the velocity data to the file? So that they be analysed in the matlab. #######

##############-- move to the destination --##################
  def moveToGoal(self,xGoal,yGoal):

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      #wait for the action server to come up
      while (not rospy.is_shutdown and not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map" #/map
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/
      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      start = rospy.get_time()  #get the current time

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)
      
      ac.wait_for_result(rospy.Duration(60))

      ##################-- goal arrived --#################
      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")

        ###########-- calculate the time of the path --##########
        end = rospy.get_time()  #get the current time
        duration = end - start
        rospy.loginfo("Duration: %.3f s"%duration)
        
        ###########-- calculate the length of the path --##########
        l = len(self.xy)
        #print(l)
        #print(self.xy)
        i = 0
        path_length = 0.0
        while i<= l-6:  #i_max = len(l)-4
          path_length += np.sqrt((self.xy[i+2]-self.xy[i])**2 + (self.xy[i+3]-self.xy[i+1])**2)
          i += 2
        #print(i)

        ############### -- write the data to the file -- ###############
        with open("ac_x.txt", 'w') as f:
          for j in self.sumax:
            f.write(str(j) + '\n')

        with open("ac_y.txt", 'w') as f:
          for k in self.sumay:
            f.write(str(k) + '\n')
        ################################################################

        sax = sum(self.sumax)
        say = sum(self.sumay)
        
        rospy.loginfo("Path_length: %.3f m"%path_length)
        rospy.loginfo("The Sum of aceleracion linear x: %.3f m/s^2"%sax)
        rospy.loginfo("The Sum of aceleracion linear y: %.3f m/s^2"%say)
        
        
        ############## initialization again ###################
        self.xy = [] # clear the xy list after  arriving the goal
        self.sumax = []
        self.sumay = []
        self.k = 0

        return True

      else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


if __name__ == '__main__':
    try:
      rospy.loginfo("You have reached the destination")
      map_navigation()
      rospy.spin()

    except rospy.ROSInterruptException:
      rospy.loginfo("map_navigation node terminated.")