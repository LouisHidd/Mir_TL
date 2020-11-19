#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import matplotlib.path as mpltPath
import sys
import rosbag

from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
# from matplotlib import pyplot as plt
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


class jackal_explore:
	def __init__(self):
		self.costmap = OccupancyGrid()

		self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.mapCb)
		
		

	def mapCb(x, y):
		rospy.loginfo(y)
if __name__ == "__main__":
    rospy.init_node('explore_lt') #make node
    rospy.sleep(1)
    gc = jackal_explore()
    rospy.sleep(1)
    rospy.sleep(0.5)
    # plt.figure()
    # plt.show()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")