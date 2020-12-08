#!/usr/bin/env python
"""Demo of sending navigation goals using actionlib. 

Moves the robot to a designated point relative to the its current
position.

Usage: 

action_nav.py TARGET_X TARGET_Y [THETA]

Author: Nathan Sprague
Version: 2/14/2019

"""
import sys
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus



def create_goal_message(x_target, y_target, theta_target, frame='map'):
    """Create a goal message in the indicated frame"""

    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
    # Create a goal message ...
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    return goal

class NavNode(object):

    def __init__(self):
        """ Set up the node. """

        rospy.init_node('nav_node')

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        
    def goto_point(self, x_target, y_target, theta_target=0):
        """ Move to a location relative to the robot's current position """

        rospy.loginfo("navigating to: ({},{})".format(x_target, y_target))

        goal = create_goal_message(x_target, y_target, theta_target,
                                   'base_link')

        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()

        rospy.loginfo("Sending goal.")
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")

        # Check in after a while to see how things are going.
        rospy.sleep(1.0)
        rospy.loginfo("Status Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "ACTIVE", "SUCCEEDED" or "ABORTED"
        state_name = actionlib.get_name_of_constant(GoalStatus,
                                                    self.ac.get_state())
        rospy.loginfo("State      : {}".format(state_name))

        # Wait until the server reports a result.
        self.ac.wait_for_result()
        rospy.loginfo("Status Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "SUCCEEDED" or "ABORTED"
        state_name = actionlib.get_name_of_constant(GoalStatus,
                                                    self.ac.get_state())
        rospy.loginfo("State      : {}".format(state_name))



if __name__ == "__main__":
    nav_node = NavNode()
    if len(sys.argv) == 3:
        nav_node.goto_point(float(sys.argv[1]), float(sys.argv[2]))
    else:
        nav_node.goto_point(float(sys.argv[1]), float(sys.argv[2]),
                            float(sys.argv[3]))
