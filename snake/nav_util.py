#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

from copy import deepcopy

class nav_util(object):
	def __init__(self):
		self.estimatedpose = PoseWithCovarianceStamped()
		self._pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
		                                          self._pose_callback,
		                                          queue_size=1)
		self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		rospy.loginfo("nav_util waiting for action server")
		self.move_base_client.wait_for_server()
		rospy.loginfo("server found")

	def _pose_callback(self, pose):
		self.estimatedpose = pose

	def where_am_i(self):
		return self.estimatedpose

	def go_to_pose(self, pose):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = pose.pose.pose
		self.move_base_client.send_goal(goal)
		#self.move_base_client.wait_for_result()

	def cancel_goal(self):
		self.move_base_client.cancel_goal()

	def get_goalStatus(self):
		return self.move_base_client.get_state()