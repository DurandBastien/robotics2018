#!/usr/bin/env python

import rospy
import actionlib
import nav_util

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

class controller(object):
	def __init__(self):
		self.exploring_topic_name = "exploring"
		self.or_topic_name = "or"
		self.speech_topic_name = "speech"

		self._or_subscriber = rospy.Subscriber(self.or_topic_name, PoseWithCovarianceStamped,
		                                          self._or_callback,
		                                          queue_size=1)

		self._speech_subscriber = rospy.Subscriber(self.speech_topic_name, String,
		                                          self._speech_callback,
		                                          queue_size=1)

		self._exploring_publisher = rospy.Publisher(self.exploring_topic_name,
                                                    String, queue_size = 1)
		self.object_list = {"obj1":{"pose":None, "collected":False}}
		self.priority_list = ["obj1"]
		current_target = {"name":self.priority_list[0]}
		del self.priority_list[0]

		self.nav = nav_util.nav_util()

		self.distance_treshold = 0.5

	def _or_callback(self, pose):
		if(object_list[pose.header.object]["pose"] == None):
			object_list[pose.header.object]["pose"] = pose
		if(pose.header.object == self.current_target["name"]):
			if(distance_robot_object(pose) < self.distance_treshold and nav.get_goalStatus() == 1):
				self.nav.cancel_goal()
				self.object_list[current_target["name"]]["collected"] = True
				current_target = {"name":self.priority_list[0]}
				del self.priority_list[0]
				if(object_list[current_target["name"]]["pose"] != None):
					self.nav.go_to_pose(object_list[current_target["name"]]["pose"])
				else:
					self._exploring_publisher.publish("explore")
			elif(nav.get_goalStatus() != 1):
				self.nav.go_to_pose(pose)
			

	def _speech_callback(self, msg):
		if(msg == "start"):
			self._exploring_publisher.publish("explore")
		elif(msg == "stop"):
			nav.cancel_goal()
			self._exploring_publisher.publish("stop")	

	def distance_robot_object(object_pose):
		pass

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("controller")
    controller = controller()
    rospy.spin()

