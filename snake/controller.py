#!/usr/bin/python

import rospy
import actionlib
import nav_util
import math
import explore

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class controller(object):
    def __init__(self):
        self.exploring_topic_name = "exploring"
        self.or_topic_name = "or"
        self.speech_topic_name = "voice_commands"

        self._or_subscriber = rospy.Subscriber(self.or_topic_name, PoseStamped,
                                                  self._or_callback,
                                                  queue_size=1)

        self.or_to_map_subscriber = rospy.Subscriber("/or_map", PoseStamped,
                                                     self._map_callback,
                                                     queue_size=1)

        self._speech_subscriber = rospy.Subscriber(self.speech_topic_name, String,
                                                  self._speech_callback,
                                                  queue_size=1)

        self._exploring_publisher = rospy.Publisher(self.exploring_topic_name,
                                                    String, queue_size = 1)

        self._exploring_subscriber = rospy.Subscriber("where_to_go", PoseWithCovarianceStamped, self._exploring_callback, queue_size = 1)

        
        #"apple": {"pose": None, "collected": False}}
        self.object_list = {"mouse": {"pose": None, "collected": False}}
        #"apple" : {"pose": None, "collected": False}}
        self.map_list = {"mouse": {"pose": None}} 
        self.priority_list = ["mouse"]
        self.current_target = {"name": self.priority_list[0]}

        self.nav = nav_util.nav_util()
        self.distance_threshold = 0.35


        
    def _exploring_callback(self, Pose):
        self.nav.go_to_pose(Pose)
        
        
    def _or_callback(self, pose):
        rospy.loginfo("in callback")
        if (self.object_list[pose.header.frame_id]["pose"] == None):
            self.object_list[pose.header.frame_id]["pose"] = pose
            rospy.loginfo("assigning pose")
            rospy.loginfo(self.current_target["name"])
        if(pose.header.frame_id == self.current_target["name"]):
            if(self.distance_robot_object(pose) < self.distance_threshold and nav.get_goalStatus() == 1):
                self.nav.cancel_goal()
                self.object_list[self.current_target["name"]]["collected"] = True
                current_target = {"name":self.priority_list[0]}
                del self.priority_list[0]
                if(self.object_list[current_target["name"]]["pose"] != None):
                    self.nav.go_to_pose(object_list[current_target["name"]]["pose"])
                else:
                    self._exploring_publisher.publish("explore")
            elif(nav.get_goalStatus() != 1):
                self.nav.go_to_pose(pose)

    def _speech_callback(self, msg):
        if(msg == "begin"):
            self._exploring_publisher.publish("explore")
        elif(msg == "stop"):
            nav.cancel_goal()
            self._exploring_publisher.publish("stop")

    def _map_callback(self, pose):
        # save the position of the object in the map
        if (self.map_list[pose.header.frame_id]["pose"] == None):
            self.map_list[pose.header.frame_id]["pose"] = pose

    def distance_robot_object(self, object_pose):
        x = object_pose.pose.position.x
        y = object_pose.pose.position.y
        dist = math.sqrt(x**2 + y**2)
        rospy.loginfo(dist)
        return dist


if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("controller")
    controller = controller()
    rospy.spin()

