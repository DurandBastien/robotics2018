#!/usr/bin/python

from nav_msgs.msg import OccupancyGrid, Odometry
from nav_util_tail import nav_util
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point


rospy.init_node("node")
nav = nav_util()
ocupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
nav.init_map(ocupancy_map)
p = nav.where_am_i()
p.pose.pose.position.x = 1
p.pose.pose.position.y = 4
nav.go_to_pose(p)
