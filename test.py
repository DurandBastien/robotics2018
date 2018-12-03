#!/usr/bin/env python
"""
Node used to get information from tf messages and object recognition.
"""
import os
import sys
from tf.msg import tfMessage
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import (PoseStamped, TransformStamped, Transform)

class VisionNode():
    """The Object recognition package must be running in the background and publishing messages for this node to work"""
    def __init__(self):
        # Perhaps publish the object on a separate topic?
        self.or_pub = rospy.Publisher('/or', PoseStamped, queue_size=3)
        self._or_subscriber = rospy.Subscriber("or", PoseStamped,
                                                  self._or_callback,
                                                  queue_size=1)

    def publi(self):
        self.or_pub.publish(PoseStamped())

    def _or_callback(self, pose):
      rospy.loginfo("oooooooooooooooo")


if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("vision")
    node = VisionNode()
    rospy.spin()