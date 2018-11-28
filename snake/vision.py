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
    """The Object recognition package must be running in the background and publishing messages for this node to work. In order to simulate, launch the or.launch file - this will publish arbitrary static transforms"""
    def __init__(self):
        # Perhaps publish the object on a separate topic?
        # define object handle
        self.item_list = ["object_0","object_1","object_2","object_3","object_4","object_5","object_6"]
        # create a dict to lookup object class
        self.item_dict = {"mouse" : ["object_0","object_2","object_3"], "apple" : ["object_4","object_5","object_6"]}
        self.in_map_log = ["mouse", "apple"]

        self.tfMessage = tfMessage()
        self.tf_listener = tf.TransformListener()
        rospy.loginfo('listener made')
        tf_subscriber = rospy.Subscriber("/tf", tfMessage,
                                                self._tf_callback,
                                                queue_size=1)

        self.tf_publisher = rospy.Publisher("/tf", tfMessage, queue_size=1)
        self.or_pub = rospy.Publisher('/or', PoseStamped, queue_size=3)
        self.or_map_pub = rospy.Publisher('/or_map', PoseStamped, queue_size=3)

    def _tf_callback(self, msg):
        """ called whenever a new tfMessage is received. Creates a list of detected objects
        and iterates through these to return their type and location w.r.t. map and odometry frames."""
        detected_objects = []
        for item in self.item_list:
            if self.tf_listener.frameExists(item):
                detected_objects.append(item)
        rospy.loginfo(detected_objects)
        if detected_objects:
            for item in detected_objects:
                object_h = item  # type: str
                obj_type = self.get_type(object_h)
                t = self.tf_listener.getLatestCommonTime("/camera_link", object_h)
                #rospy.loginfo("Object detected")
                #(pos, rot) = self.tf_listener.lookupTransform("/camera_rgb_optical_frame", , t)
                (pos, rot) = self.tf_listener.lookupTransform("/camera_link", object_h, t)
                # create the object pose
                obj_pos = geometry_msgs.msg.PoseStamped()
                obj_pos.header.frame_id = "/camera_link"
                obj_pos.pose.position.x = pos[0]
                obj_pos.pose.position.y = pos[1]
                obj_pos.pose.position.z = pos[2]
                obj_pos.pose.orientation.x = rot[0]
                obj_pos.pose.orientation.y = rot[1]
                obj_pos.pose.orientation.z = rot[2]
                obj_pos.pose.orientation.w = rot[3]
                obj_pose = self.tf_listener.transformPose("/odom", obj_pos)
                obj_pose.header.frame_id = obj_type

                transform = Transform()
                transform.translation = obj_pose.pose.position
                transform.rotation = obj_pose.pose.orientation
                # Insert new Transform into a TransformStamped object and add to the tf tree
                new_tfstamped = TransformStamped()
                new_tfstamped.child_frame_id = obj_type
                new_tfstamped.header.frame_id = "/odom"
                new_tfstamped.transform = transform
                new_tfstamped.header.stamp = t
                # add to tf list
                self.tf_message = tfMessage(transforms=[new_tfstamped])
                self.tf_publisher.publish(self.tf_message)
                self.or_pub.publish(obj_pose)

                if obj_type in self.in_map_log:
                    t = self.tf_listener.getLatestCommonTime("/map", object_h)
                    #(pos, rot) = self.tf_listener.lookupTransform("/camera_rgb_optical_frame", object_h, t)
                    (pos, rot) = self.tf_listener.lookupTransform("/camera_link", object_h, t)
                    # get the object pose
                    obj_pos = geometry_msgs.msg.PoseStamped()
                    obj_pos.header.frame_id = "/camera_link"
                    obj_pos.pose.position.x = pos[0]
                    obj_pos.pose.position.y = pos[1]
                    obj_pos.pose.position.z = pos[2]
                    obj_pos.pose.orientation.x = rot[0]
                    obj_pos.pose.orientation.y = rot[1]
                    obj_pos.pose.orientation.z = rot[2]
                    obj_pos.pose.orientation.w = rot[3]
                    self.object_to_map(obj_pos, obj_type, t)
                    self.in_map_log.remove(obj_type)


    def get_type(self, object_handle):
        if object_handle in self.item_dict["mouse"]:
            object_type = "mouse"
        elif object_handle in self.item_dict["apple"]:
            object_type = "apple"
        return object_type

    def object_to_map(self, pose, object_type, time):
        obj_in_map = self.tf_listener.transformPose("/map", pose)
        obj_in_map.header.frame_id = object_type + "_in_map"
        transform = Transform()
        transform.translation = obj_in_map.pose.position
        transform.rotation = obj_in_map.pose.orientation
        # Insert new Transform into a TransformStamped object and add to the tf tree
        tf_to_map = TransformStamped()
        tf_to_map.child_frame_id = object_type+"_in_map"
        tf_to_map.header.frame_id = "/map"
        tf_to_map.transform = transform
        tf_to_map.header.stamp = time
        #rospy.loginfo(new_tfstamped)
        # add to tf list
        self.or_map_pub.publish(obj_in_map)
        self.tf_message = tfMessage(transforms=[tf_to_map])
        self.tf_publisher.publish(self.tf_message)



if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("vision")
    node = VisionNode()
    rospy.spin()
