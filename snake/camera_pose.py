#!/usr/bin/env python
import os
import sys
from tf.msg import tfMessage
import rospy
import math
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    """Calculates and displays the Camera to base_link statis transofrmation arguments, assuming there's no X,Y offset
    i.e. the camera is directly above the base_link and the object always appears at a constant distance xc."""
    rospy.init_node('camera_pose')
    listener = tf.TransformListener()

    camX = 0
    camY = 0
    camZ = 0
    camQX = 0
    camQY = 0
    camQZ = 0

    # distance of marker from robot base in meters
    xc = 0.59
    # empty array of distance values from camera to marker
    b = []
    n = 30

    rate = rospy.Rate(10.0)
    #rospy.loginfo('works so far')
    while len(b)<=n:
        try:
            (trans,rot) = listener.lookupTransform('/camera_rgb_optical_frame', '/object_0', rospy.Time(0))
            rospy.loginfo(trans)
            b.append(trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #rospy.loginfo(trans)

    mean_b = sum(b)/n
    sin_a = float(xc)/float(mean_b)
    a = math.asin(sin_a)
    beta = math.radians(90-math.degrees(a))
    camZ = math.sqrt(mean_b**2-xc**2)
    camQY=beta

    tf_params = [camX, camY, camZ, camQX, camQY, camQZ]

    rospy.loginfo(tf_params)