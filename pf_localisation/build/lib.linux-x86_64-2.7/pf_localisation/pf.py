from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 # Odometry model y axis (side-to-side) noise        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise
        pass
 
    
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        pass

    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
        pass
