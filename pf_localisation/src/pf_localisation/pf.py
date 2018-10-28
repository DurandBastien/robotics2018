
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from tf import transformations
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time

import numpy as np


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 # Odometry model y axis (side-to-side) noise        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise  
        NUMBER_OF_PARTICLES = 100
        new_particlecloud = PoseArray()
        noise_x = np.random.normal(0,initialpose.pose.covariance[0],NUMBER_OF_PARTICLES)
        noise_y = np.random.normal(0,initialpose.pose.covariance[7],NUMBER_OF_PARTICLES)
        noise_theta = np.random.normal(0,initialpose.pose.covariance[35],NUMBER_OF_PARTICLES)
        # particle_theta = np.random.random() * 360
        # quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
        # particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
        # new_particlecloud.poses.append(Pose(Point(0,0,0), particle_quaternion))
        # new_particlecloud.poses.append(Pose(Point(0,0,1), particle_quaternion))
        for ith_particule in range(NUMBER_OF_PARTICLES):
            initialpose_quaternion = [initialpose.pose.pose.orientation.x, initialpose.pose.pose.orientation.y, initialpose.pose.pose.orientation.z, initialpose.pose.pose.orientation.w]
            particle_theta = noise_theta[ith_particule] * 360 + transformations.euler_from_quaternion(initialpose_quaternion)[0]
            particle_theta = particle_theta % 360
            particle_pos = Point(initialpose.pose.pose.position.x, initialpose.pose.pose.position.y, initialpose.pose.pose.position.z)
            particle_pos.x += noise_x[ith_particule]
            particle_pos.y += noise_y[ith_particule]
            quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
            particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
            new_particlecloud.poses.append(Pose(particle_pos, particle_quaternion))
        return new_particlecloud
        # return self.particlecloud
 
    
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
        return self.estimatedpose.pose.pose