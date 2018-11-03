
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from tf import transformations
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
import random

from time import time
from copy import deepcopy

import numpy as np

from nav_msgs.msg import OccupancyGrid, Odometry


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 1 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 1 # Odometry model y axis (side-to-side) noise        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

        # self._stage_ros_pose_subscriber = rospy.Subscriber("/base_pose_ground_truth", Odometry,
        #                                           self.stage_pose_callback,
        #                                           queue_size=1)
    
    def stage_pose_callback(self, pose):
        self.estimatedpose.pose.pose = deepcopy(pose.pose.pose)
       
    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise  
        NB_PARTICLES_AROUND_INITIAL_P = 100
        NB_PARTICLES_SPREAD = 400
        SPREADING_VARIANCE = 5
        new_particlecloud = PoseArray()

        particle_index = 0
        noise_theta = np.random.normal(0,initialpose.pose.covariance[35],NB_PARTICLES_AROUND_INITIAL_P)
        while particle_index < NB_PARTICLES_AROUND_INITIAL_P:
            particle_pos = Point(initialpose.pose.pose.position.x, initialpose.pose.pose.position.y, initialpose.pose.pose.position.z)
            particle_pos.x += np.random.normal(0,initialpose.pose.covariance[0])
            particle_pos.y += np.random.normal(0,initialpose.pose.covariance[7])
            rospy.loginfo("First loop, x: {}, y: {}".format(particle_pos.x, particle_pos.y))
            if self.in_map(particle_pos.x, particle_pos.y):
                initialpose_quaternion = [initialpose.pose.pose.orientation.x, initialpose.pose.pose.orientation.y, initialpose.pose.pose.orientation.z, initialpose.pose.pose.orientation.w]
                particle_theta = noise_theta[particle_index] * 360 + transformations.euler_from_quaternion(initialpose_quaternion)[2]
                particle_theta = particle_theta % 360
                quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
                particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
                new_particlecloud.poses.append(Pose(particle_pos, particle_quaternion))
                particle_index = particle_index + 1

        particle_index = 0 
        noise_theta = np.random.normal(0,1,NB_PARTICLES_SPREAD)
        while particle_index < NB_PARTICLES_SPREAD:
            particle_pos = Point(0, 0, 0)
            particle_pos.x += np.random.normal(0,SPREADING_VARIANCE)
            particle_pos.y += np.random.normal(0,SPREADING_VARIANCE+3)
            rospy.loginfo("First loop, x: {}, y: {}".format(particle_pos.x, particle_pos.y))
            if self.in_map(particle_pos.x, particle_pos.y):
                initialpose_quaternion = [initialpose.pose.pose.orientation.x, initialpose.pose.pose.orientation.y, initialpose.pose.pose.orientation.z, initialpose.pose.pose.orientation.w]
                particle_theta = noise_theta[particle_index] * 360
                quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
                particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
                new_particlecloud.poses.append(Pose(particle_pos, particle_quaternion))
                particle_index = particle_index + 1
                
        return new_particlecloud
        # return self.particlecloud
 
    
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        weighted_poses = []
        weight_sum = 0
        for particle in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan, particle)
            weighted_poses.append([particle, weight])
            weight_sum += weight

        #normalisation
        for counter, pose in enumerate(weighted_poses):
            weighted_poses[counter][1] = pose[1]/weight_sum

	    #weighted_poses.append([particle, 1.0/len(self.particlecloud.poses)])
            #resampled_poses = self.resample(weighted_poses, len(weighted_poses))
        n = len(weighted_poses) - 100

        random.shuffle(weighted_poses)
        # resampled_poses = self.resample(weighted_poses, n)
        resampled_poses = self.resample_particles(weighted_poses, n)

        resampled_cloud = PoseArray()
        for particle in resampled_poses:
            resampled_cloud.poses.append(deepcopy(particle))

        noise_x = np.random.normal(0,5,100)
        noise_y = np.random.normal(0,5+3,100)
        noise_theta = np.random.normal(0,1,100)
        for ith_particule in range(100):
            initialpose_quaternion = [self.estimatedpose.pose.pose.orientation.x, self.estimatedpose.pose.pose.orientation.y, self.estimatedpose.pose.pose.orientation.z, self.estimatedpose.pose.pose.orientation.w]
            particle_theta = noise_theta[ith_particule] * 360
            particle_pos = Point(0, 0, 0)
            particle_pos.x += noise_x[ith_particule]
            particle_pos.y += noise_y[ith_particule]
            quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
            particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
            resampled_cloud.poses.append(Pose(particle_pos, particle_quaternion))

        self.particlecloud = resampled_cloud
	 

    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        

    	# Better approximations could be made by doing some simple clustering,
    	# e.g. taking the average location of half the particles after 
    	# throwing away any which are outliers
    	# takes input in the form of a ROS PoseArray
    	# currently averaging values
    	# averaging quanternions only makes sense if their orientations are similar. If they're not, averages are
    	# meaningless and require multiple representations (from paper below).
    	# http://www.cs.unc.edu/techreports/01-029.pdf

    	x,y,qz,qw = 0,0,0,0

    	for item in self.particlecloud.poses:
    	    # z should always be 0
    	    x += item.position.x
    	    y += item.position.y

    	    # this should need yaw only
    	    qz += item.orientation.z
    	    qw += item.orientation.w


    	n = len(self.particlecloud.poses)

    	# calculate mean values
    	mean_x = x/n
        mean_y = y/n

        mean_qz = qz/n
    	mean_qw = qw/n

    	# the other vars should be initialised to 0.0 so don't need to be defined here
    	estimated_pose = Pose()
    	estimated_pose.position.x = mean_x
    	estimated_pose.position.y = mean_y
        
        # changed the mean to the z dim
        estimated_pose.orientation.z = mean_qz
       	estimated_pose.orientation.w = mean_qw

       	return estimated_pose
        # return self.estimatedpose.pose.pose



    def resample(self, particles, n):
        totalWeight = 0
        for i in range(len(particles)):
            totalWeight = totalWeight + particles[i][1]
        rands = []
        for i in range(n + 1):
            rands.append(random.uniform(0.0, totalWeight))
        sortedRands = sorted(rands)
        counter = 0
        rCounter = 0
        output = []
        lengthOfOutput = 0
        sumSoFar = particles[0][1]
        while (lengthOfOutput < n):
            if(sortedRands[rCounter] < sumSoFar):
                output.append(particles[counter])
                rCounter = rCounter + 1
                lengthOfOutput = lengthOfOutput + 1
            else:
                if counter < (len(particles)-1):
                    counter = counter + 1
                    sumSoFar = sumSoFar + particles[counter][1]
        return output

    def resample_particles(self, particles, n):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample.
        """
        # make sure the distribution is normalized
        # self.normalize_particles()

        newParticles = []
        for i in range(n):
            # resample the same # of particles
            choice = np.random.random_sample()
            # all the particle weights sum to 1
            csum = 0 # cumulative sum
            for particle in particles:
                csum += particle[1]
                if csum >= choice:
                    # if the random choice fell within the particle's weight
                    newParticles.append(particle[0])
                    break
        return newParticles

    def in_map(self, x, y):
        #check if the coordinates are inside the allowed area of the map

        #check if the co-ordinates are inside the ranges of the map file
        if (x > self.occupancy_map.info.resolution*self.occupancy_map.info.width/2.0) or (y > self.occupancy_map.info.resolution*self.occupancy_map.info.height/2.0):
            return False

        #the origin is set so it alligns with centre
        map_origin_x = (self.occupancy_map.info.origin.position.x + (self.occupancy_map.info.width / 2.0) * self.occupancy_map.info.resolution )
        map_origin_y = (self.occupancy_map.info.origin.position.y + (self.occupancy_map.info.height / 2.0) * self.occupancy_map.info.resolution )

        #covert the coordinates to non-negative system.
        new_x = (x - map_origin_x)/self.occupancy_map.info.resolution + self.occupancy_map.info.width/2.0
        new_y = (y - map_origin_y)/self.occupancy_map.info.resolution + self.occupancy_map.info.height/2.0
        #array needs integers
        new_x = int(new_x)
        new_y = int(new_y)

        #the location of the map in the array
        i = (new_y)*self.occupancy_map.info.width + (new_x)

        #return false if it's outside the allowed area or if a location is used/occupied
        if self.occupancy_map.data[i] == -1 or self.occupancy_map.data[i] == 100:  
            return False
        else:
            return True