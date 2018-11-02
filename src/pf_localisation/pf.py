
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from tf import transformations
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
import random

from time import time

import numpy as np
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid, Odometry


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 5 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 80     # Number of readings to predict
        
     	# self._stage_ros_pose_subscriber = rospy.Subscriber("/base_pose_ground_truth", Odometry,
        #											self.stage_pose_callback,
        #                                           queue_size=1)
    
    def stage_pose_callback(self, pose):
        self.estimatedpose.pose.pose = deepcopy(pose.pose.pose)

    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise  
        NB_PARTICLES_AROUND_INITIAL_P = 5
        NB_PARTICLES_SPREAD = 150
        SPREADING_VARIANCE = 5
        new_particlecloud = PoseArray()
        noise_x = np.random.normal(0,initialpose.pose.covariance[0],NB_PARTICLES_AROUND_INITIAL_P)
        noise_y = np.random.normal(0,initialpose.pose.covariance[7],NB_PARTICLES_AROUND_INITIAL_P)
        noise_theta = np.random.normal(0,initialpose.pose.covariance[35],NB_PARTICLES_AROUND_INITIAL_P)
        for ith_particule in range(NB_PARTICLES_AROUND_INITIAL_P):
            initialpose_quaternion = [initialpose.pose.pose.orientation.x, initialpose.pose.pose.orientation.y, initialpose.pose.pose.orientation.z, initialpose.pose.pose.orientation.w]
            particle_theta = noise_theta[ith_particule] * 360 + transformations.euler_from_quaternion(initialpose_quaternion)[2]
            particle_theta = particle_theta % 360
            particle_pos = Point(initialpose.pose.pose.position.x, initialpose.pose.pose.position.y, initialpose.pose.pose.position.z)
            particle_pos.x += noise_x[ith_particule]
            particle_pos.y += noise_y[ith_particule]
            quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
            particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
            new_particlecloud.poses.append(Pose(particle_pos, particle_quaternion))

        noise_x = np.random.normal(0,SPREADING_VARIANCE,NB_PARTICLES_SPREAD)
        noise_y = np.random.normal(0,SPREADING_VARIANCE+3,NB_PARTICLES_SPREAD)
        noise_theta = np.random.normal(0,1,NB_PARTICLES_SPREAD)
        for ith_particule in range(NB_PARTICLES_SPREAD):
            initialpose_quaternion = [initialpose.pose.pose.orientation.x, initialpose.pose.pose.orientation.y, initialpose.pose.pose.orientation.z, initialpose.pose.pose.orientation.w]
            particle_theta = noise_theta[ith_particule] * 360
            particle_pos = Point(0, 0, 0)
            particle_pos.x += noise_x[ith_particule]
            particle_pos.y += noise_y[ith_particule]
            quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
            particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
            new_particlecloud.poses.append(Pose(particle_pos, particle_quaternion))
        return new_particlecloud
        # return self.particlecloud
 
    
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        weighted_poses = []
        weight_sum = 0
        for particle in self.particlecloud.poses:
            #rospy.loginfo('getting weight')
            weight = self.sensor_model.get_weight(scan, particle)
            rospy.loginfo(weight)
            weighted_poses.append([particle, weight])
            weight_sum += weight
        
        #www=[]
        #normalisation
        for counter, pose in enumerate(weighted_poses):
            weighted_poses[counter][1] = pose[1]/weight_sum
            #rospy.loginfo(weighted_poses[counter][1])
            www.append(pose[1]/weight_sum)
	    #weighted_poses.append([particle, 1.0/len(self.particlecloud.poses)])
            #resampled_poses = self.resample(weighted_poses, len(weighted_poses))

        #rospy.loginfo(weighted_poses)
        #rospy.loginfo(www)
        
	"""
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
	"""	

	n = len(weighted_poses)
        resampled_poses = self.resample(weighted_poses, n)
		
        resampled_cloud = PoseArray()
        for particle in resampled_poses:
            new_particle = deepcopy(particle[0])
            # sample gaussian noise with sigma = noise parameters, and mean = location
            new_particle.position.x = random.gauss(new_particle.position.x,self.ODOM_TRANSLATION_NOISE)
            new_particle.position.y = random.gauss(new_particle.position.y,self.ODOM_DRIFT_NOISE)
            # sample gaussian noise from the rotation parameter
            # but does that make mathematical sense? What are the rotation noise units?
            new_particle.orientation = rotateQuaternion(new_particle.orientation,(math.radians(random.gauss(0,5))))
            resampled_cloud.poses.append(new_particle)
	

        self.particlecloud = resampled_cloud
        #rospy.loginfo(resampled_cloud)
	 

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
        #return self.estimatedpose.pose.pose



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
