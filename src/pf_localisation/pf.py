
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
        self.ODOM_ROTATION_NOISE = 5 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.04 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.04 # Odometry model y axis (side-to-side) noise        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 80     # Number of readings to predict

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

        self.add_random_particle_distribution(1, NB_PARTICLES_SPREAD, new_particlecloud)
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
        # for counter, pose in enumerate(weighted_poses):
        #     weighted_poses[counter][1] = pose[1]/weight_sum

        # n = len(weighted_poses) - 100
        n = 400
        # random.shuffle(weighted_poses)
        resampled_poses = self.resample(weighted_poses, n)

        resampled_cloud = PoseArray()
        for particle in resampled_poses:
            new_particle = deepcopy(particle[0])
            # sample gaussian noise with sigma = noise parameters, and mean = location
            new_particle.position.x = random.gauss(new_particle.position.x,self.ODOM_TRANSLATION_NOISE)
            new_particle.position.y = random.gauss(new_particle.position.y,self.ODOM_DRIFT_NOISE)
            # sample gaussian noise from the rotation parameter
            # but does that make mathematical sense? What are the rotation noise units?
            new_particle.orientation = rotateQuaternion(new_particle.orientation,(math.radians(random.uniform(-5, 5))))
            resampled_cloud.poses.append(new_particle)

        self.add_random_particle_distribution(1, 30, resampled_cloud)
        self.particlecloud = resampled_cloud
	 

    """def estimate_pose(self):
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
        # return self.estimatedpose.pose.pose"""


    #Each item in the list points should be a 4-tuple (x, y, qz, qw)
    #OR a 4-tuple (x, y, qz, qw, weight)
    def estimate_pose(self):
        points = []
        for item in self.particlecloud.poses:
            points.append((item.position.x, item.position.y, item.orientation.z, item.orientation.w))
        clusters = self.dbscan(points, 1.5, 1) #PARAMETERS SELECTED ARBITRARILY, MUST BE REFINED.
        maxSize = 0
        biggestCluster = []
        for cluster in clusters:
            if self.weightOfCluster(cluster) > maxSize:
                maxSize = self.weightOfCluster(cluster)
                biggestCluster = cluster
        x = 0
        y = 0
        qz = 0
        qw = 0
        #print(biggestCluster)
        for point in biggestCluster:
            x += (point[0] * self.weightOfPoint(point))
            y += (point[1] * self.weightOfPoint(point))
            qz += (point[2] * self.weightOfPoint(point))
            qw += (point[3] * self.weightOfPoint(point))
        estimated_pose = Pose()
    	estimated_pose.position.x = x/maxSize
    	estimated_pose.position.y = y/maxSize      
        estimated_pose.orientation.z = qz/maxSize
       	estimated_pose.orientation.w = qw/maxSize
       	return estimated_pose

    def weightOfCluster(self, cluster):
        weight = 0
        for point in cluster:
            weight += self.weightOfPoint(point)
        return weight

    def weightOfPoint(self, point):
        return 1 #If using weighted particles, comment out this line instead of the next line.
        #return point[4]

    #Runs a DBSCAN to find clusters in the points.
    #Parameters: points = list of tuples (x, y) representing the points.
        #radius = maximum distance at which points are considered nearby.
        #minPts = minimum number of points adjacent to a point for it to be a core point.
    def dbscan(self, points, radius, minPts):
        corepoints = []
        for point in points:
            if(self.getNumberNearby(point, points, radius) >= minPts):
                corepoints.append(point)
        clusters = []
        while(len(corepoints) > 0):
            clusters.append(self.createCluster(points, corepoints, corepoints[0], radius))
        return clusters

    def createCluster(self, points, corePts, starter, radius):
        corePts.remove(starter)
        points.remove(starter)          
        output = []
        output.append(starter)
        for point in points:
            d = self.distance(point, starter)
            if (point in corePts) and (d < radius):
                output = output + self.createCluster(points, corePts, point, radius)
            elif (d < radius) and not (point in output):
                output.append(point)
        return output                 
    
    def getNumberNearby(self, point, otherPoints, radius):
        counter = 0
        for pointB in otherPoints:
            if(self.distance(pointB, point) < radius):
                counter += 1
        return counter - 1 #subtract 1 so as not to count the particle itself lol.

    def distance(self, pointA, pointB):
        return math.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)



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

    def add_random_particle_distribution(self, orientation_noise, nb_particles, particles_cloud):
        noise_theta = np.random.normal(0, orientation_noise, nb_particles)
        chosen_cells = self.free_cells[np.random.choice(self.free_cells.shape[0], nb_particles), :]
        for ith_particle,cell in enumerate(chosen_cells):
            particle_pos = Point(-18.65 + cell[0]*0.05, -20.075 + cell[1]*0.054, 0)
            particle_theta = (noise_theta[ith_particle] * 360) % 360
            quaternion_array = transformations.quaternion_from_euler(0, 0, particle_theta)
            particle_quaternion = Quaternion(quaternion_array[0], quaternion_array[1], quaternion_array[2], quaternion_array[3])
            particles_cloud.poses.append(Pose(particle_pos, particle_quaternion))

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
