#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray,Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
import sys
from copy import deepcopy
import math
import numpy as np
from PIL import Image, ImageDraw

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

"""
use frontier explorer to choose where to go next
- Just looking for closest frontier
-- A method of this is "binary gain method", i think
- Information gain exploration - it takes into consideration the amount of new information

use inverse sensor method to update cells in the view frustum of the sensor 
- performed by ray tracing in the grid
- based on Bresenham line drawing algorithm



Possible improvemented
- Remove rows and columns that only have out of bound values, remember to take into account the removed rows when calculating robot position in the array map
- merge cells, remember take into account lose cells 

"""


class Display():
    def __init__(self):
        #self.ex = Explore()
        #self.test1()
        #self.display()

        #self.test2()
        pass

    def test1(self):
        p1 = Pose(Point(0, 0, 0), Quaternion(0,0,0,0))
        self.ex.update_exploration_map([], p1)
        self.ex.calc_next_goal(p1)

    def test2(self):
        map1 = np.zeros((200, 200), dtype = np.uint8)
        img = Image.fromarray(map1.T, 'L')
        ImageDraw.Draw(img).polygon([(1, 1), (1, 100), (100, 100)], outline=200, fill=200)
        img.show()

    def display(self, map1):
        #map1 = self.ex.exploration_map
        
        for i in range(len(map1)):
            for j in range(len(map1[0])):
                if map1[i, j] == 1:
                    map1[i][j] = 100
                elif map1[i, j] == 2:
                    map1[i, j] = 255

        img1 = Image.fromarray(map1.T, 'L')
        img1.show()

        
class Explore():
    def __init__(self, shrink_factor):
        self.shrink_factor = shrink_factor
        self.pose = None
        self.map_set = False
        
        #used to see which cells have been explored
        # 0=obstacle,outOfBounds 1=unexplored, 2=explored
        self.exploration_map = None
        self.img = None
        self.grid = None
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        
        #used for pathfinding and reseting exploration map
        #0=obstacle 1=free
        self.default_map = None 
        
        rospy.loginfo("Waiting for a map...")
        try:
            occ_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (occ_map.info.width, occ_map.info.height,
                       occ_map.info.resolution))

        self.map_width = occ_map.info.width
        self.map_height = occ_map.info.height
        self.map_resolution = occ_map.info.resolution
        self.map_data = occ_map.data
        self.map_origin_x = ( occ_map.info.origin.position.x +
                         (self.map_width / 2.0) * self.map_resolution*self.shrink_factor )
        self.map_origin_y = ( occ_map.info.origin.position.y +
                         (self.map_height / 2.0) * self.map_resolution*self.shrink_factor )

        self._odometry_sub = rospy.Subscriber("/odom", Odometry, self._odometry_callback, queue_size=1)
        

    def _odometry_callback(self, odo):

        if self.map_set:
            self.update_exploration_map([], odo.pose.pose)
            goal = self.calc_next_goal(odo.pose.pose)
            rospy.loginfo("next move: {}".format(goal))
            #rospy.sleep(1)

        
    def set_robot_location(self, p):
        """
        Args:
            p (Pose): position and oreintation of the robot
        """
        self.pose = p

    def restart_exploration(self):
        self.exploration_map = self.default_map
        self.img = Image.fromarray(self.exploration_map.T, 'L')
        self.map_set = True

        
    def set_map(self):
        self.map_set = false
        self.default_map = np.zeros((self.map_height, self.map_width), dtype = np.uint8)
        
        for i in range(self.map_height):
            for j in range(self.map_width):
                cell = self.map_data[i*self.map_width + j]

                if cell < 0.196 and cell >= 0:
                    #not explored
                    self.default_map[i][j] = 1

        self.grid = Grid(matrix = self.default_map)
        self.restart_exploration()

        
    def set_map_and_reduce(self):
        #reduce the resolution as well
        self.map_set = False        
        self.default_map = np.zeros((self.map_height/self.shrink_factor,
                                self.map_width/self.shrink_factor),
                               dtype = np.uint8)
        
        for i in range(0, len(self.default_map)):
            for j in range(0, len(self.default_map[0])):
                sum = 0
                for k in range(i*self.shrink_factor, i*self.shrink_factor+self.shrink_factor):
                    for l in range(j*self.shrink_factor, j*self.shrink_factor + self.shrink_factor):
                        cell = self.map_data[k*self.map_width + l]
                        if cell < 0.196 and cell >= 0:
                            sum = sum + 1
                        
                if sum == math.pow(self.shrink_factor,2):
                    self.default_map[i, j] = 1

        self.grid = Grid(matrix = self.default_map)
        self.restart_exploration()
        
        

    def update_exploration_map(self, scan_data, pose):
        # does not use  the scan_data
        # assume that camera is not abstructed
        # can be improved by taking the scan_data

        angle_of_vision = math.pi/2
        range_of_vision = 1
        robot_orientation = self.getHeading(pose.orientation)
        robot_position = pose.position

        """
        a       b
        --------
        \      /
         \    /
          \  /
           \/
            c
        
        c = the position of robot
        triangle = what the robot can see with the camera
        """
        
        if robot_orientation - angle_of_vision/2 < -math.pi:
            b_angle = math.pi + (math.pi + (robot_orientation - angle_of_vision/2))
        else:
            b_angle = robot_orientation - angle_of_vision/2
        
        b_x = range_of_vision*math.cos(b_angle) + robot_position.x
        b_y = range_of_vision*math.sin(b_angle) + robot_position.y

        
        if robot_orientation + angle_of_vision/2 > math.pi:
            a_angle =  -(math.pi - ((robot_orientation + angle_of_vision/2) - math.pi))
        else:
            a_angle = robot_orientation + angle_of_vision/2
        
        a_x = range_of_vision*math.cos(a_angle) + robot_position.x
        a_y = range_of_vision*math.sin(a_angle) + robot_position.y
        
        #transform the cords to array space
        b_x, b_y = self.translate_coord(b_x, b_y)
        a_x, a_y = self.translate_coord(a_x, a_y)
        c_x, c_y = self.translate_coord(robot_position.x, robot_position.y)

        #set cells to 2 (explored)
        ImageDraw.Draw(self.img).polygon([(a_y, a_x), (b_y, b_x), (c_y, c_x)], outline=2, fill=2)




        #rospy.loginfo("update_exploration_map. robot_o: {}, a_angle: {}, b_angle: {}".format(robot_orientation, a_angle, b_angle))
        #rospy.loginfo("update_exploration_map. a: ({},{}), b: ({},{}), c: ({},{})".format(a_x, a_y, b_x, b_y, c_x, c_y))




        
    def calc_next_goal(self, pose):
        #where the search will start (the location of robot/pose)
        x, y = self.translate_coord(pose.position.x, pose.position.y)

        self.exploration_map = np.array(self.img).T
        
        level = 0
    
        while True:
            x_start = np.max([x - level, 0])
            x_end = np.min([x + level, self.map_width - 1])
            y_start = np.max([y - level, 0])
            y_end = np.min([y + level, self.map_height - 1])

            #for each: if not explored and can be reached
            i = y_start
            j = x_start
            while j < x_end:
                if j < self.map_width and self.exploration_map[i][j] == 1:
                    if self.is_reachable(x, y, j, i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                j = j + 1

            i = y_end
            j = x_start
            while i < y_end:
                if i < self.map_height and self.exploration_map[i][j] == 1:
                    if self.is_reachable(x, y, j, i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                i = i + 1

            i = y_end
            j = x_end
            while j > x_start:
                if j >= 0 and self.exploration_map[i][j] == 1:
                    if self.is_reachable(x, y, j, i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                j = j - 1

            i = y_start
            j = x_end
            while i > y_start:
                if i >= 0 and self.exploration_map[i][j] == 1:
                    if self.is_reachable(x, y, j, i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                i = i - 1

            level = level +1

            #worst case
            if level > self.map_width and level > self.map_height:
                rospy.logwarn("Goal not found, restarting on level: {}".format(level))
                self.restart_exploration()
                level = 0
  
            

    def translate_coord(self, x, y):
        x = int(((x - self.map_origin_x)/(self.map_resolution*self.shrink_factor) + 0.5)
                + self.map_width/2.0)
        y = int(((y - self.map_origin_y)/(self.map_resolution*self.shrink_factor) + 0.5)
                + self.map_height/2.0)
        return (x, y)


    def is_reachable(self, x, y, x2, y2):
        self.grid.cleanup()
        start = self.grid.node(x, y)
        end = self.grid.node(x2, y2)

        path, runs = self.finder.find_path(start, end, self.grid)

        #rospy.loginfo("operations: {}, path length: {}".format(runs, path))
        
        return len(path) > 0














    
    
    #Not used
    def rm_dup(arr):
        # from wallhugger script
        # removes duplicates if the first n elements of the array are known to be duplicates
        # first value is always the erroneous measurement
        iters =len(arr)
        # count how many duplicates there are
        for i in range(iters):
            if arr[i] == arr[0]:
                pass # count as duplicate
            else:
                break
            fixed_arr = arr[i:]
        return fixed_arr






    def getHeading(self, q):
        """
        from pf_localisation.util.py
        Get the robot heading in radians from a Quaternion representation.
    
        :Args:
            | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
        :Return:
            | (double): Equivalent orientation about the z-axis in radians
        """
        yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw



    def test(self):
        map = np.ones((10, 10), dtype = np.uint8)
        map[1,1] = 0
        map[1,2] = 0
        map[2,1] = 0
        map[3,1] = 0
        map[1,3] = 0
        map[1,4] = 0
        map[3,2] = 0
        map[3,3] = 0
        map[1,5] = 0
        map[2,4] = 0
        map[3,4] = 0

        map[7,7] = 0
        map[7,8] = 0
        map[7,9] = 0
        map[8,7] = 0
        map[9,7] = 0
        grid = Grid(matrix=map)
        start = grid.node(0, 0)
        end = grid.node(0, 0)

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        rospy.loginfo("operations: {}, path length: {}".format(runs, path))
        rospy.loginfo(grid.grid_str(path=path, start=start, end=end))
        rospy.loginfo("walkable: {}".format(grid.walkable(9, 9)))

        



    """
    def set_exploration_map(self, occ_map):
        
        #converts map occ_map to exploration map
        #O(n^2 + n)
        # removing rows and columns is actually a bad idea, cause then the coordinates cannot be converted that easily from occ_map
        
        
        to_delete_columns = [True]*occ_map.info.width
        
        for i in range(occ_map.info.height):
            delete_row = True
            for j in range(occ_map.info.width):
                cell = occ_map.data[i*occ_map.info.width + j]
                
                #if the cell can be visited by the robot
                if cell < 0.196 and cell >=0:
                    delete_row = False
                    to_delete_columns[j] =False
                    #not explored
                    self.def_exploration_map[i][j] = 0
                else:
                    #out of bounds
                    self.def_exploration_map[i][j] = -1
    
            #delete row that have no useful information
            if delete_row == True:
                self.def_exploration_map.pop(i)

        #delete columns that have no useful infomration
        for j in to_delete_columns[::-1]:
            if to_delete_columns:
                [x.pop(j) for x in self.def_exploration_map]
    """
        
if __name__ == '__main__':
    rospy.init_node("explorer")
    d = Display()
    e = Explore(3)
    #self.set_map()
    e.set_map_and_reduce()
    rospy.spin()
    d.display(np.array(e.img).T)

