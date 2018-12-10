#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray,Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
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
- possibly more efficient alogirthm that doesn't priotize best path

Possible improvements
- Remove rows and columns that only have out of bound values, remember to take into account the removed rows when calculating robot position in the array map
"""


OBSTACLE = 0
FREE = 1 #/unexplored
EXPLORED = 2

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
                    map1[i, j] = 100
                elif map1[i, j] == 2:
                    map1[i, j] = 255
        img1 = Image.fromarray(map1.T, 'L')
        img1.show()
        
    def display_1d(self, map1, height, width, point):
	rospy.loginfo("height: {}, width: {}".format(height, width))
	temp_map = np.zeros((height, width), dtype = np.uint8)
	for i in range(height):
            for j in range(width):
                cell = map1[i*width + j]
                if cell == 0:
                    #not explored
                    temp_map[i][j] = 100
        rospy.loginfo("Point is none: {}".format(point is None))
        if point is not None:
            temp_map[point[1]][point[0]] = 240
            #ImageDraw.Draw(img1).ellipse([(point[1]-10, point[0]-10), (point[1]+10, point[0]+10)],fill=200, outline=200)
	img1 = Image.fromarray(temp_map.T, 'L')
        img1.show()

    def how_much_explored(self, base_map, explored_map):
        no_of_available_cells = 0
        no_of_explored = 0
        for i in range(len(base_map)):
            for j in range(len(base_map[0])):
                if base_map[i][j] == FREE:
                    no_of_available_cells = no_of_available_cells + 1
                    if explored_map[i][j] == EXPLORED:
                        no_of_explored = no_of_explored + 1                        
        return no_of_explored/(no_of_available_cells*1.0)

class Explore():
    def __init__(self, shrink_factor):
        self.current_goal = None
        self.global_costmap_update = None
        self.global_costmap_update_width = None
        self.global_costmap_update_height = None        
        self.shrink_factor = shrink_factor
        self.range_of_vision = 0.6        
        self.current_odometry = None
        self.map_set = False
        self.msg = ""
        #used to see which cells have been explored
        # 0=obstacle,outOfBounds 1=unexplored, 2=explored
        self.exploration_map = None
        self.grid = None
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        self.img = None
        self.previous_b_x = None
        self.previous_b_y = None
        self.previous_a_y = None
        self.previous_a_x = None
        #used for pathfinding and reseting exploration map
        #0=obstacle 1=free
        self.default_map = None
        #local costmap
        self.local_costmap = None
        self.local_costmap_origin = None
        self.local_costmap_width = None
        self.local_costmap_height = None
        self.local_costmap_setup = False
        self.global_costmap = None        
        rospy.loginfo("Waiting for a map...")
        try:
            occ_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." % (occ_map.info.width, occ_map.info.height,
                       occ_map.info.resolution))
        self.map_width = occ_map.info.width
        self.map_height = occ_map.info.height
        self.map_resolution = occ_map.info.resolution
        self.map_data = occ_map.data
        self.map_real_origin_x = occ_map.info.origin.position.x
        self.map_real_origin_y = occ_map.info.origin.position.y
        self.map_origin_x = ( occ_map.info.origin.position.x +
                         (self.map_width / 2.0) * self.map_resolution*self.shrink_factor )
        self.map_origin_y = ( occ_map.info.origin.position.y +
                         (self.map_height / 2.0) * self.map_resolution*self.shrink_factor )
        self.set_map_and_reduce()
        self._odometry_sub = rospy.Subscriber("/odom", Odometry, self._odometry_callback, queue_size=1)
        self._exploring_sub = rospy.Subscriber("/exploring", String, self._exploring_callback, queue_size=1)
        self._pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)
        self._local_costmap_subscriber = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self._local_costmap_callback, queue_size=1)
        self._local_costmap_update_subscriber = rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate, self._local_costmap_update_callback, queue_size=1)
        self._global_costmap_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self._global_costmap_callback , queue_size=1)
        self._global_costmap_update_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, self._global_costmap_update_callback , queue_size=1) 
        self.goal_publisher = rospy.Publisher("/where_to_go", PoseWithCovarianceStamped ,queue_size=1)
        self.test_pose_publisher = rospy.Publisher("/test_pose", PoseWithCovarianceStamped ,queue_size=1)

    def _local_costmap_callback(self, costmap):
        self.local_costmap = costmap.data
        self.local_costmap_origin_position = self.translate_coord(costmap.info.origin.position.x, costmap.info.origin.position.y)
        self.local_costmap_width = costmap.info.width
        self.local_costmap_height = costmap.info.height
        self.local_costmap_setup = True

    def _local_costmap_update_callback(self, costmap):
        rospy.loginfo("local_costmap_update_callback")
        self.local_costmap = costmap.data
        #rospy.loginfo("lcmu, x, y: {}".format((costmap.x, costmap.y)))
        #rospy.loginfo("Lcmu, height, width: {}".format((costmap.height, costmap.width)))

    def _global_costmap_callback(self, costmap):
        self.global_costmap= list(costmap.data)

    def _global_costmap_update_callback(self, costmap):
        self.global_costmap_update = costmap.data
        self.global_costmap_update_height = costmap.height
        self.global_costmap_update_width = costmap.width

        for i in range(len(costmap.data)):
            y = i/costmap.width + costmap.y
            x= i%costmap.width + costmap.x
            #print("type of gcm: {}, type of gcmu: {}".format(len(self.global_costmap), len(costmap.data)))
            self.global_costmap[y*self.map_width + x] = costmap.data[i]        
        #rospy.loginfo("gcmu, x, y: {}".format((costmap.x, costmap.y)))
        #rospy.loginfo("gcmu, height, width: {}".format((costmap.height, costmap.width)))
    
    def _exploring_callback(self, msg):
        self.msg = msg.data

    def _pose_callback(self, odometry):
        self.current_odometry = odometry
        if self.map_set:
            self.update_exploration_map([], self.current_odometry.pose.pose)
        
    def _odometry_callback(self, odo):
        if self.map_set and self.current_odometry is not None and self.msg == "explore":
            copy_odo = deepcopy(self.current_odometry)
            goal_pose = self.calc_next_goal(copy_odo.pose.pose)
            self.current_goal = (goal_pose[0]*3, goal_pose[1]*3)
            rospy.loginfo("current_goal: {}".format(self.current_goal))
            rospy.loginfo("current_array_pose: {}, goal_array_pose: {}".format(self.translate_coord(copy_odo.pose.pose.position.x, copy_odo.pose.pose.position.y), goal_pose))
            goal_pose = self.translate_coord_back(goal_pose[0], goal_pose[1])
            rospy.loginfo("current_real_pose: {}, goal_real_pose: {}".format((round(copy_odo.pose.pose.position.x, 3), round(copy_odo.pose.pose.position.y, 3)), (round(goal_pose[0], 3), round(goal_pose[1], 3))))
            new_odo = copy_odo
            new_odo.pose.pose.position.x = goal_pose[0]
            new_odo.pose.pose.position.y = goal_pose[1]            
            self.goal_publisher.publish(new_odo)
        rospy.sleep(3)
        
    def restart_exploration(self):
        self.exploration_map = self.default_map
        self.img = Image.fromarray(self.exploration_map.T, 'L')
        self.map_set = True
        rospy.loginfo("setup finished")

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
        self.inflate_obstacles()
        self.grid = Grid(matrix = self.default_map)
        self.restart_exploration()
        
    def inflate_obstacles(self):
        temp_img = Image.fromarray(self.default_map.T, 'L')
        dis = 4
        for i in range(len(self.default_map)):
            for j in range(len(self.default_map[0])):
                if self.default_map[i][j] == 0:
                    #if i > dis and j > dis and i < len(self.default_map)-dis and j < len(self.default_map) - dis:
                    ImageDraw.Draw(temp_img).ellipse([(i-dis, j-dis), (i+dis, j+dis)],fill=0, outline=0)
        self.default_map = np.array(temp_img).T
        #return temp_img

    def update_exploration_map(self, scan_data, pose):
        # does not use  the scan_data
        # assume that camera is not abstructed
        # can be improved by taking the scan_data
        angle_of_vision = (math.pi*3)/5
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
        b_x = self.range_of_vision*math.cos(b_angle) + robot_position.x
        b_y = self.range_of_vision*math.sin(b_angle) + robot_position.y
        if robot_orientation + angle_of_vision/2 > math.pi:
            a_angle =  -(math.pi - ((robot_orientation + angle_of_vision/2) - math.pi))
        else:
            a_angle = robot_orientation + angle_of_vision/2
        a_x = self.range_of_vision*math.cos(a_angle) + robot_position.x
        a_y = self.range_of_vision*math.sin(a_angle) + robot_position.y
        #transform the cords to array space
        b_x, b_y = self.translate_coord(b_x, b_y)
        a_x, a_y = self.translate_coord(a_x, a_y)
        c_x, c_y = self.translate_coord(robot_position.x, robot_position.y)
        #set cells to 2 (explored)
        if self.previous_b_x is None:
            ImageDraw.Draw(self.img).polygon([(a_y, a_x), (b_y, b_x), (c_y, c_x)], outline=2, fill=2)
        else:
            ImageDraw.Draw(self.img).polygon([(a_y, a_x), (b_y, b_x), (c_y, c_x)], outline=2, fill=2)
            ImageDraw.Draw(self.img).polygon([(a_y, a_x), (b_y, b_x), (self.previous_b_y, self.previous_b_x), (self.previous_a_y, self.previous_a_x)], outline=2, fill=2)
            ImageDraw.Draw(self.img).polygon([(self.previous_a_y, self.previous_a_x), (a_y, a_x), (self.previous_b_y, self.previous_b_x), (b_y, b_x)], outline=2, fill=2)  
        self.previous_b_x = b_x
        self.previous_b_y = b_y
        self.previous_a_y = a_y
        self.previous_a_x = a_x

    def calc_next_goal(self, pose):
        #where the search will start (the location of robot/pose)
        x, y = self.translate_coord(pose.position.x, pose.position.y)
        self.exploration_map = np.array(self.img).T
        level = 0
        inside_the_wall = False
        go_to = 1 #go to unexplored point
        if self.default_map[y][x] == 0:
            rospy.loginfo("calc_next_goal: inside the wall")
            inside_the_wall = True
            go_to = 2 #go to explored point
        while True:
            x_start = np.max([x - level, 0])
            x_end = np.min([x + level, self.map_width/self.shrink_factor - 1])
            y_start = np.max([y - level, 0])
            y_end = np.min([y + level, self.map_height/self.shrink_factor - 1])
            #for each: if not explored and can be reached
            i = y_start
            j = x_start
            while j < x_end:
                if j < self.map_width/self.shrink_factor and self.exploration_map[i][j] == 1:
                    if (inside_the_wall or self.is_reachable(x, y, j, i)) and self.far_enough(x, y, j, i) and not self.blocked_in_local_costmap(j, i) and not self.blocked_in_global_costmap(j,i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                j = j + 1
            i = y_end
            j = x_start
            while i < y_end:
                if i < self.map_height/self.shrink_factor and self.exploration_map[i][j] == 1:
                    if (inside_the_wall or self.is_reachable(x, y, j, i)) and self.far_enough(x, y, j, i) and not self.blocked_in_local_costmap(j, i) and not self.blocked_in_global_costmap(j,i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                i = i + 1        
            i = y_end
            j = x_end
            while j > x_start:
                if j >= 0 and self.exploration_map[i][j] == 1:
                    if (inside_the_wall or self.is_reachable(x, y, j, i)) and self.far_enough(x, y, j, i) and not self.blocked_in_local_costmap(j, i) and not self.blocked_in_global_costmap(j,i):
                        return (j, i)
                    else:
                        self.exploration_map[i][j] = 0
                j = j - 1
            i = y_start
            j = x_end
            while i > y_start:
                if i >= 0 and self.exploration_map[i][j] == 1:
                    if (inside_the_wall or self.is_reachable(x, y, j, i)) and self.far_enough(x, y, j, i) and not self.blocked_in_local_costmap(j, i) and not self.blocked_in_global_costmap(j,i):
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
        x = int((x - self.map_origin_x)/(self.map_resolution*self.shrink_factor) + 0.5
                + self.map_width/2.0)
        y = int((y - self.map_origin_y)/(self.map_resolution*self.shrink_factor) + 0.5
                + self.map_height/2.0)
        return (x, y)

    def translate_coord_back(self, x, y):
        real_x = (x - self.map_width/2.0)*self.map_resolution*self.shrink_factor + self.map_origin_x
        real_y = (y - self.map_height/2.0)*self.map_resolution*self.shrink_factor + self.map_origin_y
        return (real_x, real_y)

    def is_reachable(self, x, y, x2, y2):
        self.grid.cleanup()
        start = self.grid.node(x, y)
        end = self.grid.node(x2, y2)
        path, runs = self.finder.find_path(start, end, self.grid)
        return len(path) > 0

    def get_goal(self, x, y, x2, y2):
        min_dis = 0.25
        self.grid.cleanup()
        start = self.grid.node(x, y)
        end = self.grid.node(x2, y2)
        path, runs = self.finder.find_path(start, end, self.grid)
        if len(path) == 0:
            return None
        path = path
        point = None
        dis = 0
        while True:
            if len(path) ==0:
                break
            point = path[0]
            r_dis = math.sqrt(math.pow((x2 - point[0])*self.map_resolution*self.shrink_factor, 2) + math.pow((y2 - point[1])*self.map_resolution*self.shrink_factor, 2))
            if r_dis == 0:
                ang = 0
            elif r_dis <= self.range_of_vision and r_dis >= min_dis:
                ang = math.acos(((x2 - point[0])*self.map_resolution*self.shrink_factor)/r_dis)
                if x2 - point[0] <= 0:
                    ang = ang
                    if y2 - point[1] <= 0:
                        ang = ang - 3*math.pi/2
                    else:
                        ang = -(ang-math.pi/2)
                else:
                    if y2 - point[1] <= 0:
                        ang = ang + math.copysign(1, x2 - point[0])*math.pi/2
                    else:
                        ang = ang
                break
            elif r_dis >= min_dis:
                path = path[(len(path) - 1):]
            else:
                point = path[0]
                #ang = 
        return (point[0], point[1], ang)

    def far_enough(self, x, y, x2, y2):
        r_dis = math.sqrt(math.pow((x2 - x)*self.map_resolution*self.shrink_factor, 2) + math.pow((y2 - y)*self.map_resolution*self.shrink_factor, 2))
        return r_dis >= 0.5

    def blocked_in_local_costmap(self, x, y):
        rospy.loginfo("local costmap initiated: {}".format(self.local_costmap is not None))
        if not self.local_costmap_setup:
            return False
        rospy.loginfo("costmap, origin: {}, height/width: {}".format((self.local_costmap_origin_position[0], self.local_costmap_origin_position[1]), (self.local_costmap_height, self.local_costmap_width)))        
        local_x = (x - self.local_costmap_origin_position[0])*self.shrink_factor
        local_y = (y - self.local_costmap_origin_position[1])*self.shrink_factor
        rospy.loginfo("costmap, x: {}, y: {}".format(local_x, local_y, ))
        if local_x >= 0 and local_x < self.local_costmap_width and local_y >= 0 and local_y < self.local_costmap_height:
            if self.local_costmap[local_y*self.local_costmap_width + local_x] == 0:
                return False
            else:
                return True
        else:
            return False
        
    def blocked_in_global_costmap(self, x, y):
        rospy.loginfo("global costmap initiated: {}".format(self.global_costmap is not None))
        if self.global_costmap == None:
            return False
        
        """
        TO TEST IF ITS CONVERTED CORRECTLY TO REAL MAP

        a = (x*self.shrink_factor - self.map_width/2.0)*self.map_resolution + (self.map_real_origin_x + (self.map_width / 2.0) * self.map_resolution)
        b = (y*self.shrink_factor - self.map_height/2.0)*self.map_resolution + (self.map_real_origin_y + (self.map_height / 2.0) * self.map_resolution)
        
        test_odo = deepcopy(self.current_odometry)
        test_odo.pose.pose.position.x = a
        test_odo.pose.pose.position.y = b
        test_odo.pose.pose.orientation.x = 0
        test_odo.pose.pose.orientation.y = 0
        test_odo.pose.pose.orientation.z = 0
        test_odo.pose.pose.orientation.w = 0

        rospy.loginfo("gcm, array x,y: {}".format((x*self.shrink_factor, y*self.shrink_factor)))
               
        self.test_pose_publisher.publish(test_odo)
        """
        
        if self.global_costmap[y*self.shrink_factor*self.map_width + x*self.shrink_factor] == 0:
            return False
        else:
            rospy.loginfo("Blocked in global costmap: True")
            return True

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

if __name__ == '__main__':
    rospy.init_node("explorer")
    d = Display()
    e = Explore(3)
    #self.set_map()
    #e.set_map_and_reduce()
    rospy.spin()
    d.display(np.array(e.img).T)
    print("explored areal: {}".format(d.how_much_explored(e.default_map, e.exploration_map)))
    #d.display_1d(e.local_costmap, e.local_costmap_height, e.local_costmap_width)
    d.display_1d(e.global_costmap, e.map_height, e.map_width, e.current_goal)
    d.display_1d(e.global_costmap_update, e.global_costmap_update_height, e.global_costmap_update_width, None)
