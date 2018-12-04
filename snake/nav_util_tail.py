#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
import math
import sys
import copy

class nav_util(object):

        unoccupiedThreshold = 99 # A ROS Occupancy grid has a "probability of being occupied" for each point.
                                 # This is represented as a percentage.
                                 # So 100 is "we know for sure that this point is occupied".
                                 # unoccupiedThreshold is "how low do we need the % to be to consider the point empty for the purpose of pathfinding".
                                 # So 20 means that if a point has a 79% chance of being empty, we consider it full; if it has a 81% chance of being empty, we consider it empty.
        
        def __init__(self):
                self.estimatedpose = PoseWithCovarianceStamped()
                self.goalIsCancelled = False
                self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                rospy.loginfo("nav_util waiting for action server")
                self.move_base_client.wait_for_server()
                rospy.loginfo("server found")
                self.tailCont = SnakeTailController()
                rospy.loginfo("nav_util waiting for a map...")
                self._pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                                                  self._pose_callback,
                                                  queue_size=1)

                self.occupancy_map = None

                # try:
                #         occupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
                #         self.tailCont.update_map(self.prepareMap(occupancy_map))
                        
                # except:
                #         rospy.logerr("Problem getting a map. Check that you have a map_server running: rosrun map_server map_server <mapname> " )
                #         sys.exit(1)

        def init_map(self, map_):
            self.occupancy_map = map_
            self.tailCont.updateMap(self.prepareMap(self.occupancy_map))

        def _pose_callback(self, pose):
                self.tailCont.changePosition(pose.pose.pose.position)
                self.estimatedpose = pose

        def where_am_i(self):
                return self.estimatedpose

        def go_to_pose(self, pose):  #This will fail if we haven't ever given the nav_util a pose for the robot to be at.

                goal = MoveBaseGoal()
                #goal.target_pose.header.frame_id = 'map'
                #goal.target_pose.header.stamp = rospy.Time.now()
                #goal.target_pose.pose = pose.pose.pose
                #self.move_base_client.send_goal(goal)

                #res = self.move_base_client.wait_for_result()
                #rospy.loginfo("result recieved")
                #rospy.loginfo(res)

                targetPoint = pose.pose.pose.position
                rospy.loginfo("targetPoint obtained")
                currentTarget = self.tailCont.aStar(targetPoint)
                rospy.loginfo("currentTarget: " + str(currentTarget))
                while(self.tailCont.roundToNearest(targetPoint) != self.tailCont.currentPoint):
                        #goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = 'map'
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose = Pose(currentTarget, pose.pose.pose.orientation)
                        rospy.loginfo("targetpose assembled")
                        self.move_base_client.send_goal(goal)
                        rospy.loginfo("goal sent")
                        rospy.loginfo(str(goal))
                        res = self.move_base_client.wait_for_result()
                        rospy.loginfo("result recieved")
                        rospy.loginfo(res)
                        self.tailCont.changePosition(self.tailCont.aStar(targetPoint))
                        currentTarget = self.tailCont.aStar(targetPoint)
                        rospy.loginfo("currentTarget: " + str(currentTarget))
                        if(self.goalIsCancelled):
                                self.goalIsCancelled = False
                                return 1

        def cancel_goal(self):
                self.move_base_client.cancel_goal()
                self.goalIsCancelled = True

        def get_goalStatus(self):
                return self.move_base_client.get_state()

        def prepareMap(self, occupancy_map):
                map_info = occupancy_map.info
                map_width = occupancy_map.info.width
                map_height = occupancy_map.info.height
                map_resolution = occupancy_map.info.resolution # in m per pixel
                map_origin_x = ( occupancy_map.info.origin.position.x + (map_width / 2.0) * map_resolution )
                map_origin_y = ( occupancy_map.info.origin.position.y + (map_height / 2.0) * map_resolution )
                outputSet = set()
                for counter, cell_prob in enumerate(occupancy_map.data):
                        if(cell_prob >= 0 and cell_prob < 0.196):
                                x_pix = counter % map_width
                                y_pix = (counter - x_pix) / map_height
                                x = x_pix*map_info.resolution + map_info.origin.position.x
                                y = y_pix*(map_info.resolution + 0.004) + map_info.origin.position.y
                                # x = (math.floor((x_pix - map_origin_x) / map_resolution + 0.5) + map_width / 2);
                                # y = (math.floor((y_pix - map_origin_y) / map_resolution + 0.5) + map_height / 2);
                                outputSet.add((x, y))

                # outputSet = set()
                # origin = metD.origin.position
                # for y in range(metD.height/5):
                #         for x in range(metD.width/5):
                #                 if (occGrid.data[(y * metD.width * 5) + (x * 5)] < self.unoccupiedThreshold):
                #                         try:
                #                                 #if (occGrid.data[(y*5 * metD.width) + (x-1)] < self.unoccupiedThreshold) and (occGrid.data[((y+1) * metD.width) + x] < self.unoccupiedThreshold) and (occGrid.data[((y-1) * metD.width) + x] < self.unoccupiedThreshold) and (occGrid.data[(y * metD.width) + (x+1)] < self.unoccupiedThreshold) and (occGrid.data[((y-1) * metD.width) + (x+1)] < self.unoccupiedThreshold) and (occGrid.data[((y+1) * metD.width) + (x-1)] < self.unoccupiedThreshold) and (occGrid.data[((y-1) * metD.width) + (x-1)] < self.unoccupiedThreshold) and (occGrid.data[((y+1) * metD.width) + (x+1)] < self.unoccupiedThreshold):
                #                                         #The disgusting above statement means that a point is only empty if both it AND the eight   adjacent points are empty.
                #                                         #Hopefully this will prevent the robot from crashing into things.
                                                
                #                                 outputSet.add(((x * metD.resolution * 5 + origin.x), (y * 5 * metD.resolution + origin.y)))
                #                         except:
                #                                 dummy = 0
                # rospy.loginfo(str(outputSet))
                rospy.loginfo(outputSet)
                return outputSet



class SnakeTailController(object):
        
        mapSet = set([(0.0, 0.0), (0.05, 0.05), (0.05, 0.0), (0.0, 0.05), (0.0, 0.1), (0.05, 0.1), (0.1, 0.0), (0.1, 0.05), (0.1, 0.1)]) #FOR TESTING ONLY!
    #OK, so before this can work with the actual map
    #We'll need some way to convert the map to a grid of points.
    #And create a set that only includes the points that actually exist.
        tailLength = 0.0
        currentPoint = Point(0.0, 0.0, 0.0)
        distanceUnit = 0.25  #Set this to adjust how far a given move will go.
                         #Bigger distanceUnit = better performance but less precise movement.
                         #If distanceUnit is set too large then it is also possible that the robot will crash into things.
                         #I've provisionally set it to 0.05 because that's the resolution of the maps that we have.
        tail = []
        
        def __init__(self, startingPoint = None):
                self.currentPoint = startingPoint
                self.tailLength = self.tailUnitLength()
                self.tail = [startingPoint]

        def aStar(self, tempTargetPoint):
                targetPoint = self.roundToNearest(tempTargetPoint)
                if (abs(self.currentPoint.x - targetPoint.x) <= (self.distanceUnit/2)) and (abs(self.currentPoint.y - targetPoint.y) <= (self.distanceUnit/2)):
                        return self.currentPoint
                newPoints = self.__adjacentPointsForceReturn(self.currentPoint, self, 1)
                #print("NP: " + str(newPoints))
                frontier = []
                for newP in newPoints:
                        temp = copy.deepcopy(self)
                        temp.changePosition(Point(newP[0], newP[1], 0.0))
                        frontier.append((newP[0], newP[1], self.distanceUnit, Point(newP[0], newP[1], 0.0), temp))
                #print("Frontier:")
                #print(frontier)
                #print(list(frontier))
                counter = 0
                while(True):  #Loops until something gets returned.
                        counter = counter + 1
                        bestScore = 999999999999999999999
                        rospy.loginfo(str(counter) + " | " + str(list(frontier)))
                        try:
                                bestP = list(frontier)[0]  #Each point in the frontier is a tuple of the form: (X, Y, cost-to-get-there, first-move-to-get-there, SnakeTailController-with-tail-once-there).
                        except:
                                rospy.loginfo("pathfinding error: no valid route to target.")
                                return self.currentPoint
                        bestTail = SnakeTailController(Point(0.0, 0.0, 0.0)) #Ignore this, I just need to include this because otherwise python guesses the type wrong.
                        for p in frontier:
                                if self.__distance(Point(p[0], p[1], 0.0), targetPoint) == 0:
                                #print(str(p[2]))
                                        return p[3]
                                if p[2] < bestScore:
                                        bestScore = p[2]
                                        bestPoint = Point(p[0], p[1], 0.0)
                                        bestStartingMove = p[3]
                                        bestP = p
                                        bestTail = p[4]
                        frontier.remove(bestP)
                        if self.__distance(bestPoint, targetPoint) < self.distanceUnit:
                                #print(str(bestP[2]))
                                return bestStartingMove
                        else:
                                bestTail.changePosition(bestPoint)
                                newPoints = bestTail.__adjacentPoints(bestPoint, bestTail, bestP[2] + self.distanceUnit)
                                for newP in newPoints:
                                        frontier.append((newP[0], newP[1], bestP[2] + self.distanceUnit, bestStartingMove, bestTail))

    # Use this to update the map when a new dynamic obstacle is detected.
        def updateMap(self, newMap):
                self.mapSet = newMap

        def __adjacentPoints(self, p, snakeTail, weight = 0):
                #print(p)
                returnSet = set()
                if self.__reachable((p.x + self.distanceUnit, p.y)):
                        tailA = copy.deepcopy(snakeTail)
                        if tailA.isLegalMove(tailA.tail, Point(p.x + self.distanceUnit, p.y, 0.0)):
                                tailA.changePosition(Point(p.x + self.distanceUnit, p.y, 0.0))
                                returnSet.add((p.x + self.distanceUnit, p.y, weight, tailA))
                if self.__reachable((p.x - self.distanceUnit, p.y)):
                        tailB = copy.deepcopy(snakeTail)
                        if tailB.isLegalMove(tailB.tail, Point(p.x - self.distanceUnit, p.y, 0.0)):
                                tailB.changePosition(Point(p.x - self.distanceUnit, p.y, 0.0))
                                returnSet.add((p.x - self.distanceUnit, p.y, weight, tailB))
                if self.__reachable((p.x, p.y + self.distanceUnit)):
                        tailC = copy.deepcopy(snakeTail)
                        if tailC.isLegalMove(tailC.tail, Point(p.x, p.y + self.distanceUnit, 0.0)):
                                tailC.changePosition(Point(p.x, p.y + self.distanceUnit, 0.0))
                                returnSet.add((p.x, p.y + self.distanceUnit, weight, tailC))
                if self.__reachable((p.x, p.y - self.distanceUnit)):
                        tailD = copy.deepcopy(snakeTail)
                        if tailD.isLegalMove(tailD.tail, Point(p.x, p.y - self.distanceUnit, 0.0)):
                                tailD.changePosition(Point(p.x, p.y - self.distanceUnit, 0.0))
                                returnSet.add((p.x, p.y - self.distanceUnit, weight, tailD))
                #print(returnSet)
                return returnSet

        def __adjacentPointsForceReturn(self, p, snakeTail, weight = 0):
            #print(p)
                returnSet = set()
                if self.__reachable((p.x + self.distanceUnit, p.y)):
                        tailA = copy.deepcopy(snakeTail)
                        if tailA.isLegalMove(tailA.tail, Point(p.x + self.distanceUnit, p.y, 0.0)):
                                tailA.changePosition(Point(p.x + self.distanceUnit, p.y, 0.0))
                                returnSet.add((p.x + self.distanceUnit, p.y, weight, tailA))
                if self.__reachable((p.x - self.distanceUnit, p.y)):
                        tailB = copy.deepcopy(snakeTail)
                        if tailB.isLegalMove(tailB.tail, Point(p.x - self.distanceUnit, p.y, 0.0)):
                                tailB.changePosition(Point(p.x - self.distanceUnit, p.y, 0.0))
                                returnSet.add((p.x - self.distanceUnit, p.y, weight, tailB))
                if self.__reachable((p.x, p.y + self.distanceUnit)):
                        tailC = copy.deepcopy(snakeTail)
                        if tailC.isLegalMove(tailC.tail, Point(p.x, p.y + self.distanceUnit, 0.0)):
                                tailC.changePosition(Point(p.x, p.y + self.distanceUnit, 0.0))
                                returnSet.add((p.x, p.y + self.distanceUnit, weight, tailC))
                if self.__reachable((p.x, p.y - self.distanceUnit)):
                        tailD = copy.deepcopy(snakeTail)
                        if tailD.isLegalMove(tailD.tail, Point(p.x, p.y - self.distanceUnit, 0.0)):
                                tailD.changePosition(Point(p.x, p.y - self.distanceUnit, 0.0))
                                returnSet.add((p.x, p.y - self.distanceUnit, weight, tailD))
                #print(returnSet)
                if(len(returnSet) == 0):
                    return self.__adjacentPoints(Point(p.x + self.distanceUnit, p.y, 0.0), snakeTail, weight = 0).union(self.__adjacentPoints(Point(p.x - self.distanceUnit, p.y, 0.0), snakeTail, weight = 0), self.__adjacentPoints(Point(p.x, p.y + self.distanceUnit, 0.0), snakeTail, weight = 0), self.__adjacentPoints(Point(p.x, p.y - self.distanceUnit, 0.0), snakeTail, weight = 0))
                return returnSet
        

        def __reachable(self, point):
                return (point in self.mapSet)

        def tailUnitLength(self):  #Declaring this as a function because for some reason python doesn't actually have constants lol.
                return 0.25 #Edit this number to chage how fast the tail grows.
                                # This number doesn't in principle have anything to do with distanceUnit.
                                # It almost certainly should be a mutiple of distanceUnit tho.
                                #It's just: how much longer do we make the tail each time we eat something?

        def changePosition(self, newPoint):
                if (self.currentPoint is None): # If we've never recieved a position before
                        self.currentPoint = self.roundToNearest(newPoint) # then we set the starting point
                        self.tail = [self.roundToNearest(newPoint)]
                        return
                realnewPoint = self.roundToNearest(Point(newPoint.x, newPoint.y, newPoint.z))
                self.tail.reverse()
                self.tail.append(realnewPoint)
                self.tail.reverse()
                self.currentPoint = realnewPoint
                self.tail = self.__removeRedundantPoints(self.tail, self.tailLength)
                #print(self.tail)

        def roundToNearest(self, unroundedPoint):
                if self.__reachable(unroundedPoint):
                        return unroundedPoint
                else:
                        shortestDistance = float("inf")
                        for p in self.mapSet:
                                dist = self.__distance(Point(p[0], p[1], 0.0), unroundedPoint)
                                if dist < shortestDistance:
                                        shortestDistance = dist
                                        closestPoint = p
                        return Point(closestPoint[0], closestPoint[1], 0.0)
                        

        def increaseTailLength(self):
                self.tailLength = self.tailLength + self.tailUnitLength()

        def __distance(self, pointA, pointB):
                xOffset = pointA.x - pointB.x
                yOffset = pointA.y - pointB.y
                return math.sqrt((xOffset * xOffset) + (yOffset * yOffset))

        def __intersect(self, pointA1, pointA2, pointB1, pointB2): #Does the line segment from A1 to A2 intersect the line segment from B1 to B2?
                #y = ((d - b)/(c - a))x + b - (a(d - b)/(c - a))
                #x = (c2 - c1)/(m1 - m2)
                #y = m1(c2 - c1)/(m1 - m2) + c1
                if (pointA2.x == pointA1.x):
                        ma = (pointA2.y - pointA1.y)/0.000000001 #Avoiding division by zero in a pretty terrible way.
                else:
                        ma = (pointA2.y - pointA1.y)/(pointA2.x - pointA1.x)
                ca = pointA1.y - (pointA1.x * ma)
                if (pointB2.x == pointB1.x):
                        mb = (pointB2.y - pointB1.y)/0.000000001
                else:
                        mb = (pointB2.y - pointB1.y)/(pointB2.x - pointB1.x)
                cb = pointB1.y - (pointB1.x * mb)
                if (ma - mb) == 0:
                        return False
                X = (cb - ca)/(ma - mb) 
                Y = (ma * X) + ca #X and Y of the intersection if the lines were infinitely long, now just need to find whether it lies on the segments.
                return (self.__between(X, pointA1.x, pointA2.x) and self.__between(Y, pointA1.y, pointA2.y) and self.__between(X, pointB1.x, pointB2.x) and self.__between(Y, pointB1.y, pointB2.y))

        def __between(self, x, a, b): #Is x between a and b?  Just makes the bottom line of intersect look slightly less godawful.
                return ((a < x < b) or (b < x < a) or (b == x == a))
        

        def isLegalMove(self, tailPoints, target):
                if(len(self.tail) >= 2):
                        cumulativeLength = 0.0
                        for x in range(0, len(self.tail) - 1):
                        #print("Debug: " + str(self.tail[x].x) + " " + str(self.tail[x].y))
                                if(self.__intersect(self.currentPoint, target, self.tail[x], self.tail[x+1])):
                                        return False
                                cumulativeLength = cumulativeLength + self.__distance(self.tail[x], self.tail[x+1])
                                if(target.x == self.tail[x].x and target.y == self.tail[x].y):
                                        return False
                                if (target.x == self.tail[len(self.tail) - 1].x and target.y == self.tail[len(self.tail) - 1].y):
                                        return False
                        return True
                return True

        def __removeRedundantPoints(self, tailPoints, length): #Remove points that aren't part of the tail any more.
                cumulativeLength = 0.0
                finished = False
                for (index, value) in enumerate(tailPoints):
                        if(finished):
                                del(tailPoints[index])
                        else:
                                if(cumulativeLength > length):
                                        finished = True
                                else:
                                        if (index != 0):
                                                cumulativeLength = cumulativeLength + self.__distance(Point(tailPoints[index].x, tailPoints[index].y, 0.0), Point(tailPoints[index - 1].x, tailPoints[index - 1].y, 0.0))

                return tailPoints

