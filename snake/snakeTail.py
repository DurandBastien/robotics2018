import math
import random
import copy

class Point(object): #Having to reinvent the wheel here because ROS isn't working on my laptop, will integrate this with the actual ROS stuff next time I'm with the team.
    x = 0.0
    y = 0.0
    z = 0.0
    def __init__(self, xPos, yPos, zPos = 0):
        self.x = xPos
        self.y = yPos
        self.z = zPos
    def __str__(self):
        return ("(" + str(self.x) + "," + str(self.y) + ")")
    def __repr__(self):
        return ("(" + str(self.x) + "," + str(self.y) + ")")
    
# class SnakeTailController is designed to do the basic handling of the snake tail.
# Initialise it with the Point that the robot starts at.
# Call changePosition() whenever the robot picks up a thingy.
# Call isLegalMove(Point) to determine if a move will crash into the tail.
#   isLegalMove(Point) will assume that you intend to move from the previous point given to the robot via changePosition() to the point given as an argument.
# To avoid crossing the tail, EVERY TIME THE ROBOT MOVES:
#   Call isLegalMove(Point) before doing so.  Do not make the movement if it returns false.
#   Call changePosition(Point) after doing so.

def test():
    snake = SnakeTailController(Point(0.0, 0.0))
    snake.changePosition(snake.aStar(Point(0.0, 0.0), Point(0.1, 0.1)))
    snake.changePosition(snake.aStar(snake.currentPoint, Point(0.1, 0.1)))
    print(snake.currentPoint)
    print(snake.tail)


class SnakeTailController(object):

    mapSet = set([(0.0, 0.0), (0.1, 0.1), (0.1, 0.0), (0.0, 0.1)]) #For testing purposes, needs to have the actual map added.
    tailLength = 0.0
    currentPoint = Point(0.0, 0.0, 0.0)
    distanceUnit = 0.1
    tail = []

    def aStar(self, startingPoint, targetPoint):
        if ((-(self.distanceUnit/2) <= (startingPoint.x - targetPoint.x) <= (self.distanceUnit/2)) and ((self.distanceUnit/2) <= (startingPoint.y - targetPoint.y) <= (self.distanceUnit/2))):
            return startingPoint
        newPoints = self.__adjacentPoints(startingPoint, self, 1)
        frontier = set()
        for newP in newPoints:
            frontier.add((newP[0], newP[1], self.__distance(Point(newP[0], newP[1]), targetPoint) + self.distanceUnit, Point(newP[0], newP[1])))
        print("Frontier:")
        print(frontier)
        unfinished = True
        while(unfinished):
            bestScore = 999999999999999999999
            bestP = list(frontier)[0]
            for p in frontier:
                if self.__distance(Point(p[0], p[1]), targetPoint) == 0:
                    return p[3]
                if p[2] < bestScore:
                    bestScore = p[2]
                    bestPoint = Point(p[0], p[1])
                    bestStartingMove = p[3]
                    bestP = p
            frontier.remove(bestP)
            if self.__distance(bestPoint, targetPoint) < self.distanceUnit * 0.8:
                return bestStartingMove
            else:
                newPoints = self.__adjacentPoints(bestPoint, self, 1)
                for newP in newPoints:
                    frontier.add((newP[0], newP[1], bestScore + self.distanceUnit + self.__distance(Point(newP[0], newP[1]), targetPoint), bestStartingMove))
            
            

    def __adjacentPoints(self, p, snakeTail, weight = 0):
        returnSet = set()
        if self.__reachable((p.x + self.distanceUnit, p.y)):
            tailA = copy.deepcopy(snakeTail)
            if tailA.isLegalMove(tailA.tail, Point(p.x + self.distanceUnit, p.y)):
                tailA.changePosition(Point(p.x + self.distanceUnit, p.y))
                returnSet.add((p.x + self.distanceUnit, p.y, weight, tailA))
        if self.__reachable((p.x - self.distanceUnit, p.y)):
            tailB = copy.deepcopy(snakeTail)
            if tailB.isLegalMove(tailB.tail, Point(p.x - self.distanceUnit, p.y)):
                tailB.changePosition(Point(p.x - self.distanceUnit, p.y))
                returnSet.add((p.x - self.distanceUnit, p.y, weight, tailB))
        if self.__reachable((p.x, p.y + self.distanceUnit)):
            tailC = copy.deepcopy(snakeTail)
            if tailC.isLegalMove(tailC.tail, Point(p.x, p.y + self.distanceUnit)):
                tailC.changePosition(Point(p.x, p.y + self.distanceUnit))
                returnSet.add((p.x, p.y + self.distanceUnit, weight, tailC))
        if self.__reachable((p.x, p.y - self.distanceUnit)):
            tailD = copy.deepcopy(snakeTail)
            if tailD.isLegalMove(tailD.tail, Point(p.x, p.y - self.distanceUnit)):
                tailD.changePosition(Point(p.x, p.y - self.distanceUnit))
                returnSet.add((p.x, p.y - self.distanceUnit, weight, tailD))
        return returnSet
        

    def __reachable(self, point):
        return (point in self.mapSet)

    def tailUnitLength(self):  #Declaring this as a function because for some reason python doesn't actually have constants lol.
        return 0.3 #Edit this number to chage how fast the tail grows.
    
    def __init__(self, startingPoint):
        self.currentPoint = startingPoint
        self.tailLength = self.tailUnitLength()
        self.tail = [startingPoint]

    def changePosition(self, newPoint):
        realnewPoint = Point(newPoint.x, newPoint.y, newPoint.z)
        self.tail.reverse()  #Need to add a slight bit of random noise to each new point to avoid issues with moving directly NORTH.
        self.tail.append(realnewPoint) #Quite a hacky solution but hey.
        self.tail.reverse()
        self.currentPoint = realnewPoint
        self.tail = self.__removeRedundantPoints(self.tail, self.tailLength)
        #print(self.tail)

    def increaseTailLength(self):
        self.tailLength = self.tailLength + self.tailUnitLength()

    def __distance(self, pointA, pointB):
        xOffset = pointA.x - pointB.x
        yOffset = pointA.y - pointB.y
        return math.sqrt((xOffset * xOffset) + (yOffset * yOffset))

#    def intersect(self, pointA1, pointA2, pointB1, pointB2): #Does the line segment from A1 to A2 intersect the line segment from B1 to B2?
        #y = ((d - b)/(c - a))x + b - (a(d - b)/(c - a))
        #x = (c2 - c1)/(m1 - m2)
        #y = m1(c2 - c1)/(m1 - m2) + c1
#        ma = (pointA2.y - pointA1.y)/(pointA2.x - pointA1.x)
        #print(ma)
 #       ca = pointA1.y - (pointA1.x * ma)
        #print(ca)
  #      mb = (pointB2.y - pointB1.y)/(pointB2.x - pointB1.x)
        #print(mb)
   #     cb = pointB1.y - (pointB1.x * mb)
       # print(cb)
    #    X = (cb - ca)/(ma - mb)
        #print(X)
     #   Y = (ma * X) + ca #X and Y of the intersection if the lines were infinitely long, now just need to find whether it lies on the segments.
        #print(Y)
      #  return (self.__between(X, pointA1.x, pointA2.x) and self.__between(Y, pointA1.y, pointA2.y) and self.__between(X, pointB1.x, pointB2.x) and self.__between(Y, pointB1.y, pointB2.y))


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
            for x in range(len(self.tail) - 2):
                if(self.__intersect(self.currentPoint, target, self.tail[x], self.tail[x+1])):
                    return False
                cumulativeLength = cumulativeLength + self.__distance(self.tail[x], self.tail[x+1])
            lastPoint = self.tail[len(self.tail) - 1]
            secondLast = self.tail[len(self.tail) - 2]
            fraction = (self.tailLength - cumulativeLength)/self.__distance(lastPoint, secondLast)
            deltaX = lastPoint.x - secondLast.x #All of this faffing around is needed because the position can't be updated every instant.
            deltaY = lastPoint.y - secondLast.y #So it may not be the case that the tail stretches the entire distance from the last point
            realLastPoint = Point(secondLast.x + (deltaX * fraction), secondLast.y + (deltaY * fraction), 0.0) #to the second-last point.
            return not self.__intersect(self.currentPoint, target, secondLast, realLastPoint)
           # m = (lastPoint.y - secondLast.y)/(lastPoint.x - secondLast.x)
           # realLastPoint = Point(secondLast.x + (m * (tailLength - cumulativeLength)), secondLast.x + (m * (tailLength - cumulativeLength))
        return True


    def __removeRedundantPoints(self, tailPoints, length): #Remove points that aren't part of the tail any more.
        cumulativeLength = 0.0
        finished = False
        for (index, value) in enumerate(tailPoints):
            if(finished):
                del(tailPoints[index])
            else:
                if(cumulativeLength > length):
                    finished = true
                else:
                    if (index != 0):
                        cumulativeLength = cumulativeLength + self.__distance(tailPoints[index], tailPoints[index - 1])

        return tailPoints
