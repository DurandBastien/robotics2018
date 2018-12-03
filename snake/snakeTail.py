import math
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
#   Call isLegalMove(Point) before doing so.  Do not make the movement if it returns false.  aStar SHOULD already check this if you use it, but it can't hurt to be sure.
#   Call changePosition(Point) after doing so.
# aStar (Point) works if you give it the point you want to eventually reach.
#   It will return the point you should move to next.
#   Which will always be distanceUnit away in one of the four cardinal directions.

def test():
    teeeest = SnakeTailController()
    teeeest.setStartingPoint(Point(0.0, 0.0))
    teeeest.changePosition(teeeest.aStar(Point(0.05, 0.0)))
    snake = SnakeTailController(Point(0.0, 0.0))
    snake.changePosition(snake.aStar(Point(0.05, 0.0)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.05, 0.05)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.05, 0.1)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.1, 0.1)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.1, 0.05)))
    print("locaation: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.0, 0.05)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.0, 0.05)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.0, 0.05)))
    print("location: " + str(snake.currentPoint))
    snake.changePosition(snake.aStar(Point(0.0, 0.05)))
    print("location: " + str(snake.currentPoint))
    print(snake.tail)
    print(snake.isLegalMove(snake.tail, Point(0.0,0.05)))
    print(snake.isLegalMove(snake.tail, Point(0.05,0.0)))


class SnakeTailController(object):

    mapSet = set([(0.0, 0.0), (0.05, 0.05), (0.05, 0.0), (0.0, 0.05), (0.0, 0.1), (0.05, 0.1), (0.1, 0.0), (0.1, 0.05), (0.1, 0.1)]) #FOR TESTING ONLY!
    #OK, so before this can work with the actual map
    #We'll need some way to convert the map to a grid of points.
    #And create a set that only includes the points that actually exist.
    tailLength = 0.0
    currentPoint = Point(0.0, 0.0, 0.0)
    distanceUnit = 0.05  #Set this to adjust how far a given move will go.
                         #Bigger distanceUnit = better performance but less precise movement.
                         #If distanceUnit is set too large then it is also possible that the robot will crash into things.
                         #I've provisionally set it to 0.05 because that's the resolution of the maps that we have.
    tail = []
        
    def __init__(self, startingPoint = None):
        self.currentPoint = startingPoint
        self.tailLength = self.tailUnitLength()
        self.tail = [startingPoint]

    def setStartingPoint(self, startingPoint): #This should ONLY be called if the controller had to be initialised without a position.
        if (self.currentPoint is None):
            self.currentPoint = startingPoint
            self.tail = [startingPoint]

    def aStar(self, targetPoint):
        if (abs(self.currentPoint.x - targetPoint.x) <= (self.distanceUnit/2)) and (abs(self.currentPoint.y - targetPoint.y) <= (self.distanceUnit/2)):
            return self.currentPoint
        newPoints = self.__adjacentPoints(self.currentPoint, self, 1)
        #print("NP: " + str(newPoints))
        frontier = []
        for newP in newPoints:
            temp = copy.deepcopy(self)
            temp.changePosition(Point(newP[0], newP[1]))
            frontier.append((newP[0], newP[1], self.distanceUnit, Point(newP[0], newP[1]), temp))
        #print("Frontier:")
        #print(frontier)
        #print(list(frontier))
        counter = 0
        while(True):  #Loops until something gets returned.
            counter = counter + 1
            bestScore = 999999999999999999999
            #print(str(counter) + " | " + str(list(frontier)))
            bestP = list(frontier)[0]  #Each point in the frontier is a tuple of the form: (X, Y, cost-to-get-there, first-move-to-get-there, SnakeTailController-with-tail-once-there).
            bestTail = SnakeTailController(Point(0.0, 0.0)) #Ignore this, I just need to include this because otherwise python guesses the type wrong.
            for p in frontier:
                if self.__distance(Point(p[0], p[1]), targetPoint) == 0:
                    #print(str(p[2]))
                    return p[3]
                if p[2] < bestScore:
                    bestScore = p[2]
                    bestPoint = Point(p[0], p[1])
                    bestStartingMove = p[3]
                    bestP = p
                    bestTail = p[4]
            frontier.remove(bestP)
            if self.__distance(bestPoint, targetPoint) < self.distanceUnit * 0.8:
                #print(str(bestP[2]))
                return bestStartingMove
            else:
                bestTail.changePosition(bestPoint)
                newPoints = self.__adjacentPoints(bestPoint, bestP[4], bestP[2] + self.distanceUnit)
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
        #print(returnSet)
        return returnSet
        

    def __reachable(self, point):
        return (point in self.mapSet)

    def tailUnitLength(self):  #Declaring this as a function because for some reason python doesn't actually have constants lol.
        return 0.15 #Edit this number to chage how fast the tail grows.
                    # This number doesn't in principle have anything to do with distanceUnit.
                    # It almost certainly should be a mutiple of distanceUnit tho.
                    #It's just: how much longer do we make the tail each time we eat something?

    def changePosition(self, newPoint):
        realnewPoint = Point(newPoint.x, newPoint.y, newPoint.z)
        self.tail.reverse()
        self.tail.append(realnewPoint)
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
                        cumulativeLength = cumulativeLength + self.__distance(tailPoints[index], tailPoints[index - 1])

        return tailPoints
