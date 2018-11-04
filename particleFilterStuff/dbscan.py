import math
import random


#Each item in the list points should be a 4-tuple (x, y, qz, qw)
#OR a 4-tuple (x, y, qz, qw, weight)
def estimateLocation(points):
    clusters = dbscan(points, 1.5, 1) #THIS IS WHERE TO CHANGE THE DBSCAN PARAMETERS IF YOU WANT TO
    maxSize = 0
    biggestCluster = []
    for cluster in clusters:
        if weightOfCluster(cluster) > maxSize:
            maxSize = weightOfCluster(cluster)
            biggestCluster = cluster
    x = 0
    y = 0
    qz = 0
    qw = 0
    print(biggestCluster)
    for point in biggestCluster:
        x += (point[0] * weightOfPoint(point))
        y += (point[1] * weightOfPoint(point))
        qz += (point[2] * weightOfPoint(point))
        qw += (point[3] * weightOfPoint(point))
    return (x/maxSize, y/maxSize, qz/maxSize, qw/maxSize)

def weightOfCluster(cluster):
    weight = 0
    for point in cluster:
       weight += weightOfPoint(point)
    return weight

def weightOfPoint(point):
    return 1 #If using weighted particles, comment out this line instead of the next line.
    #return point[4]


#Runs a DBSCAN to find clusters in the points.
#Parameters: points = list of tuples (x, y) representing the points.
            #radius = maximum distance at which points are considered nearby.
            #minPts = minimum number of points adjacent to a point for it to be a core point.
def dbscan(points, radius, minPts):
    corepoints = []
    for point in points:
        if(getNumberNearby(point, points, radius) >= minPts):
            corepoints.append(point)
    clusters = []
    while(len(corepoints) > 0):
        clusters.append(createCluster(points, corepoints, corepoints[0], radius))
    return clusters

def createCluster(points, corePts, starter, radius):
    corePts.remove(starter)
    points.remove(starter)          
    output = []
    output.append(starter)
    for point in points:
        d = distance(point, starter)
        if (point in corePts) and (distance(point, starter) < radius):
            output = output + createCluster(points, corePts, point, radius)
        elif (distance(point, starter) < radius) and not (point in output):
            output.append(point)
    return output
                        
    
def getNumberNearby(point, otherPoints, radius):
    counter = 0
    for pointB in otherPoints:
        if(distance(pointB, point) < radius):
            counter += 1
    return counter - 1 #subtract 1 so as not to count the particle itself lol.


def distance(pointA, pointB):
    return math.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)

def createTestExample(x):
    output = []
    for i in range(0, x):
        output.append((random.randint(0, 10), random.randint(0, 10)))
    print(output)
    return output
        
