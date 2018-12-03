from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

unoccupiedThreshold = 20 # A ROS Occupancy grid has a "probability of being occupied" for each point.
                         # This is represented as a percentage.
                         # So 100 is "we know for sure that this point is occupied".
                         # unoccupiedThreshold is "how low do we need the % to be to consider the point empty for the purpose of pathfinding".

def prepareMap(occGrid):
    metD = occGrid.info
    outputSet = set()
    origin = metD.origin.position
    for y in range(metD.height):
        for x in range(metD.width):
            if (occGrid.data[(y * metD.width) + x] < unoccupiedThreshold):
                if (occGrid.data[(y * metD.width) + (x-1)] < unoccupiedThreshold) and (occGrid.data[((y+1) * metD.width) + x] < unoccupiedThreshold) and (occGrid.data[((y-1) * metD.width) + x] < unoccupiedThreshold) and (occGrid.data[(y * metD.width) + (x+1)] < unoccupiedThreshold): and (occGrid.data[((y-1) * metD.width) + (x+1)] < unoccupiedThreshold) and (occGrid.data[((y+1) * metD.width) + (x-1)] < unoccupiedThreshold) and (occGrid.data[((y-1) * metD.width) + (x-1)] < unoccupiedThreshold) and (occGrid.data[((y+1) * metD.width) + (x+1)] < unoccupiedThreshold):
                #The disgusting above statement means that a point is only empty if both it AND the eight adjacent points are empty.
                #Hopefully this will prevent the robot from crashing into things.
                    outputSet.add(((x * metD.resolution + origin.x), (y * metD.resolution + origin.y)))
    return outputSet
