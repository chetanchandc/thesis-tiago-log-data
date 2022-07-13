#!/usr/bin/env python

## 
# @file closest_obstacle_detection.py 
# @author Davide Piccinini <dav.piccinini\@gmail.com>
# @date 9 February 2022
# @brief This script publishes an updated scan message corresponding to the 
# 		 sector containing the closest object to the robot.

import rospy
import math
from sensor_msgs.msg import LaserScan
from log_data.msg import ClosestObstacleInfo

## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Updated scan publisher
scanPub = None

## Closest obstacle info publisher
obstacleInfoPub = None

## Updated scan message
updatedScan = LaserScan()

## Closest obstacle info message
objectInfo = ClosestObstacleInfo()

## Number of sectors
numPedestrians = 5

## Possible pedestrians
ids = ["1", "2", "3", "4", "5"]

## Setup variable
doSetup = True

## Setup variable
angleIncrementDeg = 0
## Setup variable
scanRays = 0
## Setup variable
pedestrianRays = 0

# Closest object variable
closestObstacleDistance = 0
# Closest object variable
closestPedestrianId = ""
# Closest object variable
idIndex = 0


##
# @brief The callback function for the @c '/scan' topic.
# 
# The function retrieves the scan reading from the hokuyo laser, dividing the
# detected values in 5 sectors corresponding to left, front-left, front,
# front-right, and right directions: then, all ranges of the updated scan
# are set to infinity except those of the sector which contains the closest
# obstacle. This updated scan message and the info about the closest
# obstacle (distance and qualitative position) are then published on their
# respective topics.
#
# @param scan The LaserScan message.
def detect_closest_pedestrian(scan):
    global radToDegRatio
    global updatedScan, objectInfo
    global numPedestrians
    global ids
    global doSetup
    global angleIncrementDeg, scanRays, pedestrianRays
    global closestObstacleDistance, closestPedestrianId, idIndex

    # The first time a scan message is received obtain the necessary parameters
    if doSetup:
        angleIncrementDeg = scan.angle_increment * radToDegRatio
        scanRays = int(math.ceil(((scan.angle_max - scan.angle_min) * radToDegRatio) / angleIncrementDeg))
    	
    	# Check if the total scan angle can be divided by the number of sectors cleanly
        remainder = scanRays % numPedestrians
        if remainder == 0:
            pedestrianRays = scanRays / numPedestrians
        else:
            pedestrianRays = (scanRays - remainder) / numPedestrians
        
    	# Initialize the setup variables only once
        doSetup = False
    
    # Initialize the updated scan message and set all ranges to infinity
    updatedScan = scan
    ranges = list(updatedScan.ranges)
    for item, value in enumerate(ranges):
        ranges[item] = float("inf")
    
    # Get the distance to the closest obstacle and its position
    closestObstacleDistance = min(scan.ranges)
    idIndex = scan.ranges.index(closestObstacleDistance)
    
    # If there're no detected obstacles then send an appropriate message
    if closestObstacleDistance == float("inf"):
        # Publish the updated scan message
        scanPub.publish(updatedScan)
    
        # Initialize the closest object info message and publish it
        objectInfo.distance = closestObstacleDistance
        objectInfo.id = "none"
        obstacleInfoPub.publish(objectInfo)
    
    # Keep only the ranges of the sector that contains the closest obstacle
    for i in range(numPedestrians):
        if idIndex <= (((i + 1) * pedestrianRays) - 1):
            closestPedestrianId = ids[i]
            startIndex = (i * pedestrianRays)
            endIndex = ((i + 1) * pedestrianRays)
            ranges[startIndex:endIndex] = scan.ranges[startIndex:endIndex]
            updatedScan.ranges = ranges
            break

	# Publish the updated scan message
    scanPub.publish(updatedScan)
    
    # Initialize the closest object info message and publish it
    objectInfo.distance = closestObstacleDistance
    objectInfo.id = closestPedestrianId
    obstacleInfoPub.publish(objectInfo)

##
# Main function
if __name__ == '__main__':

	try:
		# Initialize the node
		rospy.init_node('closest_obstacle_detection')
    	
    	# Subscribe to the topic
		rospy.Subscriber('/scan', LaserScan, detect_closest_pedestrian)
    	
    	# Define the updated scan publisher
		scanPub = rospy.Publisher('closest_obstacle_scan', LaserScan, queue_size=1)
		
		# Define the closest obstacle info publisher
		obstacleInfoPub = rospy.Publisher('closest_obstacle_info', ClosestObstacleInfo, queue_size=1)
    
		while not rospy.is_shutdown():
            # Keep from exiting until the node is stopped
			rospy.spin()
        
	except rospy.ROSInterruptException:
		pass
