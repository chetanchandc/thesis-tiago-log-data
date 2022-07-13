#!/usr/bin/env python
### this script provides all the log data for the robot and pedestrians positions(x,y,theta) and robot velocites and scan data.
	    ##### @todo convert the pedestrians angle from radians to degrees
	    ##### add a LOOP to optimize code for not defining every pedestrian.
import rospy
import math
import datetime
import time
import os
import threading
import csv
import tf_conversions
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import AgentStates
from colorama import Fore, Back, Style

from sensor_msgs.msg import LaserScan

## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Threading event
odometryReceived = threading.Event()
infoReceived = threading.Event()
scanReceived = threading.Event()
distReceived = threading.Event()

goalTime = None

## Odometry variable
robotPosition = None
robotOrientationRad = None
robotOrientationDeg = None

robotLinVel = None
robotAngVel = None

## Pedestrians Variable
pedPosition1 = None
pedPosition2 = None
pedPosition3 = None
pedPosition4 = None
pedPosition5 = None

pedOrientation1 = None
pedOrientation2 = None
pedOrientation3 = None
pedOrientation4 = None
pedOrientation5 = None

distance1 = None
distance2 = None
distance3 = None
distance4 = None
distance5 = None

objectScan = None

distance = None

## Time variable
startingTime = None
## Time variable
currentTime = None


def odometry_callback(odometryMsg):
    global goalTime, robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
	
    if not odometryReceived.is_set():

	goalTime = odometryMsg.header.stamp.secs
	
        robotPosition = odometryMsg.pose.pose.position
        
# Convert the attitude to degrees
        robotOrientationRad = tf_conversions.transformations.euler_from_quaternion([odometryMsg.pose.pose.orientation.x, odometryMsg.pose.pose.orientation.y, odometryMsg.pose.pose.orientation.z, odometryMsg.pose.pose.orientation.w])
        robotOrientationDeg = [value * radToDegRatio for value in robotOrientationRad]

        robotLinVel = odometryMsg.twist.twist.linear

        robotAngVel = odometryMsg.twist.twist.angular
        robotAngVel.x *= radToDegRatio
        robotAngVel.y *= radToDegRatio
        robotAngVel.z *= radToDegRatio

        # Set the flag to true
        odometryReceived.set()


def pedestrians_info_callback(pedMsg):
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5
    
    # Process the message only when the previous one has been logged
    if not infoReceived.is_set():
	    ##### @todo convert the pedestrians angle from radians to degrees
	    ##### add a LOOP to optimize code for not defining every pedestrian.
           pedPosition1 = pedMsg.agent_states[0].pose.position
	   pedOrientation1 = pedMsg.agent_states[0].pose.orientation
           pedPosition2 = pedMsg.agent_states[1].pose.position
	   pedOrientation2 = pedMsg.agent_states[1].pose.orientation
	   pedPosition3 = pedMsg.agent_states[2].pose.position
	   pedOrientation3 = pedMsg.agent_states[2].pose.orientation
	   pedPosition4 = pedMsg.agent_states[3].pose.position
	   pedOrientation4 = pedMsg.agent_states[3].pose.orientation
	   pedPosition5 = pedMsg.agent_states[4].pose.position
           pedOrientation5 = pedMsg.agent_states[4].pose.orientation
	
        # Set the flag to true
           infoReceived.set()


def callback(msg):
    global objectScan
    
    #print len(msg.ranges) = 666
    if not scanReceived.is_set():
	
 	objectScan = msg.ranges[360]

        # Set the flag to true
        scanReceived.set()


def rob_ped_distance(math):
    global robotPosition
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global distance1, distance2, distance3, distance4, distance5

    if not distReceived.is_set():
	distance1 = math.sqrt( ((int(robotPosition.x)-int(pedPosition1.x))**2)+((int(robotPosition.y)-int(pedPosition1.y))**2) )
        distance2 = math.sqrt( ((int(robotPosition.x)-int(pedPosition2.x))**2)+((int(robotPosition.y)-int(pedPosition2.y))**2) )
	distance3 = math.sqrt( ((int(robotPosition.x)-int(pedPosition3.x))**2)+((int(robotPosition.y)-int(pedPosition3.y))**2) )
	distance4 = math.sqrt( ((int(robotPosition.x)-int(pedPosition4.x))**2)+((int(robotPosition.y)-int(pedPosition4.y))**2) )
	distance5 = math.sqrt( ((int(robotPosition.x)-int(pedPosition5.x))**2)+((int(robotPosition.y)-int(pedPosition5.y))**2) )
	
	distReceived.set()


def print_feedback():
    global goalTime, robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5
    global startingTime, currentTime
    
    # Threshold in meters after which the terminal warns the pilot
    #distanceThreshold = 1.5
    
    # Compute the time elapsed from the start of the experiment
    currentTime = int(time.time())
    timePeriod = currentTime - startingTime
    minutes = int(math.floor(timePeriod / 60))
    seconds = timePeriod % 60
    
    # Print on the terminal the information that is useful to the pilot
    print(Style.BRIGHT + Fore.YELLOW + "Current elapsed time from the start of the experiment:" + Style.RESET_ALL)
    print(Style.BRIGHT + Fore.YELLOW + "* " + str(minutes) + " minutes and " + str(seconds) + " seconds." + Style.RESET_ALL)
    print
    print("The current position (in meters) of the robot is:")
    print(Style.BRIGHT + "* x: " + str(round(robotPosition.x, 2)) + "\n* y: " + str(round(robotPosition.y, 2)) + Style.RESET_ALL)
    print
    print("The current yaw (in degrees) of the robot is:")
    print(Style.BRIGHT + "* Yaw: " + str(round(robotOrientationDeg[2], 2)) + Style.RESET_ALL)
    print
    print("The current linear velocity (in m/s) of the robot is:")
    print(Style.BRIGHT + "* Velocity in x: " + str(round(robotLinVel.x, 2)) + Style.RESET_ALL)
    print
    print("The current yaw angular velocity (in deg/s) of the robot is:")
    print(Style.BRIGHT + "* Rotation about the Z axis: " + str(round(robotAngVel.z, 2)) + Style.RESET_ALL)
    print


def log_feedback():
    global goalTime, robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5
    global objectScan
    global distance1, distance2, distance3, distance4, distance5

    currentTimestamp = [int(time.time())]

    logGoalTime = [goalTime]
    logRobotPose = [robotPosition.x, robotPosition.y, str(robotOrientationDeg[2])]
    logRobotVelocities = [robotLinVel.x, robotAngVel.z]
    logPed1 = [pedPosition1.x, pedPosition1.y, pedOrientation1.z]
    logPed2 = [pedPosition2.x, pedPosition2.y, pedOrientation2.z]
    logPed3 = [pedPosition3.x, pedPosition3.y, pedOrientation3.z]   
    logPed4 = [pedPosition4.x, pedPosition4.y, pedOrientation4.z]
    logPed5 = [pedPosition5.x, pedPosition5.y, pedOrientation5.z]
    logScan = [objectScan]
    logDist1 = [distance1]
    logDist2 = [distance2]
    logDist3 = [distance3]
    logDist4 = [distance4]
    logDist5 = [distance5]

    data = currentTimestamp + logGoalTime + logRobotPose + logRobotVelocities + logPed1 + logPed2 + logPed3 + logPed4 + logPed5 + logScan + logDist1, logDist2, logDist3, logDist4, logDist5
    outputCsv.writerow(data)
    csvFile.flush()


def feedback():

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Clear the terminal
        os.system("clear")
    
        # Wait until all messages have been received
        odometryReceived.wait()
        infoReceived.wait()
        scanReceived.wait()
        distReceived.wait()

        # Log the feedback on a csv file
        log_feedback()
        
        # Print the feedback on the terminal
        print_feedback()
        
        # Clear the flags
        odometryReceived.clear()
        infoReceived.clear()
        scanReceived.clear()
        distReceived.clear()

        rate.sleep()



if __name__ == '__main__':

	try:
	    
 	    rospy.init_node('feedback_new')

            rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)

            rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, pedestrians_info_callback)      
  
            rospy.Subscriber('/scan', LaserScan, callback)

		# Open the log file
	    csv_path = rospy.get_param("output_csv_path")
    	    currentDate = datetime.datetime.now()
	    path = csv_path + str(currentDate.year) + "-" + str(currentDate.month) + "-" + str(currentDate.day) + "-" + str(currentDate.hour) + "-" + str(currentDate.minute) + ".csv"
	    csvFile = open(path, 'w')
	    outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
		# Write the header that defines the contents of the log file
	    header = ["timestamp", "ros_time", "robot_pos_x", "robot_pos_y", "robot_yaw_th", "robot_lin_vel_x", "robot_ang_vel_z", "person1_pos_x", "person1_pos_y", "person1_yaw_th", "person2_pos_x", "person2_pos_y", "person2_yaw_th", "person3_pos_x", "person3_pos_y", "person3_yaw_th", "person4_pos_x", "person4_pos_y", "person4_yaw_th", "person5_pos_x", "person5_pos_y", "person5_yaw_th", "scan", "distance", "distance", "distance", "distance", "distance"]
	    outputCsv.writerow(header)
            csvFile.flush()
		
		# Get the starting timestamp
	    startingTime = int(time.time())
		
		# Start showing feedback
	    feedback()
		
        except rospy.ROSInterruptException:
	    pass
