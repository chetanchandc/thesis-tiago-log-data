#!/usr/bin/env python

import rospy
import math
import datetime
import time
import os
import threading
import csv
import tf_conversions
from nav_msgs.msg import Odometry
from colorama import Fore, Back, Style


## Threading event
odometryReceived = threading.Event()
infoReceived = threading.Event()

## Odometry variable
robotPosition = None
robotLinVel = None
robotAngVel = None

## Time variable
startingTime = None
## Time variable
currentTime = None


def odometry_callback(odometryMsg):
    global robotPosition, robotLinVel, robotAngVel
	
    if not odometryReceived.is_set():
        robotPosition = odometryMsg.pose.pose.position

        robotLinVel = odometryMsg.twist.twist.linear

        robotAngVel = odometryMsg.twist.twist.angular

        # Set the flag to true
        odometryReceived.set()


def print_feedback():
    global robotPosition, robotLinVel, robotAngVel
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
    print(Style.BRIGHT + "* x: " + str(round(robotPosition.x, 2)) + "\n* y: " + str(round(robotPosition.y, 2)) + "\n* z: " + str(round(robotPosition.z, 2)) + Style.RESET_ALL)
    print
    print("The current linear velocity (in m/s) of the robot is:")
    print(Style.BRIGHT + "* Velocity in x: " + str(round(robotLinVel.x, 2)) + "\n* Velocity in y: " + str(round(robotLinVel.y, 2)) + "\n* Velocity in z: " + str(round(robotLinVel.z, 2)) + Style.RESET_ALL)
    print
    print("The current yaw angular velocity (in deg/s) of the robot is:")
    print(Style.BRIGHT + "* Rotation about the Z axis: " + str(round(robotAngVel.z, 2)) + Style.RESET_ALL)
    print


def log_feedback():
    global robotPosition, robotLinVel, robotAngVel

    currentTimestamp = [int(time.time())]

    logRobotPose = [robotPosition.x, robotPosition.y, robotPosition.z]
    logRobotVelocities = [robotLinVel.x, robotLinVel.y, robotLinVel.z, robotAngVel.x, robotAngVel.y, robotAngVel.z]

    data = currentTimestamp + logRobotPose + logRobotVelocities
    outputCsv.writerow(data)
    csvFile.flush()


def feedback():

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Clear the terminal
        os.system("clear")
    
        # Wait until all messages have been received
        odometryReceived.wait()
    
        # Log the feedback on a csv file
        log_feedback()
        
        # Print the feedback on the terminal
        print_feedback()
        
        # Clear the flags
        odometryReceived.clear()
        infoReceived.clear()
        
        rate.sleep()



if __name__ == '__main__':

	try:
	    
 	    rospy.init_node('feedback')

            rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)

		# Open the log file
	    csv_path = rospy.get_param("output_csv_path")
    	    currentDate = datetime.datetime.now()
	    path = csv_path + str(currentDate.year) + "-" + str(currentDate.month) + "-" + str(currentDate.day) + "-" + str(currentDate.hour) + "-" + str(currentDate.minute) + ".csv"
	    csvFile = open(path, 'w')
	    outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
		# Write the header that defines the contents of the log file
	    header = ["timestamp", "robot_pos_x", "robot_pos_y", "robot_pos_z", "robot_lin_vel_x", "robot_lin_vel_y", "robot_lin_vel_z", "robot_ang_vel_x", "robot_ang_vel_y", "robot_ang_vel_z"]
	    outputCsv.writerow(header)
            csvFile.flush()
		
		# Get the starting timestamp
	    startingTime = int(time.time())
		
		# Start showing feedback
	    feedback()
		
        except rospy.ROSInterruptException:
	    pass
