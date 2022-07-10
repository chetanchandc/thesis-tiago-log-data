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
from pedsim_msgs.msg import AgentStates
from colorama import Fore, Back, Style

## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Threading event
odometryReceived = threading.Event()
infoReceived = threading.Event()

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

## Time variable
startingTime = None
## Time variable
currentTime = None


def odometry_callback(odometryMsg):
    global robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
	
    if not odometryReceived.is_set():
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


def pedestrians_info_callback(infoMsg):
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5
    
    # Process the message only when the previous one has been logged
    if not infoReceived.is_set():
	
           pedPosition1 = infoMsg.agent_states[0].pose.position
	   pedOrientation1 = infoMsg.agent_states[0].pose.orientation
           pedPosition2 = infoMsg.agent_states[1].pose.position
	   pedOrientation2 = infoMsg.agent_states[1].pose.orientation
	   pedPosition3 = infoMsg.agent_states[2].pose.position
	   pedOrientation3 = infoMsg.agent_states[2].pose.orientation
	   pedPosition4 = infoMsg.agent_states[3].pose.position
	   pedOrientation4 = infoMsg.agent_states[3].pose.orientation
	   pedPosition5 = infoMsg.agent_states[4].pose.position
           pedOrientation5 = infoMsg.agent_states[4].pose.orientation
	
        # Set the flag to true
           infoReceived.set()


def print_feedback():
    global robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
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
    print(Style.BRIGHT + "* x: " + str(round(robotPosition.x, 2)) + "\n* y: " + str(round(robotPosition.y, 2)) + "\n* z: " + str(round(robotPosition.z, 2)) + Style.RESET_ALL)
    print
    print("The current yaw (in degrees) of the robot is:")
    print(Style.BRIGHT + "* Yaw: " + str(round(robotOrientationDeg[2], 2)) + Style.RESET_ALL)
    print
    print("The current linear velocity (in m/s) of the robot is:")
    print(Style.BRIGHT + "* Velocity in x: " + str(round(robotLinVel.x, 2)) + "\n* Velocity in y: " + str(round(robotLinVel.y, 2)) + "\n* Velocity in z: " + str(round(robotLinVel.z, 2)) + Style.RESET_ALL)
    print
    print("The current yaw angular velocity (in deg/s) of the robot is:")
    print(Style.BRIGHT + "* Rotation about the Z axis: " + str(round(robotAngVel.z, 2)) + Style.RESET_ALL)
    print


def log_feedback():
    global robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5

    currentTimestamp = [int(time.time())]

    logRobotPose = [robotPosition.x, robotPosition.y, robotPosition.z, str(robotOrientationDeg[0]), str(robotOrientationDeg[1]), str(robotOrientationDeg[2])]
    logRobotVelocities = [robotLinVel.x, robotLinVel.y, robotLinVel.z, robotAngVel.x, robotAngVel.y, robotAngVel.z]
    logPedPos1 = [pedPosition1.x, pedPosition1.y, pedPosition1.z]
    logPedOrn1 = [pedOrientation1.x, pedOrientation1.y, pedOrientation1.z]
    logPedPos2 = [pedPosition2.x, pedPosition2.y, pedPosition2.z]
    logPedOrn2 = [pedOrientation2.x, pedOrientation2.y, pedOrientation2.z]
    logPedPos3 = [pedPosition3.x, pedPosition3.y, pedPosition3.z]
    logPedOrn3 = [pedOrientation3.x, pedOrientation3.y, pedOrientation3.z]   
    logPedPos4 = [pedPosition4.x, pedPosition4.y, pedPosition4.z]
    logPedOrn4 = [pedOrientation4.x, pedOrientation4.y, pedOrientation4.z]   
    logPedPos5 = [pedPosition5.x, pedPosition5.y, pedPosition5.z]
    logPedOrn5 = [pedOrientation5.x, pedOrientation5.y, pedOrientation5.z]

    data = currentTimestamp + logRobotPose + logRobotVelocities + logPedPos1 + logPedOrn1 + logPedPos2 + logPedOrn2 + logPedPos3 + logPedOrn3 + logPedPos4 + logPedOrn4 + logPedPos5 + logPedOrn5
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
	    
 	    rospy.init_node('feedback_new')

            rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)

            rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, pedestrians_info_callback)

		# Open the log file
	    csv_path = rospy.get_param("output_csv_path")
    	    currentDate = datetime.datetime.now()
	    path = csv_path + str(currentDate.year) + "-" + str(currentDate.month) + "-" + str(currentDate.day) + "-" + str(currentDate.hour) + "-" + str(currentDate.minute) + ".csv"
	    csvFile = open(path, 'w')
	    outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
		# Write the header that defines the contents of the log file
	    header = ["timestamp", "robot_pos_x", "robot_pos_y", "robot_pos_z", "robot_roll", "robot_pitch", "robot_yaw", "robot_lin_vel_x", "robot_lin_vel_y", "robot_lin_vel_z", "robot_ang_vel_x", "robot_ang_vel_y", "robot_ang_vel_z", "1.x", "1.y", "1.z", "r1.x", "p1.y", "y1.z", "2.x", "2.y", "2.z", "r2.x", "p2.y", "y2.z", "3.x", "3.y", "3.z", "r3.x", "p3.y", "y3.z", "4.x", "4.y", "4.z", "r4.x", "p4.y", "y4.z", "5.x", "5.y", "5.z", "r5.x", "p5.y", "y5.z",]
	    outputCsv.writerow(header)
            csvFile.flush()
		
		# Get the starting timestamp
	    startingTime = int(time.time())
		
		# Start showing feedback
	    feedback()
		
        except rospy.ROSInterruptException:
	    pass
