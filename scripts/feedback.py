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
from sensor_msgs.msg import LaserScan
from colorama import Fore, Back, Style

## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Threading event
robOdometryReceived = threading.Event()
pedInfoReceived = threading.Event()
scanReceived = threading.Event()

#ros time variable
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

#laser scan variable
objectScan = None

## Time variable
startingTime = None
currentTime = None


def odometry_callback(odometryMsg):
    global goalTime, robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
	
    if not robOdometryReceived.is_set():

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
        robOdometryReceived.set()


def pedestrians_info_callback(pedMsg):
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5
    
    # Process the message only when the previous one has been logged
    if not pedInfoReceived.is_set():
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
           pedInfoReceived.set()


def callback(scanMsg):
    global objectScan
    
    #print len(msg.ranges) 
    # 666 - in case of RGBD laser scan
    if not scanReceived.is_set():
	
 	objectScan = scanMsg.ranges[360]

        # Set the flag to true
        scanReceived.set()


def print_feedback():
    global goalTime, robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
    global objectScan
    global startingTime, currentTime
    
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
    print("The laser scan distance from robot to obstcale(any) in the range [360] is:")
    print(Style.BRIGHT + str(round(objectScan, 2)) + Style.RESET_ALL)
    print


def log_feedback():
    global goalTime, robotPosition, robotOrientationDeg, robotLinVel, robotAngVel
    global pedPosition1, pedPosition2, pedPosition3, pedPosition4, pedPosition5
    global pedOrientation1, pedOrientation2, pedOrientation3, pedOrientation4, pedOrientation5
    global objectScan

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

    data = currentTimestamp + logGoalTime + logRobotPose + logRobotVelocities + logPed1 + logPed2 + logPed3 + logPed4 + logPed5 + logScan
    outputCsv.writerow(data)
    csvFile.flush()


def feedback():

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Clear the terminal
        os.system("clear")
    
        # Wait until all messages have been received
        robOdometryReceived.wait()
        pedInfoReceived.wait()
        scanReceived.wait()
    
        # Log the feedback on a csv file
        log_feedback()
        
        # Print the feedback on the terminal
        print_feedback()
        
        # Clear the flags
        robOdometryReceived.clear()
        pedInfoReceived.clear()
        scanReceived.clear()
        
        rate.sleep()


if __name__ == '__main__':

	try:
	    
 	    rospy.init_node('feedback')
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
	    header = ["timestamp", "ros_time(in sec)", "robot_pos_x(in meters)", "robot_pos_y(in meters)", "robot_pos_th(in deg)", "robot_lin_vel_x(in m/s)", "robot_ang_vel_z(in deg/s)", "person1_pos_x(in meters)", "person1_pos_y(in meters)", "person1_pos_th(in deg)", "person2_pos_x(in meters)", "person2_pos_y(in meters)", "person2_pos_th(in deg)", "person3_pos_x(in meters)", "person3_pos_y(in meters)", "person3_pos_th(in deg)", "person4_pos_x(in meters)", "person4_pos_y(in meters)", "person4_pos_th(in deg)", "person5_pos_x(in meters)", "person5_pos_y(in meters)", "person5_pos_th(in deg)", "scan"]
	    outputCsv.writerow(header)
            csvFile.flush()
		
		# Get the starting timestamp
	    startingTime = int(time.time())
		
		# Start showing feedback
	    feedback()
		
        except rospy.ROSInterruptException:
	    pass
