#!/usr/bin/env python

#        This script shows on the terminal the feedback about the robot
#        state: the displayed information comprises the robot's linear and
#        angular velocities, pose with respect to the world frame, and the
#        distance and the qualitative position of the closest obstacle. The
#        data is also logged in a csv file.

import rospy
import math
import datetime
import time
import os
import threading
import csv
import tf_conversions
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from rospkg import RosPack
from nav_msgs.msg import Odometry
from colorama import Fore, Back, Style


## Ratio to convert rad to deg
#radToDegRatio = 180 / math.pi

## Threading event
#odometryReceived = threading.Event()
## Threading event
#infoReceived = threading.Event()

## Odometry variable
robotPosition = None
## Odometry variable
robotLinVel = None
## Odometry variable
robotAngVelDeg = None

## Time variable
startingTime = None
## Time variable
currentTime = None


##
# @brief The callback function for the @c '/ground_truth/state' topic.
#
# @param odometryMsg The Odometry message.
def odometry_callback(field):
    global robotPosition, robotLinVel, robotAngVelDeg
    
    # Process the message only when the previous one has been logged
    if not odometryReceived.is_set():
        robotPosition = field.pose.pose.position
        
        robotLinVel = field.twist.twist.linear
        
        # Convert the angular velocities to degrees per second
        robotAngVelDeg = field.twist.twist.angular
        #robotAngVelDeg.x *= radToDegRatio
       
        #robotAngVelDeg.z *= radToDegRatio
        
        # Set the flag to true
        odometryReceived.set()




##
# @brief This function takes all data and writes it in the log file.
def log_feedback():
    global robotPosition, robotLinVel, robotAngVelDeg
    
    # Create the list of all data and write it in the log file
    currentTimestamp = [int(time.time())]
    logRobotPose = [str(field.pose.pose.position.x), str(field.pose.pose.position.y), str(field.pose.pose.position.z)]
    logRobotVelocities = [str(field.twist.twist.linear.x), str(field.twist.twist.angular.z)]
    
    data = currentTimestamp + logrobotPose + logrobotVelocities
    outputCsv.writerow(data)
    csvFile.flush()


def feedback():
    
    # Show feedback every second
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
        
    
##
# Main function
if __name__ == '__main__':

    try:
        # Initialize the node
        rospy.init_node('test1')
    	
    	# Subscribe to the "/mobile_base_controller/odom" topic
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)

		
		# Open the log file
        csv_path = rospy.get_param("output_csv_path")
        currentDate = datetime.datetime.now()
        path = csv_path + str(currentDate.year) + "-" + str(currentDate.month) + "-" + str(currentDate.day) + "-" + str(currentDate.hour) + "-" + str(currentDate.minute) + ".csv"
        csvFile = open(path, 'w')
        outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
        # Write the header that defines the contents of the log file
        header = ["timestamp", "robot_pos_x", "robot_pos_y", "robot_pos_z", "robot_lin_vel_x", "robot_ang_vel_z"]
        outputCsv.writerow(header)
        csvFile.flush()
        
        # Get the starting timestamp
        startingTime = int(time.time())
        
        # Start showing feedback
        feedback()
        
    except rospy.ROSInterruptException:
        pass
