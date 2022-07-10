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
from geometry_msgs.msg import *
from rospkg import RosPack
from nav_msgs.msg import Odometry
import AgentState
from colorama import Fore, Back, Style


## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Threading event
odometryReceived = threading.Event()
## Threading event
infoReceived = threading.Event()
## Threading event
#idsReceived = threading.Event()

## Odometry variable
robotPosition = None
## Odometry variable
robotLinVel = None
## Odometry variable
robotAngVelDeg = None

## Obstacle info variable
obstacleDistance = None
## Obstacle info variable
obstaclePosition = None

## Time variable
startingTime = None
## Time variable
currentTime = None


##
# @brief The callback function for the @c '/ground_truth/state' topic.
#
# @param odometryMsg The Odometry message.
def odometry_callback(odometryMsg):
    global robotPosition, robotLinVel, robotAngVelDeg
    
    # Process the message only when the previous one has been logged
    if not odometryReceived.is_set():
        robotPosition = odometryMsg.pose.pose.position
        
        robotLinVel = odometryMsg.twist.twist.linear
        
        # Convert the angular velocities to degrees per second
        robotAngVelDeg = odometryMsg.twist.twist.angular
        robotAngVelDeg.x *= radToDegRatio
        robotAngVelDeg.y *= radToDegRatio
        robotAngVelDeg.z *= radToDegRatio
        
        # Set the flag to true
        odometryReceived.set()

global xml_file

#def actor_poses_callback(actors):
    #for actor in actors.agent_states:
       # actor_id = str( actor.id )
        #actor_pose = actor.pose
        #rospy.loginfo("Spawning model: actor_id = %s", actor_id)

        #model_pose = Pose(Point(x= actor_pose.position.x,
                       #        y= actor_pose.position.y,
                        #       z= actor_pose.position.z),
                       #  Quaternion(actor_pose.orientation.x,
                         #           actor_pose.orientation.y,
                             #       actor_pose.orientation.z,
                              #      actor_pose.orientation.w) )

       # spawn_model(actor_id, xml_string, "", model_pose, "world")
    #rospy.signal_shutdown("all agents have been spawned !")


##
# @brief The callback function for the @c '/pedsim_simulator/simulated_agents' topic.
# 
# @param infoMsg The AgentState message.
def obstacle_info_callback(infoMsg):
    global obstacleDistance, obstaclePosition
    
    # Process the message only when the previous one has been logged
    if not infoReceived.is_set():
        obstacleDistance = infoMsg.distance
        obstaclePosition = infoMsg.position
        
        # Set the flag to true
        infoReceived.set()


def print_feedback():
    global robotPosition, robotLinVel, robotAngVelDeg
    global obstacleDistance, obstaclePosition
    global startingTime, currentTime
    
    # Threshold in meters after which the terminal warns the pilot
    distanceThreshold = 1.5
    
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
    print(Style.BRIGHT + "* Rotation about the Z axis: " + str(round(robotAngVelDeg.z, 2)) + Style.RESET_ALL)
    print
    
    if obstaclePosition == "none":
        print(Style.BRIGHT + Fore.YELLOW + "There are no detected obstacles at the moment." + Style.RESET_ALL)
    elif obstacleDistance <= distanceThreshold:
        print(Style.BRIGHT + Fore.RED + "The closest obstacle is situated at " + str(round(obstacleDistance, 2)) + " meters on the " + obstaclePosition + " of the robot." + Style.RESET_ALL)
    else:
        print(Style.BRIGHT + Fore.GREEN + "The closest obstacle is situated at " + str(round(obstacleDistance, 2)) + " meters on the " + obstaclePosition + " of the robot." + Style.RESET_ALL)
    print


##
# @brief This function takes all data and writes it in the log file.
def log_feedback():
    global robotPosition, robotLinVel, robotAngVelDeg
    global obstacleDistance, obstaclePosition
    
    # Create the list of all data and write it in the log file
    currentTimestamp = [int(time.time())]
    logRobotPose = [str(robotPosition.x), str(robotPosition.y), str(robotPosition.z)]
    logRobotVelocities = [str(robotLinVel.x), str(robotLinVel.y), str(robotLinVel.z), str(robotAngVelDeg.x), str(robotAngVelDeg.y), str(robotAngVelDeg.z)]
    logObstacleInfo = [str(obstacleDistance), obstaclePosition]
    
    data = currentTimestamp + logrobotPose + logrobotVelocities + logObstacleInfo
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
        #idsReceived.wait()
    
        # Log the feedback on a csv file
        log_feedback()
        
        # Print the feedback on the terminal
        print_feedback()
        
        # Clear the flags
        odometryReceived.clear()
        infoReceived.clear()
        #idsReceived.clear()
        
        rate.sleep()
        
    
##
# Main function
if __name__ == '__main__':

    try:
        # Initialize the node
        rospy.init_node('robot_feedback')
    	
    	# Subscribe to the "/mobile_base_controller/odom" topic
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)
		
    	# Subscribe to the "/pedsim_simulator/simulated_agents" topic
        rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentState, obstacle_info_callback)

		
		# Open the log file
        csv_path = rospy.get_param("output_csv_path")
        currentDate = datetime.datetime.now()
        path = csv_path + str(currentDate.year) + "-" + str(currentDate.month) + "-" + str(currentDate.day) + "-" + str(currentDate.hour) + "-" + str(currentDate.minute) + ".csv"
        csvFile = open(path, 'w')
        outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
        # Write the header that defines the contents of the log file
        header = ["timestamp", "robot_pos_x", "robote_pos_y", "robot_pos_z", "robot_roll", "robot_pitch", "robot_yaw", "robot_lin_vel_x", "robot_lin_vel_y", "robot_lin_vel_z", "robot_ang_vel_x", "robot_ang_vel_y", "robot_ang_vel_z", "person1_x", "person1_y", "person2_x", "person2_y", "person3_x", "person3_y", "closest_obs_pos"]
        outputCsv.writerow(header)
        csvFile.flush()
        
        # Get the starting timestamp
        startingTime = int(time.time())
        
        # Start showing feedback
        feedback()
        
    except rospy.ROSInterruptException:
        pass
