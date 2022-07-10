#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import tf_conversions


def broadcast_robot_frame(msg):
	
	# Define the broadcaster and the transform message
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

	# Create the fixed orientation drone frame message
    t.header = msg.header
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

	# Broadcast the transform
    br.sendTransform(t)


# Main function
if __name__ == '__main__':

	try:
		# Initialize the node
		rospy.init_node('robot_frame_broadcaster')
    	
    	# Subscribe to the topic
		rospy.Subscriber('/mobile_base_controller/odom', nav_msgs.msg.Odometry, broadcast_robot_frame)
    
		while not rospy.is_shutdown():
            # Keep from exiting until the node is stopped
			rospy.spin()
        
	except rospy.ROSInterruptException:
		pass
