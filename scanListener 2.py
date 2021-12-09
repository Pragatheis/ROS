
# This program prints the data-indexes of the +90 and -90 scan values based on the data in LaserScan
# data record. To do so, the program defines a 'listener()' node that subscribes to the LaserScan Topic.
# This node works based on software-interrups caused by the LIDAR scan data being published to the LaserScan
# topic. Thus, in order for this node to work, it is essential that the LIDAR is connected and the pertinent
# publisher node is launched by running the roalaunch command.
#
# DETAILS OF THE FUNCTIONS USED
# -----------------------------
# 1. listener(): essentially defines the 'lsitener' node subscribing to LaserScan topic.
# 2. scanCallback(): computes the index values corresponding to the scans at -90 and +90 degrees.
# 3. main(): initializes the lisneter node; prints the index values corresponding to -90 and +90 degnree scans.
#
# CODE REVISION HISTORY
# ---------------------------------------------------------------------------------
# Date			Revision ID 		Developper(s)		          Revision Details
# ---------------------------------------------------------------------------------
# 10/02/2018	RevA				Pawan Rao, Vishnu Nandan	  Initial Release
# --------------------------- Help Block ----------------------------------------------

#!/usr/bin/env python
import rospy		#Intent: Write the ROS program in Python
import numpy		#Intent: utilize array computation functions defined in numpy library
from sensor_msgs.msg import LaserScan		#reusing message-type LaserScan from sensor_msgs library
import math			#Intent: utilize basic math complutation functions defined in math library

flag = 0		#Global Variable Declaration & Definition
index90 = 0		#Global Variable Declaration & Definition
indexNeg90 = 0	#Global Variable Declaration & Definition

def scanCallback(msg):	#definition for scanCallback() function starts here

	global flag,index90,indexNeg90		#global variables previously defined are being used inside this function
	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', msg.angle_increment)	#used for debugging / checking
	len_ranges = len(msg.ranges)		#msg.range gives a double array whos size is computed by 'len' and stored in len_ranges
	angle_inc = msg.angle_increment * (180 / 3.14)		#converting the angle_increment to
	stepsPerDeg = 1/angle_inc 		#calculating number of steps in 1-degree scan
	zeroDegInd = round(len_ranges/2)	#calculating the index corresponding to 0-degree scan; this wilb used for calculating the -90 and +90 index
	#total_angle = round(len_ranges/stepsPerDeg)
	index90 = round(zeroDegInd + (90*stepsPerDeg))		#calculating the index for 90 degree scan and storing in the index90 (global variable)
	indexNeg90 = round(zeroDegInd + (-90*stepsPerDeg))	#calculating the index for -90 degree scan and storing in the indexNeg90 (global variable)

	#print(len(msg.ranges))
	flag = 1		#setting the value of the global variable flag to 1; this will be used in the main function to print the index value once


def listener():		#definition of the listener-node starts here

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.


		rospy.init_node('scanListener', anonymous=True)		#node declaration

		rospy.Subscriber('scan',LaserScan, scanCallback)	#this gives the topic a name (scan) and specifies the type of message that will received (LasersScan) connecting the callback() function



if __name__ == '__main__':		#definition of the main() function starts here
        listener()				#initializing the node listener()
	main_flag = 1 				#to ensure that the print happens only once
	global flag 				#calling the global flag
	while not rospy.is_shutdown():		#using a while loop to keep the node listener active

		if flag == 1 & main_flag == 1:		#will be true only for the first time after which this if condition will never be true
			print('Index for 90 degrees is at : '+str(index90))			#printing information

			print('Index for -90 degrees is at : '+str(indexNeg90))		#printing information

			flag = 0			#changing the values such that the if condition will never be executed again
			main_flag = 0		#changing the values such that the if condition will never be executed again
